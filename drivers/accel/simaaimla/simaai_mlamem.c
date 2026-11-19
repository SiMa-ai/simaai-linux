// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021-2026 SiMa.ai
//
// /dev/simaai-mlamem: the legacy MLA memory/register interface for
// versions of mla-rt that drive the MLA directly instead of through
// /dev/mla. Moved here from drivers/remoteproc/simaai_rproc_m4.c, keeping
// the uapi (include/uapi/linux/simaai/simaai_mlamem_ioctl.h, rev 1):
//
//   mmap               M4 SRAM and the MLA register window; root may map
//                      any physical address.
//   READ_REG/WRITE_REG the 32-bit registers listed in DT "simaai,reg-list",
//                      addressed by index; 0xffffffff entries are holes.
//   GET_REV            driver revision.
//   RESET_MLA          the PRC mla_ck_rst sequence: M4 into reset, MLA
//                      reset cycled with its clocks enabled.
//
// This is not a separate platform driver: mla_probe() calls
// simaai_mlamem_init() and mla_remove() calls simaai_mlamem_exit(). Each
// open holds a runtime-PM reference on the MLA device, so its gate clock
// stays on while userspace has the registers mapped.
//
// "simaai,reg-list" lives on the mla node. Older DTBs carried it on the m4
// remoteproc node instead, so that node is still consulted as a fallback.

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <uapi/linux/simaai/simaai_mlamem_ioctl.h>

#include "mla_drv.h"
#include "mla_regs.h"

#define SIMAAI_MLAMEM_REV		1
#define SIMAAI_MLAMEM_DEV_NAME		"simaai-mlamem"
#define SIMAAI_MLAMEM_REG_LIST		"simaai,reg-list"
#define SIMAAI_MLAMEM_M4_COMPAT		"simaai,modalix-m4-rproc"
#define SIMAAI_MLAMEM_REG_HOLE		0xffffffffu

/* M4 SRAM, mappable by non-root users. */
#define SIMAAI_MLAMEM_M4_SRAM_BASE	0x05100000
#define SIMAAI_MLAMEM_M4_SRAM_SIZE	0x00080000

/* mla_ck_rst masks used by RESET_MLA. Modalix has no active-high reset
 * bit; the sequence still toggles bit 31, which is unused.
 */
#define SIMAAI_MLAMEM_CLK_EN_MASK	(MLA_CK_RST_MLA_CLK_EN | MLA_CK_RST_MLA_DBG_CLK_EN)
#define SIMAAI_MLAMEM_MLA_RESET_MASK	(MLA_CK_RST_MLA_XXX_RESET_N | MLA_CK_RST_MLA_DBG_RSTN)
#define SIMAAI_MLAMEM_M4_RESET_MASK	(MLA_CK_RST_MLA_UCTRL_SYS_RST | \
					 MLA_CK_RST_MLA_UCTRL_POR_RST)
#define SIMAAI_MLAMEM_RESET_BIT		BIT(31)

struct simaai_mlamem {
	struct miscdevice	miscdev;
	struct device		*dev;		/* the MLA platform device */
	int			reg_list_size;
	void __iomem		**mapped_regs;	/* NULL for a hole */
	phys_addr_t		mla_base;
	size_t			mla_size;	/* page-rounded for mmap */
};

static void simaai_mlamem_unmap_regs(struct simaai_mlamem *mlamem)
{
	int i;

	for (i = 0; i < mlamem->reg_list_size; i++)
		if (mlamem->mapped_regs[i])
			iounmap(mlamem->mapped_regs[i]);

	kfree(mlamem->mapped_regs);
	mlamem->mapped_regs = NULL;
}

/* Read "simaai,reg-list" from @np and ioremap each register. Returns
 * -EINVAL if @np has no such property.
 */
static int simaai_mlamem_map_regs(struct simaai_mlamem *mlamem, struct device_node *np)
{
	u32 *hw_reg_list;
	int n, i, ret;

	n = of_property_count_u32_elems(np, SIMAAI_MLAMEM_REG_LIST);
	if (n < 0)
		return n;

	hw_reg_list = kcalloc(n, sizeof(*hw_reg_list), GFP_KERNEL);
	mlamem->mapped_regs = kcalloc(n, sizeof(*mlamem->mapped_regs), GFP_KERNEL);
	if (!hw_reg_list || !mlamem->mapped_regs) {
		ret = -ENOMEM;
		goto err;
	}

	ret = of_property_read_u32_array(np, SIMAAI_MLAMEM_REG_LIST, hw_reg_list, n);
	if (ret)
		goto err;

	mlamem->reg_list_size = n;
	for (i = 0; i < n; i++) {
		if (hw_reg_list[i] == SIMAAI_MLAMEM_REG_HOLE)
			continue;

		mlamem->mapped_regs[i] = ioremap(hw_reg_list[i], 4);
		if (!mlamem->mapped_regs[i]) {
			dev_err(mlamem->dev, "%s: ioremap failed for 0x%x\n",
				SIMAAI_MLAMEM_DEV_NAME, hw_reg_list[i]);
			ret = -EIO;
			goto err;
		}
	}

	kfree(hw_reg_list);
	return 0;

err:
	if (mlamem->mapped_regs)
		simaai_mlamem_unmap_regs(mlamem);
	mlamem->reg_list_size = 0;
	kfree(hw_reg_list);
	return ret;
}

static bool simaai_mlamem_range_ok(struct simaai_mlamem *mlamem, unsigned long pfn, size_t size)
{
	phys_addr_t start = (phys_addr_t)pfn << PAGE_SHIFT;
	phys_addr_t end = start + size - 1;

	if (start >= SIMAAI_MLAMEM_M4_SRAM_BASE &&
	    end < SIMAAI_MLAMEM_M4_SRAM_BASE + SIMAAI_MLAMEM_M4_SRAM_SIZE)
		return true;
	if (start >= mlamem->mla_base && end < mlamem->mla_base + mlamem->mla_size)
		return true;

	return false;
}

static int simaai_mlamem_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct simaai_mlamem *mlamem = filep->private_data;
	size_t size = vma->vm_end - vma->vm_start;
	phys_addr_t offset = (phys_addr_t)vma->vm_pgoff << PAGE_SHIFT;

	/* Does it even fit in phys_addr_t? */
	if (offset >> PAGE_SHIFT != vma->vm_pgoff)
		return -EINVAL;

	/* It's illegal to wrap around the end of the physical address space. */
	if (offset + (phys_addr_t)size - 1 < offset)
		return -EINVAL;

	/* Only root may map outside the M4 SRAM and the MLA window. */
	if (!capable(CAP_SYS_ADMIN) &&
	    !simaai_mlamem_range_ok(mlamem, vma->vm_pgoff, size)) {
		dev_err(mlamem->dev, "%s: mmap() error: cannot mmap offset = %pa size = %zu\n",
			SIMAAI_MLAMEM_DEV_NAME, &offset, size);
		return -EPERM;
	}

	vma->vm_page_prot = phys_mem_access_prot(filep, vma->vm_pgoff, size,
						 vma->vm_page_prot);

	/* remap_pfn_range() marks the range VM_IO. */
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int simaai_mlamem_reset_mla(struct simaai_mlamem *mlamem)
{
	void __iomem *prc;
	u32 val;

	prc = ioremap(MLA_PRC_REGS_BASE, MLA_PRC_REGS_SIZE);
	if (!prc)
		return -ENOMEM;

	val = readl(prc + MLA_PRC_MLA_CK_RST_OFFSET);

	/* Put M4 into reset */
	val |= SIMAAI_MLAMEM_M4_RESET_MASK;

	/* Stop MLA */
	val &= ~SIMAAI_MLAMEM_MLA_RESET_MASK;
	val |= SIMAAI_MLAMEM_RESET_BIT;
	writel(val, prc + MLA_PRC_MLA_CK_RST_OFFSET);

	udelay(1);

	/* Enable MLA clocks */
	val |= SIMAAI_MLAMEM_CLK_EN_MASK;
	writel(val, prc + MLA_PRC_MLA_CK_RST_OFFSET);

	udelay(1);

	/* Start MLA */
	val |= SIMAAI_MLAMEM_MLA_RESET_MASK;
	val &= ~SIMAAI_MLAMEM_RESET_BIT;
	writel(val, prc + MLA_PRC_MLA_CK_RST_OFFSET);

	iounmap(prc);
	return 0;
}

static long simaai_mlamem_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	struct simaai_mlamem *mlamem = filep->private_data;
	void __user *argp = (void __user *)arg;
	struct simaai_reg_info rargs = {
		.rev = SIMAAI_MLAMEM_REV
	};
	int i;

	switch (cmd) {
	case SIMAAI_IOC_WRITE_REG:
	case SIMAAI_IOC_READ_REG:
		if (copy_from_user(&rargs, argp, sizeof(rargs)))
			return -EFAULT;

		if (rargs.num_of_regs == 0 ||
		    rargs.num_of_regs > ARRAY_SIZE(rargs.reg_idx))
			return -EINVAL;

		/* Validate every index before touching any register. */
		for (i = 0; i < rargs.num_of_regs; i++) {
			if (rargs.reg_idx[i] >= mlamem->reg_list_size ||
			    !mlamem->mapped_regs[rargs.reg_idx[i]])
				return -EINVAL;
		}

		for (i = 0; i < rargs.num_of_regs; i++) {
			void __iomem *addr = mlamem->mapped_regs[rargs.reg_idx[i]];

			if (cmd == SIMAAI_IOC_WRITE_REG)
				writel(rargs.reg_value[i], addr);
			else
				rargs.reg_value[i] = readl(addr);
		}

		if (cmd == SIMAAI_IOC_READ_REG &&
		    copy_to_user(argp, &rargs, sizeof(rargs)))
			return -EFAULT;
		return 0;

	case SIMAAI_IOC_GET_REV:
		if (copy_to_user(argp, &rargs, sizeof(rargs)))
			return -EFAULT;
		return 0;

	case SIMAAI_IOC_RESET_MLA:
		return simaai_mlamem_reset_mla(mlamem);

	default:
		dev_err(mlamem->dev, "%s: bad ioctl cmd %u\n", SIMAAI_MLAMEM_DEV_NAME, cmd);
		return -EINVAL;
	}
}

static int simaai_mlamem_open(struct inode *inode, struct file *filep)
{
	struct simaai_mlamem *mlamem = container_of(filep->private_data,
						struct simaai_mlamem, miscdev);
	int ret;

	filep->private_data = mlamem;
	ret = pm_runtime_resume_and_get(mlamem->dev);
	return ret < 0 ? ret : 0;
}

static int simaai_mlamem_release(struct inode *inode, struct file *filep)
{
	struct simaai_mlamem *mlamem = filep->private_data;

	pm_runtime_put(mlamem->dev);
	return 0;
}

static const struct file_operations simaai_mlamem_fops = {
	.owner		= THIS_MODULE,
	.mmap		= simaai_mlamem_mmap,
	.unlocked_ioctl	= simaai_mlamem_ioctl,
	.open		= simaai_mlamem_open,
	.llseek		= noop_llseek,
	.release	= simaai_mlamem_release,
};

/**
 * simaai_mlamem_init() - publish /dev/simaai-mlamem for the MLA device
 * @mdev: the probed MLA device
 *
 * Return: 0 on success or a negative errno. A failure leaves /dev/mla
 * unaffected; the caller only warns.
 */
int simaai_mlamem_init(struct mla_device *mdev)
{
	struct platform_device *pdev = to_platform_device(mdev->dev);
	struct simaai_mlamem *mlamem;
	struct device_node *np;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	mlamem = kzalloc(sizeof(*mlamem), GFP_KERNEL);
	if (!mlamem)
		return -ENOMEM;

	mlamem->dev = mdev->dev;
	mlamem->mla_base = res->start;
	mlamem->mla_size = PAGE_ALIGN(resource_size(res));

	/* The mla node first; the m4 node only for DTBs that predate the move. */
	np = of_node_get(mdev->dev->of_node);
	ret = simaai_mlamem_map_regs(mlamem, np);
	if (ret == -EINVAL) {
		of_node_put(np);
		np = of_find_compatible_node(NULL, NULL, SIMAAI_MLAMEM_M4_COMPAT);
		ret = np ? simaai_mlamem_map_regs(mlamem, np) : -ENODEV;
	}
	if (ret) {
		dev_err(mlamem->dev, "%s: no usable %s (%d)\n", SIMAAI_MLAMEM_DEV_NAME,
			SIMAAI_MLAMEM_REG_LIST, ret);
		goto err_free;
	}

	mlamem->miscdev.minor  = MISC_DYNAMIC_MINOR;
	mlamem->miscdev.name   = SIMAAI_MLAMEM_DEV_NAME;
	mlamem->miscdev.fops   = &simaai_mlamem_fops;
	mlamem->miscdev.parent = mdev->dev;

	ret = misc_register(&mlamem->miscdev);
	if (ret) {
		dev_err(mlamem->dev, "%s: misc_register failed (%d)\n",
			SIMAAI_MLAMEM_DEV_NAME, ret);
		goto err_unmap;
	}

	dev_info(mlamem->dev, "%s: %d regs from %pOF\n", SIMAAI_MLAMEM_DEV_NAME,
		 mlamem->reg_list_size, np);
	of_node_put(np);
	mdev->mlamem = mlamem;
	return 0;

err_unmap:
	simaai_mlamem_unmap_regs(mlamem);
err_free:
	of_node_put(np);
	kfree(mlamem);
	return ret;
}

/**
 * simaai_mlamem_exit() - remove /dev/simaai-mlamem
 * @mdev: the MLA device; a no-op if simaai_mlamem_init() did not succeed
 */
void simaai_mlamem_exit(struct mla_device *mdev)
{
	struct simaai_mlamem *mlamem = mdev->mlamem;

	if (!mlamem)
		return;

	misc_deregister(&mlamem->miscdev);
	simaai_mlamem_unmap_regs(mlamem);
	kfree(mlamem);
	mdev->mlamem = NULL;
}
