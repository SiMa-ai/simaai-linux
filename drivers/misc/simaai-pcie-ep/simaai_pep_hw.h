// SPDX-License-Identifier: GPL-2.0
/**
 * Platform PCIe EP driver for SiMa.ai Davinci SoCs
 * HW Registers
 *
 * Copyright (C) 2021-2022 SiMa.ai
 * Author:
 */

#ifndef _SIMA_PEP_HW_H_
#define _SIMA_PEP_HW_H_

#include <asm/io.h>
#include <linux/bitfield.h>

#define SI_PEP_WORK_DELAY (25)

#define SI_PEP_NUM_INTERRUPTS (19)

/* VSEC DMA Extended capability offset in configuration space */
#define SI_PCI_EXT_CAP_OFFSET_VSECDMA 0x280
/* PORT LOGIC registers block offset in configuration space */
#define SI_PCI_PF0_PORT_LOGIC 0x700

#define SI_PF0_DMA_REGS 0x280000
/* Address spacing between consecutive r/w channel pair */
#define SI_DMA_INTERCH_SPACING 0x200
/* Address offset of write channel in r/w channel pair */
#define SI_DMA_WCH_OFFSET 0x0
/* Address offset of  read channel in r/w channel pair */
#define SI_DMA_RCH_OFFSET 0x100

/* GLUE LOGIC registers offsets definition (glue_logic) */
#define GL_PCIE_CTRL_1 0x4
#define GL_PCIE_CTRL_1_LTSSM_ENABLE BIT(7)
#define GL_PCIE_CTRL_1_HOLD_PHY_RST BIT(4)
#define GL_PCIE_CTRL_AXIM_IB_ADDR_MAP 0x404
#define GL_PCIE_CTRL_AXIM_IB_ADDR_MAP_BASE_ADDR_MASK GENMASK(15, 8)
#define GL_PCIE_CTRL_AXIM_IB_ADDR_MAP_BASE_ADDR(n) \
	FIELD_PREP(GL_PCIE_CTRL_AXIM_IB_ADDR_MAP_BASE_ADDR_MASK, n)
#define GL_PCIE_CTRL_AXIM_IB_ADDR_MAP_BASE_MASK_MASK GENMASK(7, 0)
#define GL_PCIE_CTRL_AXIM_IB_ADDR_MAP_BASE_MASK(n) \
	FIELD_PREP(GL_PCIE_CTRL_AXIM_IB_ADDR_MAP_BASE_MASK_MASK, n)

/* PORT LOGIC registers offsets definition (SIMA_PF0_PORT_LOGIC) */
#define PL_PCIE_VERSION_NUMBER 0x1F8 /* PORT LOGIC register */
#define PL_PCIE_VERSION_TYPE 0x1FC /*PORT LOGIC register */
#define PL_PCIE_MISC_CONTROL_1 0x1BC /* PORT LOGIC register */
#define PL_PCIE_MISC_CONTROL_1_DBI_RO_WR_EN BIT(0)

/* BAR MASK shadow registers in dbi2 block (SIMA_PF0_TYPE0_HDR_DBI2)*/
#define BAR0_MASK_REG 0x10
#define BAR1_MASK_REG 0x14
#define BAR2_MASK_REG 0x18
#define BAR3_MASK_REG 0x1C
#define BAR4_MASK_REG 0x20
#define BAR5_MASK_REG 0x24
#define EXP_ROM_BAR_MASK_REG 0x30

/* VSEC DMA Extended capability registers (SIMA_PCI_EXT_CAP_OFFSET_VSECDMA)*/
#define VSECDMA_EXT_CAP_HDR 0x00
#define VSECDMA_VENDOR_SPECIFIC_HDR 0x04
#define VSECDMA_DEVICE_INFORMATION 0x08
#define VSECDMA_NUM_CHAN 0x0C
#define VSECDMA_UNROLL_ADDR_OFFSET_LOW 0x10
#define VSECDMA_UNROLL_ADDR_OFFSET_HIGH 0x14

#define DMA_EN_ENABLE BIT(0)

#define DMA_DOORBELL_DB_STOP BIT(1)
#define DMA_DOORBELL_DB_START BIT(0)

#define DMA_STATUS_STATUS_MASK GENMASK(2, 0)
#define DMA_STATUS_RESERVED 0x0
#define DMA_STATUS_RUNNING 0x1
#define DMA_STATUS_ABORTED 0x2
#define DMA_STATUS_STOPPED 0x3

#define DMA_INT_ST_ERROR_MASK GENMASK(6, 3)
#define DMA_INT_ST_ERROR_CLEAR (0x0 << 3)
#define DMA_INT_ST_ERROR_DATA_CPL_CA (0xa << 3)
#define DMA_INT_ST_ERROR_DATA_CPL_EP (0xb << 3)
#define DMA_INT_ST_ERROR_DATA_CPL_TIMEOUT (0x8 << 3)
#define DMA_INT_ST_ERROR_DATA_CPL_UR (0x9 << 3)
#define DMA_INT_ST_ERROR_DATA_MWR (0xc << 3)
#define DMA_INT_ST_ERROR_LL_CPL_CA (0x2 << 3)
#define DMA_INT_ST_ERROR_LL_CPL_EP (0x3 << 3)
#define DMA_INT_ST_ERROR_LL_CPL_UR (0x1 << 3)
#define DMA_INT_ST_ABORT BIT(2)
#define DMA_INT_ST_WATERMARK BIT(1)
#define DMA_INT_ST_STOP BIT(0)
#define DMA_INT_ST_INTS_MASK GENMASK(2, 0)

#define DMA_INT_SETUP_LAIE BIT(6)
#define DMA_INT_SETUP_RAIE BIT(5)
#define DMA_INT_SETUP_LSIE BIT(4)
#define DMA_INT_SETUP_RSIE BIT(3)
#define DMA_INT_SETUP_ABORT_MASK BIT(2)
#define DMA_INT_SETUP_WATERMARK_MASK BIT(1)
#define DMA_INT_SETUP_STOP_MASK BIT(0)
#define DMA_INT_SETUP_INTS_MASK GENMASK(2, 0)

#define DMA_INT_CLEAR_ABORT BIT(2)
#define DMA_INT_CLEAR_WATERMARK BIT(1)
#define DMA_INT_CLEAR_STOP BIT(0)
#define DMA_INT_CLEAR_INTS_MASK GENMASK(2, 0)

/* DMA Write Channel registers, 0x100 bytes */
struct si_dma_wrch_regs {
	volatile u32 en; /* 0x00 Enable */
	volatile u32 doorbell; /* 0x04 Doorbell */
	volatile u32 elem_pf; /* 0x08 Prefetch  */
	volatile u32 handshake; /* 0x0C Handshake  */
	volatile u32 llp_low; /* 0x10 Linked List Pointer Low  */
	volatile u32 llp_high; /* 0x14 Linked List Pointer High  */
	volatile u32 cycle; /* 0x18 Producer-Consumer Cycle Synchronization  */
	volatile u32 xfersize; /* 0x1C Transfer Size  */
	volatile u32 sar_low; /* 0x20 SAR Low  */
	volatile u32 sar_high; /* 0x24 SAR High  */
	volatile u32 dar_low; /* 0x28 DAR Low  */
	volatile u32 dar_high; /* 0x2C DAR High  */
	volatile u32 watermark_en; /* 0x30 Linked-list Watermark Enable  */
	volatile u32 control1; /* 0x34 Control Settings 1  */
	volatile u32 func_num; /* 0x38 Function Number  */
	volatile u32 qos; /* 0x3C QoS Settings  */
	volatile u32 padding_0[16]; /* [0x40..0x7C] */
	volatile u32 status; /* 0x80 Status  */
	volatile u32 int_status; /* 0x84 Interrupt Status  */
	volatile u32 int_setup; /* 0x88 Interrupt Setup  */
	volatile u32 int_clear; /* 0x8C Interrupt Clear  */
	volatile u32 msi_stop_low; /* 0x90 Read Stop Remote Interrupt addr Low */
	volatile u32
		msi_stop_high; /* 0x94 Read Stop Remote Interrupt addr High */
	volatile u32
		watermark_low; /* 0x98 Read Watermark Remote Interrupt addr Low */
	volatile u32
		watermark_high; /* 0x9C Read Watermark Remote Interrupt addr High */
	volatile u32
		msi_abort_low; /* 0xA0 Read Abort Remote Interrupt addr Low */
	volatile u32
		msi_abort_high; /* 0xA4 Read Abort Remote Interrupt addr High */
	volatile u32 msi_msgd; /* 0xA8 Remote Interrupt Data  */
	volatile u32 padding_1[1 + 20]; /* [0xAC..0xFC] */
} __packed;

/* DMA Read Channel registers, 0x100 bytes */
struct si_dma_rdch_regs {
	volatile u32 en; /* 0x000 Enable */
	volatile u32 doorbell; /* 0x004 Doorbell */
	volatile u32 elem_pf; /* 0x008 Prefetch */
	volatile u32 handshake; /* 0x00C Handshake */
	volatile u32 llp_low; /* 0x010 Linked List Pointer Low */
	volatile u32 llp_high; /* 0x014 Linked List Pointer High */
	volatile u32 cycle; /* 0x018 Producer-Consumer Cycle Synchronization */
	volatile u32 xfersize; /* 0x01C Transfer Size */
	volatile u32 sar_low; /* 0x020 SAR Low */
	volatile u32 sar_high; /* 0x024 SAR High */
	volatile u32 dar_low; /* 0x028 DAR Low */
	volatile u32 dar_high; /* 0x02C DAR High */
	volatile u32 watermark_en; /* 0x030 Linked-list Watermark Enable */
	volatile u32 control1; /* 0x034 Control Settings 1 */
	volatile u32 func_num; /* 0x038 Function Number */
	volatile u32 qos; /* 0x03C QoS Settings */
	volatile u32 padding_0[16]; /* [0x40..0x7C] */
	volatile u32 status; /* 0x080 Status */
	volatile u32 int_status; /* 0x084 Interrupt Status */
	volatile u32 int_setup; /* 0x088 Interrupt Setup */
	volatile u32 int_clear; /* 0x08C Interrupt Clear */
	volatile u32 msi_stop_low; /* 0x090 Stop Remote Interrupt Address Low */
	volatile u32 msi_stop_high; /* 0x094 Stop Remote Interrupt Address High */
	volatile u32
		watermark_low; /* 0x098 Watermark Remote Interrupt Address Low */
	volatile u32
		watermark_high; /* 0x09C Watermark Remote Interrupt Address High */
	volatile u32 msi_abort_low; /* 0x0A0 Abort Remote Interrupt Address Low */
	volatile u32
		msi_abort_high; /* 0x0A4 Abort Remote Interrupt Address High */
	volatile u32 msi_msgd; /* 0x0A8 Remote Interrupt Data */
	volatile u32 padding_1[1 + 20]; /* [0xAC..0xFC] */
} __packed;

enum si_dma_dir {
	DIR_WRITE = 0, /* Local memory to Host memory */
	DIR_READ, /* Host memory to Local memory */
	DIR_MAX
};

#define DMA_REG_OFFSET(pep, ch, dir, reg)                                     \
	(dir == DIR_WRITE) ?                                                  \
		(&((struct si_dma_wrch_regs *)si_pep_dma_regs(pep, ch, dir))  \
			  ->reg) :                                            \
		(&((struct si_dma_rdch_regs *)si_pep_dma_regs(pep, ch, dir))  \
			  ->reg)

#define DMA_REG_WRITE(pep, ch, dir, reg_name, value)                           \
	writel(value,                                                          \
	       (DIR_WRITE == dir) ?                                            \
		       (&((struct si_dma_wrch_regs *)si_pep_dma_regs(pep, ch,  \
								     dir))     \
				 ->reg_name) :                                 \
		       (&((struct si_dma_rdch_regs *)si_pep_dma_regs(pep, ch,  \
								     dir))     \
				 ->reg_name))

#define DMA_REG_READ(pep, ch, dir, reg_name)                                  \
	readl((DIR_WRITE == dir) ?                                            \
		      (&((struct si_dma_wrch_regs *)si_pep_dma_regs(pep, ch,  \
								    dir))     \
				->reg_name) :                                 \
		      (&((struct si_dma_rdch_regs *)si_pep_dma_regs(pep, ch,  \
								    dir))     \
				->reg_name))

#define CYCLE_BIT	BIT(0)
#define TCB_BIT		BIT(1)
#define LLP_ELEMENT	BIT(2)
#define LWIE_BIT	BIT(3)
#define RWIE_BIT	BIT(4)

struct si_pep_dma_ll_delem {
	u32 flags;
	u32 xfer_size;
	u32 sar_low;
	u32 sar_high;
	u32 dar_low;
	u32 dar_high;
} __packed;

struct si_pep_dma_ll_lelem {
	u32 flags;
	u32 _pad1;
	u32 ll_ptr_low;
	u32 ll_ptr_high;
	u32 _pad2;
	u32 _pad3;
} __packed;

struct si_pep_dma_ll_ctx {
	enum si_dma_dir dir;
	struct si_pep_dma_ll_delem *dma_ll;
	dma_addr_t p_dma_ll;
	size_t size;
	size_t nents;
	size_t tsize;
};

#endif /* _SIMA_PEP_HW_H_ */
