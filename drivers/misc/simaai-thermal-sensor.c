#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <asm/io.h>

#define THM_BASE_SIZE		0x28
#define THM_BASE_ADDR		0x301000C0
#define THM_PMON_CTRL		0x00
#define THM_PMON_TRIM		0x04
#define THM_PMON_STATUS		0x08
#define THM_PMON_CLK_CTRL	0x0C
#define THM_PMON_DELAY_CTRL	0x10
#define THM_PMON_DATA_A		0x14
#define THM_PMON_DATA_B		0x18
#define THM_PMON_DATA		0x1C
#define THM_PMON_DAT_INTR	0x20
#define THM_PMON_DAT_INTR_EN	0x24

static ssize_t sysfs_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf);
struct kobj_attribute sys_temp_attr = __ATTR(temperature_profile, 0440,
						sysfs_show, NULL);

/*
 * Temperature mapping table
 *
 * This table maps calculated temperature sensor data with the degree celsius
 * 777 --> -40C,
 * 774 --> -39C,
 * 771 --> -38C etc
 */
int tmap[332] = { -40, 777, -39, 774, -38, 771, -37, 767, -36, 764,
		-35,  761, -34, 758, -33, 755, -32, 751, -31, 748,
		-30,  745, -29, 742, -28, 739, -27, 735, -26, 732,
		-25,  729, -24, 726, -23, 722, -22, 719, -21, 716,
		-20,  713, -19, 709, -18, 706, -17, 703, -16, 700,
		-15,  696, -14, 693, -13, 690, -12, 687, -11, 683,
		-10,  680,  -9, 677,  -8, 674,  -7, 670,  -6, 667,
		 -5,  664,  -4, 661,  -3, 658,  -2, 654,  -1, 651,
		  0,  648,   1, 645,   2, 642,   3, 638,   4, 635,
		  5,  632,   6, 629,   7, 625,   8, 622,   9, 619,
		 10,  616,  11, 612,  12, 609,  13, 606,  14, 603,
		 15,  599,  16, 596,  17, 593,  18, 590,  19, 586,
		 20,  583,  21, 580,  22, 576,  23, 573,  24, 570,
		 25,  566,  26, 563,  27, 560,  28, 556,  29, 553,
		 30,  550,  31, 546,  32, 543,  33, 540,  34, 536,
		 35,  533,  36, 530,  37, 526,  38, 523,  39, 520,
		 40,  516,  41, 513,  42, 510,  43, 506,  44, 503,
		 45,  500,  46, 496,  47, 493,  48, 490,  49, 486,
		 50,  483,  51, 480,  52, 476,  53, 473,  54, 469,
		 55,  466,  56, 463,  57, 459,  58, 456,  59, 452,
		 60,  449,  61, 446,  62, 442,  63, 439,  64, 435,
		 65,  432,  66, 429,  67, 425,  68, 422,  69, 418,
		 70,  415,  71, 412,  72, 408,  73, 405,  74, 401,
		 75,  398,  76, 395,  77, 391,  78, 388,  79, 384,
		 80,  381,  81, 378,  82, 374,  83, 371,  84, 367,
		 85,  364,  86, 361,  87, 357,  88, 354,  89, 350,
		 90,  347,  91, 344,  92, 340,  93, 337,  94, 333,
		 95,  330,  96, 327,  97, 323,  98, 320,  99, 316,
		100,  313, 101, 309, 102, 306, 103, 302, 104, 299,
		105,  295, 106, 292, 107, 288, 108, 285, 109, 281,
		110,  278, 111, 275, 112, 271, 113, 268, 114, 264,
		115,  261, 116, 257, 117, 254, 118, 250, 119, 247,
		120,  243, 121, 240, 122, 236, 123, 233, 124, 229,
		125,  226 };
u8 __iomem *thermal_regbase;

inline static u32 si_thrm_read_reg(u32 offset)
{
	return readl(thermal_regbase + offset);
}

inline static void si_thrm_write_reg(int val, u32 offset)
{
	return writel(val, thermal_regbase + offset);
}

static int si_thrm_get_temp(u8 sensor)
{
	u32 ddelta;
	u32 data_a;
	u32 data_b;
	u32 cal_val;
	u32 tmap_ind;
	u32 tmap_sz;

	ddelta = si_thrm_read_reg(THM_PMON_TRIM);
	ddelta = ddelta & 0x3FF;

	si_thrm_write_reg(0, THM_PMON_DAT_INTR_EN);
	si_thrm_write_reg(0, THM_PMON_DAT_INTR);

	si_thrm_write_reg(7, THM_PMON_DAT_INTR_EN);
	si_thrm_write_reg(0x63a880 + sensor, THM_PMON_CTRL);

	while(si_thrm_read_reg(THM_PMON_DAT_INTR) != 7);

	data_a = si_thrm_read_reg(THM_PMON_DATA_A);
	data_a = data_a & 0x3FF;
	data_b = si_thrm_read_reg(THM_PMON_DATA_B);
	data_b = data_b & 0x3FF;
	cal_val = data_b - data_a + 511 - ddelta;

	si_thrm_write_reg(0x2800, THM_PMON_CTRL);
	si_thrm_write_reg(0, THM_PMON_DAT_INTR);
	si_thrm_write_reg(0, THM_PMON_DAT_INTR_EN);

	tmap_sz = sizeof(tmap);
	for(tmap_ind = 0; tmap_ind < tmap_sz + 3; tmap_ind += 2) {
		if (cal_val < tmap[tmap_ind + 1] &&
		    cal_val >= tmap[tmap_ind + 3]) {
				return tmap[tmap_ind + 2];
		}
	}
	return -100;
}

static ssize_t sysfs_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int pos = 0;
	int temp;

	printk(KERN_INFO "Reading chip temperature");
	temp = si_thrm_get_temp(0);
	pos += sprintf(buf + pos,
		"Temperature from analog input channel select 0 is %d C\n",
		temp);

	temp = si_thrm_get_temp(1);
	pos += sprintf(buf + pos,
		"Temperature from analog input channel select 1 is %d C\n",
		temp);

	temp = si_thrm_get_temp(2);
	pos += sprintf(buf + pos,
		"Temperature from analog input channel select 2 is %d C\n",
		temp);

	temp = si_thrm_get_temp(3);
	pos += sprintf(buf + pos,
		"Temperature from analog input channel select 3 is %d C\n",
		temp);

	temp = si_thrm_get_temp(7);
	pos += sprintf(buf + pos,
		"Temperature from analog input channel select 3 is %d C\n",
		temp);
	return pos;
}

static int __init si_thrm_init(void)
{

	thermal_regbase = ioremap(THM_BASE_ADDR, THM_BASE_SIZE);
	if(sysfs_create_file(kernel_kobj, &sys_temp_attr.attr)) {
		printk("Temperature sensor sysfs file creation failed");
		return -1;
	}
	printk("Temperature sensor driver loaded successfully");

	return 0;
}

static void __exit si_thrm_exit(void)
{
	sysfs_remove_file(kernel_kobj, &sys_temp_attr.attr);
	printk("Temperature sensor driver removed successfully");
}

module_init(si_thrm_init);
module_exit(si_thrm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SiMa.ai");
MODULE_DESCRIPTION("Thermal sensor driver");
