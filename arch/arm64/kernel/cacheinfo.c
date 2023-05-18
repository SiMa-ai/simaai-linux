// SPDX-License-Identifier: GPL-2.0-only
/*
 *  ARM64 cacheinfo support
 *
 *  Copyright (C) 2015 ARM Ltd.
 *  All Rights Reserved
 */

#include <linux/acpi.h>
#include <linux/cacheinfo.h>
#include <linux/of.h>

#define MAX_CACHE_LEVEL			7	/* Max 7 level supported */
/* Ctypen, bits[3(n - 1) + 2 : 3(n - 1)], for n = 1 to 7 */
#define CLIDR_CTYPE_SHIFT(level)	(3 * (level - 1))
#define CLIDR_CTYPE_MASK(level)		(7 << CLIDR_CTYPE_SHIFT(level))
#define CLIDR_CTYPE(clidr, level)	\
	(((clidr) & CLIDR_CTYPE_MASK(level)) >> CLIDR_CTYPE_SHIFT(level))

/*
 * NumSets, bits[27:13] - (Number of sets in cache) - 1
 * Associativity, bits[12:3] - (Associativity of cache) - 1
 * LineSize, bits[2:0] - (Log2(Number of words in cache line)) - 2
 */
#define CCSIDR_EL1_WRITE_THROUGH	BIT(31)
#define CCSIDR_EL1_WRITE_BACK		BIT(30)
#define CCSIDR_EL1_READ_ALLOCATE	BIT(29)
#define CCSIDR_EL1_WRITE_ALLOCATE	BIT(28)
#define CCSIDR_EL1_LINESIZE_MASK	0x7
#define CCSIDR_EL1_LINESIZE(x)		((x) & CCSIDR_EL1_LINESIZE_MASK)
#define CCSIDR_EL1_ASSOCIATIVITY_SHIFT	3
#define CCSIDR_EL1_ASSOCIATIVITY_MASK	0x3ff
#define CCSIDR_EL1_ASSOCIATIVITY(x)	\
	(((x) >> CCSIDR_EL1_ASSOCIATIVITY_SHIFT) & CCSIDR_EL1_ASSOCIATIVITY_MASK)
#define CCSIDR_EL1_NUMSETS_SHIFT	13
#define CCSIDR_EL1_NUMSETS_MASK		0x7fff
#define CCSIDR_EL1_NUMSETS(x) \
	(((x) >> CCSIDR_EL1_NUMSETS_SHIFT) & CCSIDR_EL1_NUMSETS_MASK)

#define CACHE_LINESIZE(x)	(16 << CCSIDR_EL1_LINESIZE(x))
#define CACHE_NUMSETS(x)	(CCSIDR_EL1_NUMSETS(x) + 1)
#define CACHE_ASSOCIATIVITY(x)	(CCSIDR_EL1_ASSOCIATIVITY(x) + 1)

int cache_line_size(void)
{
	if (coherency_max_size != 0)
		return coherency_max_size;

	return cache_line_size_of_cpu();
}
EXPORT_SYMBOL_GPL(cache_line_size);

static inline enum cache_type get_cache_type(int level)
{
	u64 clidr;

	if (level > MAX_CACHE_LEVEL)
		return CACHE_TYPE_NOCACHE;
	clidr = read_sysreg(clidr_el1);
	return CLIDR_CTYPE(clidr, level);
}

/*
 * Cache Size Selection Register(CSSELR) selects which Cache Size ID
 * Register(CCSIDR) is accessible by specifying the required cache
 * level and the cache type. We need to ensure that no one else changes
 * CSSELR by calling this in non-preemtible context
 */
u64 __attribute_const__ cache_get_ccsidr(u64 csselr)
{
	u64 ccsidr;

	/* Put value into CSSELR */
	asm volatile("msr csselr_el1, %x0" : : "r" (csselr));
	isb();
	/* Read result out of CCSIDR */
	asm volatile("mrs %x0, ccsidr_el1" : "=r" (ccsidr));

	return ccsidr;
}

static void ci_leaf_init(struct cacheinfo *this_leaf,
			 enum cache_type type, unsigned int level)
{
	bool is_icache = type & CACHE_TYPE_INST;
	u64 tmp = cache_get_ccsidr((level - 1) << 1 | is_icache);

	this_leaf->level = level;
	this_leaf->type = type;
	this_leaf->coherency_line_size = CACHE_LINESIZE(tmp);
	this_leaf->number_of_sets = CACHE_NUMSETS(tmp);
	this_leaf->ways_of_associativity = CACHE_ASSOCIATIVITY(tmp);
	this_leaf->size = this_leaf->number_of_sets *
	    this_leaf->coherency_line_size * this_leaf->ways_of_associativity;
	this_leaf->attributes =
		((tmp & CCSIDR_EL1_WRITE_THROUGH) ? CACHE_WRITE_THROUGH : 0) |
		((tmp & CCSIDR_EL1_WRITE_BACK) ? CACHE_WRITE_BACK : 0) |
		((tmp & CCSIDR_EL1_READ_ALLOCATE) ? CACHE_READ_ALLOCATE : 0) |
		((tmp & CCSIDR_EL1_WRITE_ALLOCATE) ? CACHE_WRITE_ALLOCATE : 0);
}

int init_cache_level(unsigned int cpu)
{
	unsigned int ctype, level, leaves;
	int fw_level;
	struct cpu_cacheinfo *this_cpu_ci = get_cpu_cacheinfo(cpu);

	for (level = 1, leaves = 0; level <= MAX_CACHE_LEVEL; level++) {
		ctype = get_cache_type(level);
		if (ctype == CACHE_TYPE_NOCACHE) {
			level--;
			break;
		}
		/* Separate instruction and data caches */
		leaves += (ctype == CACHE_TYPE_SEPARATE) ? 2 : 1;
	}

	if (acpi_disabled)
		fw_level = of_find_last_cache_level(cpu);
	else
		fw_level = acpi_find_last_cache_level(cpu);

	if (fw_level < 0)
		return fw_level;

	if (level < fw_level) {
		/*
		 * some external caches not specified in CLIDR_EL1
		 * the information may be available in the device tree
		 * only unified external caches are considered here
		 */
		leaves += (fw_level - level);
		level = fw_level;
	}

	this_cpu_ci->num_levels = level;
	this_cpu_ci->num_leaves = leaves;
	return 0;
}

int populate_cache_leaves(unsigned int cpu)
{
	unsigned int level, idx;
	enum cache_type type;
	struct cpu_cacheinfo *this_cpu_ci = get_cpu_cacheinfo(cpu);
	struct cacheinfo *this_leaf = this_cpu_ci->info_list;

	for (idx = 0, level = 1; level <= this_cpu_ci->num_levels &&
	     idx < this_cpu_ci->num_leaves; idx++, level++) {
		type = get_cache_type(level);
		if (type == CACHE_TYPE_SEPARATE) {
			ci_leaf_init(this_leaf++, CACHE_TYPE_DATA, level);
			ci_leaf_init(this_leaf++, CACHE_TYPE_INST, level);
		} else {
			ci_leaf_init(this_leaf++, type, level);
		}
	}
	return 0;
}
