/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SCHED_DEBUG_H
#define _LINUX_SCHED_DEBUG_H

/*
 * Various scheduler/task debugging interfaces:
 */

struct task_struct;
struct pid_namespace;

extern void dump_cpu_task(int cpu);

/*
 * Only dump TASK_* tasks. (0 for all tasks)
 */
extern void show_state_filter(unsigned int state_filter);

static inline void show_state(void)
{
	show_state_filter(0);
}

struct pt_regs;

extern void show_regs(struct pt_regs *);

/*
 * TASK is a pointer to the task whose backtrace we want to see (or NULL for current
 * task), SP is the stack pointer of the first frame that should be shown in the back
 * trace (or NULL if the entire call-chain of the task should be shown).
 */
extern void show_stack(struct task_struct *task, unsigned long *sp,
		       const char *loglvl);

extern void sched_show_task(struct task_struct *p);

#ifdef CONFIG_SCHED_DEBUG
struct seq_file;
extern void proc_sched_show_task(struct task_struct *p,
				 struct pid_namespace *ns, struct seq_file *m);
extern void proc_sched_set_task(struct task_struct *p);
#endif

/* Attach to any functions which should be ignored in wchan output. */
#define __sched		__section(".sched.text")

/* Linker adds these: start and end of __sched functions */
extern char __sched_text_start[], __sched_text_end[];

/* Is this address in the __sched functions? */
extern int in_sched_functions(unsigned long addr);

#ifdef CONFIG_EDAC
extern void edac_dump_device_stats(void);
#endif

#ifdef CONFIG_ARM_GIC_V3
extern void gic_dump_irq_status(void);
#endif

#ifdef CONFIG_ARM64
#include <asm/sysreg.h>
#include <linux/printk.h>
static inline void arm64_dump_err_status(void)
{
	u64 val, errselr;
	u64 i;

	pr_info("Dumping ARM Cortex-A65 ERRXSTATUS registers\n");

	errselr = read_sysreg_s(SYS_ERRSELR_EL1);
	for(i = 0; i < 2; i++) {
		write_sysreg_s((errselr & (~0xffff)) | i, SYS_ERRSELR_EL1);
		isb();
		val = read_sysreg_s(SYS_ERXSTATUS_EL1);
		pr_info("ERR%lldSTATUS: 0x%016llx\n", i, val);
	}
	write_sysreg_s(errselr, SYS_ERRSELR_EL1);
	isb();
}
#endif

static inline void dump_platform_devices(void) {
#ifdef CONFIG_ARM64
	arm64_dump_err_status();
#endif
#ifdef CONFIG_EDAC
	edac_dump_device_stats();
#endif
#ifdef CONFIG_ARM_GIC_V3
	gic_dump_irq_status();
#endif
}

#endif /* _LINUX_SCHED_DEBUG_H */
