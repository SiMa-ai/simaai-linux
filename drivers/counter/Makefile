# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for Counter devices
#

obj-$(CONFIG_COUNTER) += counter.o
counter-y := counter-core.o counter-sysfs.o counter-chrdev.o

obj-$(CONFIG_104_QUAD_8)	+= 104-quad-8.o
obj-$(CONFIG_INTERRUPT_CNT)		+= interrupt-cnt.o
obj-$(CONFIG_STM32_TIMER_CNT)	+= stm32-timer-cnt.o
obj-$(CONFIG_STM32_LPTIMER_CNT)	+= stm32-lptimer-cnt.o
obj-$(CONFIG_TI_EQEP)		+= ti-eqep.o
obj-$(CONFIG_FTM_QUADDEC)	+= ftm-quaddec.o
obj-$(CONFIG_MICROCHIP_TCB_CAPTURE)	+= microchip-tcb-capture.o
obj-$(CONFIG_INTEL_QEP)		+= intel-qep.o
obj-$(CONFIG_TI_ECAP_CAPTURE)	+= ti-ecap-capture.o
obj-$(CONFIG_DW_TIMER_CNT)	+= dw-apb-timer-cnt.o
