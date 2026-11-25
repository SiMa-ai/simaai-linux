# SPDX-License-Identifier: GPL-2.0
#
# SiMa.ai MLA (Machine Learning Accelerator) driver.
#
# In-tree kbuild makefile: kbuild descends into this file with KERNELRELEASE
# set and obj-$(CONFIG_SIMAAI_MLA) selects the driver.
#
# To build out of tree straight from the command line:
#
#   make -C <kernel-tree> M=$(pwd) CONFIG_SIMAAI_MLA=m modules
#
# e.g. the aarch64 board build:
#
#   LOCALVERSION= make -C /workspace/simaai-linux M=$(pwd) \
#       ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
#       CONFIG_SIMAAI_MLA=m modules
#
# CONFIG_SIMAAI_MLA=m has to be passed explicitly. This driver's Kconfig is not
# sourced by the build kernel's tree, so the symbol is absent from its .config,
# obj-$(CONFIG_SIMAAI_MLA) expands to a bare `obj-', and kbuild then exits 0
# having built nothing, leaving any stale mla.ko in place to be mistaken for a
# fresh one. Point -C at the shipped kernel tree rather than this repo, or
# vermagic gains a '+' and insmod rejects the module as "Invalid module
# format".

ccflags-y += -I$(src)
# uapi header lives outside the driver source dir (repo layout mirrors
# the kernel's drivers/ / include/uapi/ split). OOT builds land here.
ccflags-y += -I$(src)/../../../include/uapi

obj-$(CONFIG_SIMAAI_MLA) += mla.o
mla-y := mla_drv.o mla_ops.o mla_ops_buf.o \
         mla_dmabuf.o mla_pool.o \
         mla_job.o mla_hw.o mla_sched.o \
         mla_plist.o mla_debugfs.o \
         simaai_mlamem.o
