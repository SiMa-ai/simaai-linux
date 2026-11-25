# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Synopsys DWC DPHY drivers
#
dw-dphy-objs := dw-dphy-plat.o dw-dphy-rx.o
ifeq ($(CONFIG_DWC_MIPI_TC_DPHY_GEN3),y)
	dw-dphy-objs += dw-dphy-sysfs.o
endif
obj-$(CONFIG_DWC_MIPI_DPHY_GEN3) += dw-dphy.o
