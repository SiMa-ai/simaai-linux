# SPDX-License-Identifier: GPL-2.0
CFLAGS += -static -O3 -Wl,-no-as-needed -Wall $(KHDR_INCLUDES)

TEST_GEN_PROGS = dmabuf-heap carveout-heap mla-heap

include ../lib.mk

$(OUTPUT)/mla-heap: LDLIBS += -lpthread
