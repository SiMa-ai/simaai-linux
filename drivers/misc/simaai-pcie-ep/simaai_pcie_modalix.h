// SPDX-License-Identifier: GPL-2.0
/**
 * Definitions for Modalix HW offset
 *
 * Copyright (C) 2025 SiMa.ai
 * Author:
 */

#ifndef _SIMA_PCIE_MODALIX_H_
#define _SIMA_PCIE_MODALIX_H_

/*
 * Application specific registers on Michelangelo start at offsets
 * x8,x4 - 0xC000
 * x2    - 0xA000
 */
#define MOD_ASR_OFFSET	0xC000

#define MOD_OFF_MLSOC_STATUS	0x04
#define MOD_OFF_FW_STATUS	0x08

#define MOD_OFF_MCMD		0x80
#define MOD_OFF_MCMD_DATA	0x84
#define MOD_OFF_MCMD_DB		0xFC

#define MOD_OFF_MCMD_RESP      0x100
#define MOD_OFF_MCMD_RESP_DATA 0x104

#define MOD_OFF_MCMD_ADDR_LOW	0x30
#define MOD_OFF_MCMD_ADDR_HIGH	0x34

#define MOD_DOORBELL_INT	(BIT(16))
#define MOD_DOORBELL_INT_DIS	(BIT(17))
#define MOD_DOORBELL_VAL(x)	(u16)lower_16_bits(x)

/*
 * Michelangelo ASR has writeable register with doorbell enabled named as
 * wq_tail, cq_head and rq_head. Readable registers as wq_head, cq_tail and
 * rq_tail.
 *
 * This is opposite of what we use in code. We update head for WQ to send
 * request and update tails for CQ and RQ once consumed. Keeping the naming
 * uniform. Below would differ from data sheet
 */
#define MOD_OFF_MWQ_HEAD_DB	0x150
#define MOD_OFF_MWQ_TAIL	0x154

#define MOD_OFF_MCQ_TAIL_DB	0x194
#define MOD_OFF_MCQ_HEAD	0x190

#define MOD_OFF_MRQ_TAIL_DB	0x1D4
#define MOD_OFF_MRQ_HEAD	0x1D0

#define MOD_OFF_WQ1_HEAD_DB	0x210
#define MOD_OFF_WQ1_TAIL	0x214

#define MOD_OFF_CQ1_TAIL_DB	0x254
#define MOD_OFF_CQ1_HEAD	0x250

#define MOD_OFF_RQ1_TAIL_DB	0x294
#define MOD_OFF_RQ1_HEAD	0x290

#define MOD_OFF_WQ_BASE		0x200
#define MOD_OFF_WQ_HEAD_DB(q)	((MOD_OFF_WQ_BASE + ((q * 0x100) + 0x10)))
#define MOD_OFF_WQ_TAIL(q)	((MOD_OFF_WQ_BASE + ((q * 0x100) + 0x14)))

#define MOD_OFF_CQ_HEAD(q)	((MOD_OFF_WQ_BASE + ((q * 0x100) + 0x50)))
#define MOD_OFF_CQ_TAIL_DB(q)	((MOD_OFF_WQ_BASE + ((q * 0x100) + 0x54)))

#define MOD_OFF_RQ_HEAD(q)	((MOD_OFF_WQ_BASE + ((q * 0x100) + 0x90)))
#define MOD_OFF_RQ_TAIL_DB(q)	((MOD_OFF_WQ_BASE + ((q * 0x100) + 0x94)))

/* Host sets TX head and RX tail. TX tail and RX head are RO on host */
#define MOD_OFF_NET_RXTAIL 	0x48
#define MOD_OFF_NET_TXHEAD	0x50
#define MOD_OFF_NET_TXTAIL	0x28
#define MOD_OFF_NET_RXHEAD	0x2c

#define MOD_MCMD_DB		0
#define MOD_MWQ_DB		1
#define MOD_MCQ_DB		2
#define MOD_MRQ_DB		3
#define MOD_DQ_DB_START		4
#define MOD_DQ_DB_END		27
#define MOD_SPARE_DB_START	28
#define MOD_SPARE_DB_END	35
#define MOD_DWQ_DB_REM		0
#define MOD_DCQ_DB_REM		1
#define MOD_DRQ_DB_REM		2
#define MOD_NET_TXHEAD_DB	MOD_SPARE_DB_START

#endif
