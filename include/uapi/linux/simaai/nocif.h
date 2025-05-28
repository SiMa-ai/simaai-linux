
#ifndef NOCIF_H
#define NOCIF_H

#define NOC_DRIVER_MAGIC 'N'
#define NOC_IOCTL_CONFIGURE_AND_RUN _IO(NOC_DRIVER_MAGIC, 1)
#define NOC_IOCTL_READ _IO(NOC_DRIVER_MAGIC, 2)

#define MAX_SUPPORTED_OBSERVED_SEL_CNT 2

typedef uint8_t transactionFilterId_t;
enum transactionProbesId {
	prb_apu_cfg_dat_trn = 0,
	prb_bsu_nvs_trn,
	prb_cvu_trn,
	prb_eth_trn,
	prb_isp_trn,
	prb_mla_dma_trn,
	prb_pnoc_trn,
	prb_sdma_trn,
	prb_vdec_trn,
	prb_venc_trn,
	prb_vis_trn,
	prb_max
};

enum txnProfilerMode { txnProfilerModeDelay = 0, txnProfilerModePending = 1 };
enum txnFilterMode { txnFilterModeHandshake = 0, txnFilterModeLatency = 1 };
enum txnFilterOpCode {
	txnFilterOpCodeDisableAll = 0,
	txnFilterOpCodeReadOnlyEnable = 1,
	txnFilterOpCodeWriteOnlyEnable = 2,
	txnFilterOpCodeReadAndWriteEnable = 3,
};

struct txnFilterAddrBase {
	uint32_t addrBase_low;
	uint8_t addrBase_high;
};

struct observedDesc {
	char label[64];
	uint32_t threshold[4];
	enum txnFilterMode mode;
	uint8_t addrWindowSize;
	uint32_t histogramBin[5];
	transactionFilterId_t fltId;
	enum txnFilterOpCode opcode;
	struct txnFilterAddrBase addrBase;
};

struct txnProbeDesc {
	char label[64];
	uint32_t ovfStatus;
	uint8_t nTenureLines;
	uint8_t nObservedSel;
	enum transactionProbesId id;
	enum txnProfilerMode profilerMode;
	struct observedDesc observedSel[2];
};
#endif //NOCIF_H
