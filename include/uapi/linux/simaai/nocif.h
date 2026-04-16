
#ifndef NOCIF_H
#define NOCIF_H

#define NOC_DRIVER_MAGIC 'N'
#define NOC_IOCTL_CONFIGURE_AND_RUN _IO(NOC_DRIVER_MAGIC, 1)
#define NOC_IOCTL_READ _IO(NOC_DRIVER_MAGIC, 2)
#define NOC_IOCTL_CONFIGURE_PKT_AND_RUN _IO(NOC_DRIVER_MAGIC, 3)
#define NOC_IOCTL_BW_READ _IO(NOC_DRIVER_MAGIC, 4)

#define MAX_SUPPORTED_OBSERVED_SEL_CNT 2
#define MAX_PKT_FLT_CNT 4

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

struct filterAddrBase {
  uint32_t addrBase_low;
  uint8_t addrBase_high;
  uint32_t windowSize;
};

struct observedDesc {
  char label[64];
  uint32_t threshold[4];
  enum txnFilterMode mode;
  uint8_t addrWindowSize;
  uint32_t histogramBin[5];
  transactionFilterId_t fltId;
  enum txnFilterOpCode opcode;
  struct filterAddrBase addrBase;
};


enum pktCounterSource {
	pktCounterSourceOff = 0,
	pktCounterSourceCycle,
	pktCounterSourceIdle,
	pktCounterSourceXfer,
	pktCounterSourceBusy,
	pktCounterSourceWait,
	pktCounterSourcePkt,
	pktCounterSourceLut,
	pktCounterSourceByte,
	pktCounterSourcePress0,
	pktCounterSourcePress1,
	pktCounterSourcePress2,
	pktCounterSourceFilt0,
	pktCounterSourceFilt1,
	pktCounterSourceFilt2,
	pktCounterSourceFilt3,
	pktCounterSourceChain,
	pktCounterSourceLutByteEn,
	pktCounterSourceLutByte,
	pktCounterSourceFiltByteEn,
	pktCounterSourceFiltByte,
	pktCounterSourcePress3,
	pktCounterSourcePress4,
	pktCounterSourcePress5,
	pktCounterSourcePress6,
	pktCounterSourceReserved
};

enum alarmMode {
	pktCounterAlarmModeOff = 0,
	pktCounterAlarmModeMin,
	pktCounterAlarmModeMax,
	pktCounterAlarmModeMinMax
};

enum pktProbeId {
	prb_bsu_nvs_pkt = 0,
	prb_prc_sio12_pkt,
	prb_prc_sio7_pkt
};

struct routeIdDesc {
  uint32_t routeIdBase;
  uint32_t routeIdMask;
};

struct securityDesc {
  uint32_t securityBase;
  uint32_t securityMask;
};

struct pktFilterDesc {
  struct routeIdDesc fltRouteIdDesc;
  struct filterAddrBase fltAddrDesc;
  struct securityDesc fltSecurityDesc;
  uint16_t fltOpCode;
  uint8_t fltStatus:2;
  uint8_t fltLength : 4;
  uint8_t fltUrgency : 3;
};

struct counterDesc {
  uint8_t counterPortSel : 3;
  enum pktCounterSource pktCounter;
  enum alarmMode pktCounterAlarmMode;
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

struct pktProbeDesc {
  enum pktProbeId id;
  uint8_t tracePort;
  uint8_t nFilterCount;
  char label[64];
  uint64_t bytesCount;
  struct pktFilterDesc fltDesc[4];
  struct counterDesc pktCounterDesc;
};

#endif //NOCIF_H
