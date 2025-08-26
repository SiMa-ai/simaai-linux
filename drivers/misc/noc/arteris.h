#ifndef ARTERIS
#define ARTERIS

#include <linux/types.h>

#define SIMAAI_NOC_DEV_NAME "arteris_flex_noc"
#define MAX_TRANSACTION_PROBES 11
#define MAX_PACKET_PROBES 3
#define MAX_TRANSACTION_STAT_FILTER_PER_PROBE 6
#define MAX_THRESHOLD_COUNT_PER_CH 4

/* Transaction Probe reg offset */
#define TXN_PROBE_CNT_SRC_OFFSET(x) (0x204 + (x)*0x10)
#define TXN_PROBE_CNT_SRC_EXT (1 << 5)
#define TXN_PROBE_CNT_VAL_OFFSET(x) (0x20C + (x)*0x10)
#define TXN_PROBE_MAIN_CTL_OFFSET (0x08)
#define TXN_PROBE_MAIN_CTL_STATEN (1 << 3)
#define TXN_PROBE_CFG_CTL_OFFSET (0x0C)
#define TXN_PROBE_CFG_CTL_GLOBAL_EN (1 << 0)

/* Transaction Stat profiler reg offsets */
#define TXN_PROFILER_EN_OFFSET (0x08)
#define TXN_PROFILER_EN (0x01)
#define TXN_PROFILER_MODE_OFFSET (0x0C)
#define TXN_PROFILER_MODE_MASK (0x03)
#define TXN_PROFILER_TENURE_LINES_OFFSET (0x20)
#define TXN_PROFILER_TENURE_LINES_MASK (0x0F)
#define TXN_PROFILER_OBS_SEL_OFFSET(x) (0x10 + ((x)*0x4))
#define TXN_PROFILER_OBS_SEL_MASK(x) ((x) > 3 ? 0x7 : 0x3)
#define TXN_PROFILER_THRESHOLD_OFFSET(init) (0x2C + ((init & 0x01) * 0x10))
#define TXN_PROFILER_THRESHOLD_MASK (0x3FF)
#define TXN_PROFILER_OVF_STATUS_OFFSET (0x6c)
#define TXN_PROFILER_OVF_RESET_OFFSET (0x70)
#define TXN_PROFILER_OVF_RESET_STATUS (0x3)

/* Transaction Stat filter reg offsets*/
#define TXN_FILTER_MODE_OFFSET (0x08)
#define TXN_FILTER_MODE_MASK (0x01)
#define TXN_FILTER_OPCODE_OFFSET (0x20)
#define TXN_FILTER_OPCODE_MASK (0x03)
#define TXN_FILTER_ADDRBASE_LOW_OFFSET (0x0C)
#define TXN_FILTER_ADDRBASE_HIGH_OFFSET (0x10)
#define TXN_FILTER_ADDRWINDOW_SIZE_OFFSET (0x14)
#define TXN_FILTER_ADDRWINDOW_SIZE_MASK (0x3F)


/* Packet probe offsets */
#define PKT_PROBE_MAIN_CTL_OFFSET (0x8)
#define PKT_PROBE_MAIN_CTL_MASK (0xFF)
#define PKT_PROBE_CFG_CTL_OFFSET (0xC)
#define PKT_PROBE_TRACE_PORT_SEL_OFFSET (0x10)
#define PKT_PROBE_FILTER_LUT_OFFSET (0x14)
#define PKT_PROBE_TRACE_ALARM_EN_OFFSET (0x18)
#define PKT_PROBE_TRACE_ALARM_STATUS_OFFSET (0x1C)
#define PKT_PROBE_TRACE_ALARM_CLR_OFFSET (0x20)
#define PKT_PROBE_STAT_PERIOD_OFFSET (0x24)
#define PKT_PROBE_STAT_GO_OFFSET (0x28)
#define PKT_PROBE_STAT_ALARM_MIN_OFFSET (0x2C)
#define PKT_PROBE_STAT_ALARM_MIN_HIGH_OFFSET (0x30)
#define PKT_PROBE_STAT_ALARM_MAX_OFFSET (0x34)
#define PKT_PROBE_STAT_ALARM_MAX_HIGH_OFFSET (0x38)
#define PKT_PROBE_STAT_ALARM_STATUS_OFFSET (0x3C)
#define PKT_PROBE_STAT_ALARM_CLEAR_OFFSET (0x40)
#define PKT_PROBE_STAT_ALARM_EN_OFFSET (0x44)

//Pkt filters
#define MAX_PKT_PROBE_FILT (4)
#define PKT_PROBE_FILT_X_ROUTE_ID_BASE_OFFSET(x) ((x * 0x60) + 0x80)
#define PKT_PROBE_FILT_X_ROUTE_ID_MASK_OFFSET(x) ((x * 0x60) + 0x84)
#define PKT_PROBE_FILT_X_ADDR_BASE_LOW_OFFSET(x) ((x * 0x60) + 0x88)
#define PKT_PROBE_FILT_X_ADDR_BASE_HIGH_OFFSET(x) ((x * 0x60) + 0x8C)
#define PKT_PROBE_FILT_X_WINDOW_SIZE_OFFSET(x) ((x * 0x60) + 0x90)
#define PKT_PROBE_FILT_X_SECURITY_BASE_OFFSET(x) ((x * 0x60) + 0x94)
#define PKT_PROBE_FILT_X_SECURITY_MASK_OFFSET(x) ((x * 0x60) + 0x98)
#define PKT_PROBE_FILT_X_OPCODE_OFFSET(x) ((x * 0x60) + 0x9C)
#define PKT_PROBE_FILT_X_STATUS_OFFSET(x) ((x * 0x60) + 0xA0)
#define PKT_PROBE_FILT_X_LENGTH_OFFSET(x) ((x * 0x60) + 0xA4)
#define PKT_PROBE_FILT_X_URGENCY_OFFSET(x) ((x * 0x60) + 0xA8)

//Pkt counters
#define MAX_PKT_PROBE_COUNTERS (8)
#define PKT_PROBE_COUNTER_X_PORTSEL(x) ((x * 0x10) + 0x200)
#define PKT_PROBE_COUNTER_X_SRC(x) ((x * 0x10) + 0x204)
#define PKT_PROBE_COUNTER_X_ALARM_MODE(x) ((x * 0x10) + 0x208)
#define PKT_PROBE_COUNTER_X_VAL(x) ((x * 0x10) + 0x20C)

//PKT probe Fields
#define PROBE_MAIN_CTL_STATEN (1 << 3)
#define PROBE_MAIN_CTL_PAYLOADEN (1 << 2)
#define PROBE_CFG_CTL_GLOBAL_EN (1 << 0)


struct transactionFilter {
  void __iomem  *reg;
  const char *label;
};

struct transactionProfiler {
  void __iomem *reg;
  u32 nObsSel; //1 or 2
};

struct transactionProbe {
  void __iomem *reg;
  const char *label;
  u32 nFilters;
  u32 nCounters;
  struct transactionProfiler profiler;
  struct transactionFilter fltDesc[MAX_TRANSACTION_STAT_FILTER_PER_PROBE];
};

struct packetProbe {
  void __iomem *reg;
  u32 probeId;
  const char *label;
  u32 nCounters;
};

#endif //ARTERIS
