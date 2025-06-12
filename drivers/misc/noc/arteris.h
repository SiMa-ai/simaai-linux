#ifndef ARTERIS
#define ARTERIS

#include <linux/types.h>

#define SIMAAI_NOC_DEV_NAME "arteris_flex_noc"
#define MAX_TRANSACTION_PROBES 11
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

#endif //ARTERIS
