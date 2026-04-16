#ifndef __ISP_IOCTL_H__
#define __ISP_IOCTL_H__

#include <linux/types.h>

struct isp_calibration_data {
    __u32 ctx_id;
    __u32 size;
    void *data;
};

#define ISP_IOC_MAGIC 'I'
#define ISP_IOC_SET_CALIBRATION _IOW(ISP_IOC_MAGIC, 1, struct isp_calibration_data)

#endif /* __ISP_IOCTL_H__ */