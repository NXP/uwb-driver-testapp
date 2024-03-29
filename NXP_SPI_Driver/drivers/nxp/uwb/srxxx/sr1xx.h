// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
// Copyright 2018-2024 NXP
/*
 * SPI driver for UWB SR1xx
 * Author: Manjunatha Venkatesh <manjunatha.venkatesh@nxp.com>
 */
 
#define SR1XX_MAGIC 0xEA
#define SR1XX_SET_PWR _IOW(SR1XX_MAGIC, 0x01, uint32_t)
#define SR1XX_SET_FWD _IOW(SR1XX_MAGIC, 0x02, uint32_t)
#define SR1XX_ESE_RESET _IOW(SR1XX_MAGIC, 0x03, uint32_t)
#define SR1XX_GET_THROUGHPUT _IOW(SR1XX_MAGIC, 0x04, uint32_t)

/* Power enable/disable and read abort ioctl arguments */
enum {
  PWR_DISABLE = 0,
  PWR_ENABLE,
  ABORT_READ_PENDING
};

struct rx_buffer_node {
  struct list_head list;     //linux kernel list implementation
  unsigned char *rx_buffer;
  unsigned int rx_count;
};
