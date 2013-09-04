/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef IMC_IDI_BT_IOCTL_H
#define IMC_IDI_BT_IOCTL_H
#include <linux/ioctl.h>

#define IMC_IDI_MAGIC 'i'

/* Changes Device states */
#define IMC_IDI_BT_SET_POWER_STATE _IOWR(IMC_IDI_MAGIC, 1, unsigned long)
/* Changes BT WAKEUP high/low */
#define IMC_IDI_BT_SET_BT_WUP _IOW(IMC_IDI_MAGIC, 2, unsigned long)
/* Changes HOST_WAKEUP high/low: Not used */
#define IMC_IDI_BT_GET_HOST_WUP _IOR(IMC_IDI_MAGIC, 3, unsigned long)
/* Sets RTS */
#define IMC_IDI_BT_SET_RTS _IOW(IMC_IDI_MAGIC, 4, unsigned long)
/* Gets RTS: Not used */
#define IMC_IDI_BT_GET_RTS _IOR(IMC_IDI_MAGIC, 5, unsigned long)
/* Gets CTS status */
#define IMC_IDI_BT_GET_CTS _IOR(IMC_IDI_MAGIC, 6, unsigned long)
#define IMC_IDI_BT_SET_TEMP_MEAS _IOW(IMC_IDI_MAGIC, 7, unsigned long)
#define IMC_IDI_BT_GET_TEMP_MEAS _IOR(IMC_IDI_MAGIC, 8, unsigned long)
#define IMC_IDI_BT_GET_TEMP_MEAS _IOR(IMC_IDI_MAGIC, 8, unsigned long)
#define IMC_IDI_BT_DISABLE_SIGNALING _IO(IMC_IDI_MAGIC, 9)
#endif
