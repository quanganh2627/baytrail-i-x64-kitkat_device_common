/******************************************************************************
 *
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
 *
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Filename:      bt_vendor.c
 *
 *  Description:   Broadcom vendor specific library implementation
 *
 ******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <utils/Log.h>
#include "bt_vendor.h"
#include "upio.h"
#include "userial_vendor.h"

#define BTVND_DBG TRUE

#ifndef BTVND_DBG
#define BTVND_DBG FALSE
#endif

#if (BTVND_DBG == TRUE)
#define BTVNDDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTVNDDBG(param, ...) {}
#endif

/******************************************************************************
**  Externs
******************************************************************************/
#if defined INTEL_WP2_USB
void hw_config_start(void);
#endif
uint8_t register_int_evt_callback();
uint8_t hw_lpm_enable(uint8_t turn_on);
uint32_t hw_lpm_get_idle_timeout(void);
void hw_lpm_set_wake_state(uint8_t wake_assert);
void vnd_load_conf(const char *p_path);
#if (SCO_CFG_INCLUDED == TRUE)
void hw_sco_config(void);
#endif
#if (HW_END_WITH_HCI_RESET == TRUE)
void hw_epilog_process(void);
#endif

/******************************************************************************
**  Variables
******************************************************************************/

bt_vendor_callbacks_t *bt_vendor_cbacks = NULL;
uint8_t vnd_local_bd_addr[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/******************************************************************************
**  Local type definitions
******************************************************************************/

/******************************************************************************
**  Static Variables
******************************************************************************/

static const tUSERIAL_CFG userial_init_cfg =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200
};

/******************************************************************************
**  Functions
******************************************************************************/

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/

static int init(const bt_vendor_callbacks_t* p_cb, unsigned char *local_bdaddr)
{
    ALOGI("init");

    if (p_cb == NULL)
    {
        ALOGE("init failed with no user callbacks!");
        return -1;
    }

    userial_vendor_init();
    upio_init();

    vnd_load_conf(VENDOR_LIB_CONF_FILE);

    /* store reference to user callbacks */
    bt_vendor_cbacks = (bt_vendor_callbacks_t *) p_cb;

    /* This is handed over from the stack */
    memcpy(vnd_local_bd_addr, local_bdaddr, 6);

    return 0;
}

#define CASE_RETURN_STR(const) case const: return #const;

static const char* dump_vendor_op(bt_vendor_opcode_t opcode)
{
    switch(opcode)
    {
        CASE_RETURN_STR(BT_VND_OP_POWER_CTRL);
        CASE_RETURN_STR(BT_VND_OP_FW_CFG);
        CASE_RETURN_STR(BT_VND_OP_SCO_CFG);
        CASE_RETURN_STR(BT_VND_OP_USERIAL_OPEN);
        CASE_RETURN_STR(BT_VND_OP_LPM_SET_MODE);
        CASE_RETURN_STR(BT_VND_OP_USERIAL_CLOSE);
        CASE_RETURN_STR(BT_VND_OP_GET_LPM_IDLE_TIMEOUT);
        CASE_RETURN_STR(BT_VND_OP_LPM_WAKE_SET_STATE);
        CASE_RETURN_STR(BT_VND_OP_LPM_SET_DEVICE_STATE);
        CASE_RETURN_STR(BT_VND_OP_LPM_SET_BT_WAKE_STATE);
        CASE_RETURN_STR(BT_VND_OP_LPM_GET_CTS_STATE);
        CASE_RETURN_STR(BT_VND_OP_LPM_SET_RTS_STATE);

        default:
            return "unknown opcode";
    }

}

/** Requested operations */
static int op(bt_vendor_opcode_t opcode, void *param)
{
    int retval = 0;

    BTVNDDBG("op for %d named: %s", opcode,dump_vendor_op(opcode));

    switch(opcode)
    {
        case BT_VND_OP_POWER_CTRL:
            {
                int *state = (int *) param;
                if (*state == BT_VND_PWR_OFF)
#if (INTEL_WP2_UART == TRUE || INTEL_WP2_USB == TRUE)
                    upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
#else
                    ;
#endif
                else if (*state == BT_VND_PWR_ON)
                {
#if (INTEL_WP2_UART == TRUE || INTEL_WP2_USB == TRUE)
                    upio_set_bluetooth_power(UPIO_BT_POWER_ON);
#else
                    ;
#endif
                }
            }
            break;

        case BT_VND_OP_FW_CFG:
            {
                BTVNDDBG("BT_VND_OP_FW_CFG");
#if defined INTEL_AG6XX_UART
                BTVNDDBG("DO NOTHING.");
#elif defined INTEL_WP2_USB
                hw_config_start();
#endif
            }
            break;

        case BT_VND_OP_SCO_CFG:
            {
#if (SCO_CFG_INCLUDED == TRUE)
                hw_sco_config();
                bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
#else
                retval = -1;
#endif
            }
            break;

        case BT_VND_OP_USERIAL_OPEN:
            {
                int (*fd_array)[] = (int (*)[]) param;
                int fd, idx;
                fd = userial_vendor_open((tUSERIAL_CFG *) &userial_init_cfg);
                if (fd != -1)
                {
                    for (idx=0; idx < CH_MAX; idx++)
                        (*fd_array)[idx] = fd;

                    retval = 1;
                }
                /* retval contains numbers of open fd of HCI channels */
            }
            break;

        case BT_VND_OP_USERIAL_CLOSE:
            {
                userial_vendor_close();
            }
            break;

        case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
            {
                uint32_t *timeout_ms = (uint32_t *) param;
                *timeout_ms = hw_lpm_get_idle_timeout();
            }
            break;

        case BT_VND_OP_LPM_SET_MODE:
            {
                uint8_t mode = *(uint8_t *) param;
                BTVNDDBG("%s mode:%d", __func__, mode);
                if( mode == BT_VND_LPM_ENABLE)
                {
                    /*
                     * Before power on the chip. Enable the callback
                     * to receive the first default bd data event
                     */
                    register_int_evt_callback();
                    int netlinkfd = upio_create_netlink_socket();
                    if(netlinkfd > 0)
                    {
                        upio_netlink_send_msg();
                        int status = upio_netlink_listen_thread();
                        if(bt_vendor_cbacks && (status == 0))
                            bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS);
                    }else
                    {
                        if(bt_vendor_cbacks)
                            bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_FAIL);
                    }

                }
                else
                {
                    upio_close_netlink_socket();
                }
            }
            break;

        case BT_VND_OP_LPM_WAKE_SET_STATE:
            {
                uint8_t *state = (uint8_t *) param;
                uint8_t wake_assert = (*state == BT_VND_LPM_WAKE_ASSERT) ? \
                                        TRUE : FALSE;

                //hw_lpm_set_wake_state(wake_assert);
            }
            break;

        case BT_VND_OP_LPM_SET_DEVICE_STATE:
            {
                uint8_t state = *(uint8_t*) param;
                upio_set_d_state(state);
            }
            break;

        case BT_VND_OP_LPM_SET_BT_WAKE_STATE:
            {
                uint8_t state = *(uint8_t*) param;
                upio_set_bt_wake_state(state);
            }
            break;

        case BT_VND_OP_LPM_GET_CTS_STATE:
            return upio_get_cts_state();

        case BT_VND_OP_LPM_SET_RTS_STATE:
            {
                uint8_t state = *(uint8_t*) param;
                upio_set_rts_state(state);
            }
            break;

        case BT_VND_OP_EPILOG:
            {
#if (HW_END_WITH_HCI_RESET == FALSE)
                if (bt_vendor_cbacks)
                    bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
#else
                hw_epilog_process();
#endif
            }
            break;
    }

    return retval;
}

/** Closes the interface */
static void cleanup( void )
{
    BTVNDDBG("cleanup");

    upio_cleanup();

    bt_vendor_cbacks = NULL;
}

// Entry point of DLib
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(bt_vendor_interface_t),
    init,
    op,
    cleanup
};
