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
 *  Filename:      hardware.c
 *
 *  Description:   Contains controller-specific functions, like
 *                      firmware patch download
 *                      low power mode operations
 *
 ******************************************************************************/

#define LOG_TAG "bt_hwcfg"

#include <ctype.h>
#include <cutils/properties.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <utils/Log.h>

#include "bt_hci_bdroid.h"
#include "bt_vendor.h"
#include "userial.h"
#include "userial_vendor.h"
#include "upio.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#define BTHW_DBG TRUE

#ifndef BTHW_DBG
#define BTHW_DBG FALSE
#endif

#if (BTHW_DBG == TRUE)
#define BTHWDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTHWDBG(param, ...) {}
#endif

#if defined INTEL_AG6XX_UART
#define FW_PATCHFILE_EXTENSION      ".pbn"
#define BDDATA_FILE                 "/system/etc/bluetooth/bddata"
#else
#define FW_PATCHFILE_EXTENSION      ".seq"
#endif
#define FW_PATCHFILE_EXTENSION_LEN  4
#define FW_PATCHFILE_PATH_MAXLEN    128 /* FW patch path Max length */

#define HCI_CMD_MAX_LEN             258

/* HCI command opcode */
#if defined INTEL_WP2_USB
#define HCI_RESET                               0x0C03
#endif
#define HCI_INTEL_READ_SW_VERSION               0xFC05
#if defined INTEL_WP2_UART
#define HCI_INTEL_SET_UART_BAUD                 0xFC06
#endif
#define HCI_INTEL_MANUFACTURE_MODE              0xFC11
#ifdef INTEL_AG6XX_UART
#define HCI_INTEL_INF_BDDATA                    0xFC2F
#endif
#define HCI_INTEL_INF_MEM_WRITE                 0xFC8E
/* HCI command param sizes */
#define HCI_INTEL_MEM_WRITE_MODE_BYTE           0
#define HCI_CMD_PREAMBLE_SIZE                   3
#define HCI_INTEL_MANUFACTURE_MODE_PARAM_SIZE   2
#define HCI_INTEL_SET_UART_BAUD_PARAM_SIZE      1
#define HCI_INTEL_READ_SW_VERSION_PARAM_SIZE    0
#define HCI_INTEL_INF_BDDATA_PARAM_SIZE         80
/* HCI read sw version number byte location */
#if defined INTEL_AG6XX_UART
#define HCI_EVT_READ_HW_VARIANT                 7
#define HCI_EVT_READ_HW_REVISION                8
#else

#endif
#define HCI_EVT_CMD_CMPL_STATUS_RET_BYTE        5
#define HCI_COMMAND_STATUS_EVT_STATUS_BYTE      2
#define HCI_INTEL_EVT_STATUS_RET_BYTE           3
/* HCI event code & event id */
#define HCI_COMMAND_CMPL_EVT_CODE               0X0E
#define HCI_COMMAND_STATUS_EVT_CODE             0x0F
#define HCI_INTEL_DEBUG_EVT_CODE                0xFF
#define HCI_INTEL_STARTUP                       0x00
#define HCI_INTEL_DEFAULT_BD_DATA               0x05
#define HCI_INTEL_WRITE_BD_DATA_CMPL            0x19

#define PATCH_MAX_LENGTH                        244

#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}

/******************************************************************************
**  Local type definitions
******************************************************************************/
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/* Return Types */
typedef enum
{
    FAILURE = -1,
    SUCCESS = 0
} return_value;

/* Hardware Configuration State */
enum {
    HW_CFG_INIT = 0,            /* Initial state. */
    HW_CFG_MANUFACTURE_ON,      /* Intel manufature mode on */
    HW_CFG_MANUFACTURE_OFF,     /* Intel manufature mode off */
#if defined INTEL_AG6XX_UART
    HW_CFG_BDDATA,              /* Open bddata file and download the BD data */
    HW_CFG_BDDATA_STATUS,       /* This handles the command status comes after write BD DATA command */
#endif
#if defined INTEL_WP2_USB
    HW_CFG_MEMWRITE,            /* Patching in WP2 family of chips. For RAM chips this is whole FW image dl. */
#endif
    HW_SET_BAUD_HS,             /* Change the controller to higher baud rate*/
    HW_SET_HOST_BAUD,           /* Change the host baud rate */
    HW_CFG_MANUFACTURE_OFF_CMPL,/* Command complete after manufacture mode off received */
    HW_CFG_SW_READ_VERSION,     /* Read SW version from controller to construct required patch filename */
    HW_CFG_SW_FIND_PATCH,       /* Find the required patch file in file system */
    HW_CFG_DL_FW_PATCH,         /* Donwload the firmware patch file */
    HW_CFG_DL_FW_PATCH1,
    HW_CFG_SUCCESS,             /* Controller init finished */
};

/* h/w config control block */
typedef struct
{
    uint8_t state;                          /* Hardware configuration state */
    FILE*   fw_fd;                          /* FW patch file pointer */
    uint8_t is_patch_enabled;               /* Is patch is enabled? 2: enabled 1:not enabled */
#if defined INTEL_AG6XX_UART
    uint32_t address;                       /* Address in which data has to be written */
    uint32_t nr_of_bytes;                   /* No of bytes to be written field in pbn file */
#endif
} bt_hw_cfg_cb_t;

/* low power mode parameters */
typedef struct
{
    uint8_t sleep_mode;                     /* 0(disable),1(UART),9(H5) */
    uint8_t host_stack_idle_threshold;      /* Unit scale 300ms/25ms */
    uint8_t host_controller_idle_threshold; /* Unit scale 300ms/25ms */
    uint8_t bt_wake_polarity;               /* 0=Active Low, 1= Active High */
    uint8_t host_wake_polarity;             /* 0=Active Low, 1= Active High */
    uint8_t allow_host_sleep_during_sco;
    uint8_t combine_sleep_mode_and_lpm;
    uint8_t enable_uart_txd_tri_state;      /* UART_TXD Tri-State */
    uint8_t sleep_guard_time;               /* sleep guard time in 12.5ms */
    uint8_t wakeup_guard_time;              /* wakeup guard time in 12.5ms */
    uint8_t txd_config;                     /* TXD is high in sleep state */
    uint8_t pulsed_host_wake;               /* pulsed host wake if mode = 1 */
} bt_lpm_param_t;

/******************************************************************************
**  Externs
******************************************************************************/

void hw_config_cback(void *p_evt_buf);

/******************************************************************************
**  Static variables
******************************************************************************/
#if defined INTEL_WP2_USB
static int fw_patchfile_empty = 0;
#endif
static char fw_patchfile_path[256] = FW_PATCHFILE_LOCATION;
static char fw_patchfile_name[128] = { 0 };
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
static int fw_patch_settlement_delay = -1;
#endif

static bt_hw_cfg_cb_t hw_cfg_cb;

static bt_lpm_param_t lpm_param =
{
    LPM_SLEEP_MODE,
    LPM_IDLE_THRESHOLD,
    LPM_HC_IDLE_THRESHOLD,
    LPM_BT_WAKE_POLARITY,
    LPM_HOST_WAKE_POLARITY,
    LPM_ALLOW_HOST_SLEEP_DURING_SCO,
    LPM_COMBINE_SLEEP_MODE_AND_LPM,
    LPM_ENABLE_UART_TXD_TRI_STATE,
    0,  /* not applicable */
    0,  /* not applicable */
    0,  /* not applicable */
    LPM_PULSED_HOST_WAKE
};

/******************************************************************************
**  Global functions
******************************************************************************/
/*******************************************************************************
**
** Function        register_int_evt_callbaks
**
** Description     Registers callback function to get internal async event.
**
** Returns         success/failure to register.
**
*******************************************************************************/
uint8_t register_int_evt_callback()
{
    /* Initialize the state */
    BTHWDBG("%s", __func__);
    hw_cfg_cb.state = HW_CFG_INIT;
    return bt_vendor_cbacks->int_evt_callback_reg_cb(hw_config_cback);
}

/******************************************************************************
**  Controller Initialization Static Functions
******************************************************************************/
/*******************************************************************************
**
** Function        check_event
**
** Description     checks the event for HCI success and returns the result
**
** Returns         0: success, otherwise errorcode.
**
*******************************************************************************/
static uint8_t check_event(uint8_t* p_buf)
{
    uint8_t event_code, sub_event_code;
    uint8_t status;
    if (!p_buf)
        return FAILURE;
    event_code = p_buf[0];
    BTHWDBG("%s event_code:0x%x", __func__, event_code);
    switch(event_code)
    {
        case HCI_INTEL_DEBUG_EVT_CODE:
            sub_event_code = p_buf[2];
            BTHWDBG("%s subevent:0x%x", __func__, sub_event_code);
            switch(sub_event_code)
            {
                case HCI_INTEL_STARTUP:
                    return SUCCESS;
                case HCI_INTEL_WRITE_BD_DATA_CMPL:
                    return p_buf[HCI_INTEL_EVT_STATUS_RET_BYTE];
                case HCI_INTEL_DEFAULT_BD_DATA:
                    /* This is first default bd data event */
                    if (hw_cfg_cb.state == HW_CFG_INIT)
                    {
                        /* Deregister the callback */
                        bt_vendor_cbacks->int_evt_callback_dereg_cb();
                        hw_cfg_cb.state = HW_CFG_MANUFACTURE_ON;
                    }
                    /* 3rd byte (memory status): 02- manufacture data in RAM is not valid */
                    return p_buf[HCI_INTEL_EVT_STATUS_RET_BYTE]==0x02? SUCCESS : FAILURE;
                default:
                    BTHWDBG("%s Unknown vsc event. EI:0x%02X", __func__, sub_event_code);
                    return FAILURE; // Unexpected event. Something Unusual happened.
            }
        case HCI_COMMAND_CMPL_EVT_CODE:
            return p_buf[HCI_EVT_CMD_CMPL_STATUS_RET_BYTE];
        case HCI_COMMAND_STATUS_EVT_CODE:
            return p_buf[HCI_COMMAND_STATUS_EVT_STATUS_BYTE];
        default:
            return SUCCESS;
    }
}

/*******************************************************************************
**
** Function        ms_delay
**
** Description     sleep unconditionally for timeout milliseconds
**
** Returns         None
**
*******************************************************************************/
void ms_delay (uint32_t timeout)
{
    struct timespec delay;
    int err;

    if (timeout == 0)
        return;

    delay.tv_sec = timeout / 1000;
    delay.tv_nsec = 1000 * 1000 * (timeout%1000);

    /* [u]sleep can't be used because it uses SIGALRM */
    do {
        err = nanosleep(&delay, &delay);
    } while (err < 0 && errno ==EINTR);
}

#if defined INTEL_WP2_UART
/*******************************************************************************
**
** Function        line_speed_to_userial_baud
**
** Description     helper function converts line speed number into USERIAL baud
**                 rate symbol
**
** Returns         unit8_t (USERIAL baud symbol)
**
*******************************************************************************/
uint8_t line_speed_to_userial_baud(uint32_t line_speed)
{
    uint8_t baud;

    if (line_speed == 4000000)
        baud = USERIAL_BAUD_4M;
    else if (line_speed == 3000000)
        baud = USERIAL_BAUD_3M;
    else if (line_speed == 2000000)
        baud = USERIAL_BAUD_2M;
    else if (line_speed == 1000000)
        baud = USERIAL_BAUD_1M;
    else if (line_speed == 921600)
        baud = USERIAL_BAUD_921600;
    else if (line_speed == 460800)
        baud = USERIAL_BAUD_460800;
    else if (line_speed == 230400)
        baud = USERIAL_BAUD_230400;
    else if (line_speed == 115200)
        baud = USERIAL_BAUD_115200;
    else if (line_speed == 57600)
        baud = USERIAL_BAUD_57600;
    else if (line_speed == 19200)
        baud = USERIAL_BAUD_19200;
    else if (line_speed == 9600)
        baud = USERIAL_BAUD_9600;
    else if (line_speed == 1200)
        baud = USERIAL_BAUD_1200;
    else if (line_speed == 600)
        baud = USERIAL_BAUD_600;
    else
    {
        ALOGE( "userial vendor: unsupported baud speed %d", line_speed);
        baud = USERIAL_BAUD_115200;
    }

    return baud;
}
#endif

/*******************************************************************************
**
** Function         hw_strncmp
**
** Description      Used to compare two strings in caseless
**
** Returns          0: match, otherwise: not match
**
*******************************************************************************/
static int hw_strncmp (const char *p_str1, const char *p_str2, const int len)
{
    int i;

    if (!p_str1 || !p_str2)
        return (1);

    for (i = 0; i < len; i++)
    {
        if (toupper(p_str1[i]) != toupper(p_str2[i]))
            return (i+1);
    }

    return 0;
}

/*******************************************************************************
**
** Function         hw_config_findpatch
**
** Description      Search for a proper firmware patch file
**                  The selected firmware patch file name with full path
**                  will be stored in the input string parameter, i.e.
**                  p_chip_id_str, when returns.
**
** Returns          TRUE when found the target patch file, otherwise FALSE
**
*******************************************************************************/
static uint8_t hw_config_findpatch(char *p_chip_id_str, char** p_patch_file)
{
    DIR *dirp;
    struct dirent *dp;
    int filenamelen;
    uint8_t retval = FALSE;
    char* patch_filename = NULL;

    BTHWDBG("Target name = [%s]", p_chip_id_str);

    //TODO: Implement read from conf file
    if (strlen(fw_patchfile_name)> 0)
    {
        /* If specific filepath and filename have been given in run-time
         * configuration /etc/bluetooth/bt_vendor.conf file, we will use them
         * to concatenate the filename to open rather than searching a file
         * matching to chipset name in the fw_patchfile_path folder.
         */
        sprintf(p_chip_id_str, "%s", fw_patchfile_path);
        if (fw_patchfile_path[strlen(fw_patchfile_path)- 1] != '/')
        {
            strcat(p_chip_id_str, "/");
        }
        strcat(p_chip_id_str, fw_patchfile_name);

        BTHWDBG("FW patchfile: %s", p_chip_id_str);
        return TRUE;
    }

    if ((dirp = opendir(fw_patchfile_path)) != NULL)
    {
        /* Fetch next filename in patchfile directory */
        while ((dp = readdir(dirp)) != NULL)
        {
            /* Check if filename starts with chip-id name */
            if ((hw_strncmp(dp->d_name, p_chip_id_str, strlen(p_chip_id_str)) \
                ) == 0)
            {
                /* Check if it has .pbn extenstion */
                filenamelen = strlen(dp->d_name);
                if ((filenamelen >= FW_PATCHFILE_EXTENSION_LEN) &&
                    ((hw_strncmp(
                          &dp->d_name[filenamelen-FW_PATCHFILE_EXTENSION_LEN], \
                          FW_PATCHFILE_EXTENSION, \
                          FW_PATCHFILE_EXTENSION_LEN) \
                     ) == 0))
                {
                    BTHWDBG("Found patchfile: %s%s", \
                        fw_patchfile_path, dp->d_name);

                    /* Make sure length does not exceed maximum */
                    if ((filenamelen + strlen(fw_patchfile_path)) > \
                         FW_PATCHFILE_PATH_MAXLEN)
                    {
                        ALOGE("Invalid patchfile name (too long)");
                    }
                    else
                    {
                        BTHWDBG("Found patchfile. Store location and name");
                        //free(p_chip_id_str);
                        //p_chip_id_str = NULL;
                        int path_length = strlen(fw_patchfile_path) + strlen(dp->d_name);
                        if (fw_patchfile_path[ \
                            strlen(fw_patchfile_path)- 1 \
                            ] != '/')
                            path_length++;
                        patch_filename = (char*) malloc (path_length * sizeof (char));

                        memset(patch_filename, 0, sizeof (patch_filename));
                        /* Found patchfile. Store location and name */
                        strcpy(patch_filename, fw_patchfile_path);
                        //BTHWDBG("1 p_chip_id_str:%s",patch_filename);
                        if (fw_patchfile_path[ \
                            strlen(fw_patchfile_path)- 1 \
                            ] != '/')
                        {
                            strcat(patch_filename, "/");
                        }
                        strcat(patch_filename, dp->d_name);
                        //BTHWDBG("2 p_chip_id_str:%s",patch_filename);
                        *p_patch_file = patch_filename;
                        retval = TRUE;
                    }
                    break;
                }
            }
        }

        closedir(dirp);

        if (retval == FALSE)
        {
            ALOGE("Could not open %s", fw_patchfile_path);
        }
    }
    else
    {
        ALOGE("Patch file path doesn't exist");
    }

    return (retval);
}

#ifdef INTEL_AG6XX_UART
unsigned char char_to_hex(char c)
{
    if (c == '0') return 0x00;
    else if (c == '1') return 0x01;
    else if (c == '2') return 0x02;
    else if (c == '3') return 0x03;
    else if (c == '4') return 0x04;
    else if (c == '5') return 0x05;
    else if (c == '6') return 0x06;
    else if (c == '7') return 0x07;
    else if (c == '8') return 0x08;
    else if (c == '9') return 0x09;
    else if (c == 'A' || c == 'a') return 0x0a;
    else if (c == 'B' || c == 'b') return 0x0b;
    else if (c == 'C' || c == 'c') return 0x0c;
    else if (c == 'D' || c == 'd') return 0x0d;
    else if (c == 'E' || c == 'e') return 0x0e;
    else if (c == 'F' || c == 'f') return 0x0f;
    else
        return -1;
}


/*******************************************************************************
**
** Function         open_bddata
**
** Description      Function to open the bddata file stored in /data/misc/bluedroid/bddata
**
** Returns          None
**
*******************************************************************************/
int open_bddata(uint8_t *p)
{
    FILE *fp;
    unsigned int i;
    int ret;
    unsigned char c = '\0';
    unsigned char *line = NULL;
    int read;
    size_t cmd_size;

    BTHWDBG("%s",__func__);
    fp = fopen(BDDATA_FILE, "rb");
    if (fp == NULL)
    {
        ALOGE("cannot open:%s",BDDATA_FILE);
        return FAILURE;
       }
    line = (uint8_t*) malloc (1024 * sizeof (uint8_t));
    if (line == NULL)
    {
        ALOGE("Malloc failure");
        return FAILURE;
    }
       read = fread(line, sizeof(unsigned char), 1024, fp);
       if (read < 0)
       {
           ALOGE("line is not read properly. read:%d errno:%d strerror:%s", read, errno, strerror(errno));
           free(line);
           return FAILURE;
    }
       line[read] = '\0';
    cmd_size = read;
    for(i=0; i<cmd_size; i++)
    {
        if (i%2 == 1)
        {
            c |= char_to_hex(line[i]);
            *p++ = c;
        }
        else
        {
            c = char_to_hex(line[i]) << 4;
        }
    }
    if (line)
        free(line);
    fclose(fp);
    return SUCCESS;
}
#endif

#if defined INTEL_WP2_USB
/*******************************************************************************
**
** Function         form_byte
**
** Description      Convert input to a byte
**
** Returns          formed byte
**
*******************************************************************************/
unsigned char form_byte(char msb, char lsb)
{
    unsigned char byte;
    byte = (char_to_hex(msb)) << 4;
    byte |= char_to_hex(lsb);
    return byte;
}

/*******************************************************************************
**
** Function         form_word
**
** Description      Convert input to a word
**
** Returns          formed word
**
*******************************************************************************/
uint16_t form_word(uint8_t msb, uint8_t lsb)
{
    uint16_t byte;
    byte = msb << 8;
    byte |= lsb;
    return byte;
}
#endif

#if (BTHW_DBG == TRUE)
/*******************************************************************************
**
** Function         hex_print
**
** Description      Print buffer 'a' in hex
**
** Returns          None
**
*******************************************************************************/
static void hex_print(char* msg, unsigned char *a, unsigned int size)
{
    if (a==NULL)
    {
        ALOGE("%s nothing to print.", __func__);
    }
    char *p = (char*) malloc (sizeof (char) * ((size*3) + strlen(msg)+1));
    if (!p)
        return;
    bzero(p, size + strlen(msg)+1+(size));
    int i;
    sprintf (p,"%s%s:", p, msg);
    for (i=0;i<size;i++)
        sprintf (p, "%s %X", p, a[i]);
    p[(size*3) + strlen(msg)] = '\0';
    BTHWDBG ("%s\n",p);
    if (p)
    {
        free(p);
        p = NULL;
    }
}
#endif

/*******************************************************************************
**
** Function         hw_config_cback
**
** Description      Callback function for controller configuration
**
** Returns          None
**
*******************************************************************************/
void hw_config_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = NULL;
    char        *p_name;
    char *p_tmp;
    uint8_t     *p;
    uint8_t status = FAILURE;
    uint16_t    opcode;
    HC_BT_HDR *p_buf = NULL;
    uint8_t     is_proceeding = FALSE;
#if defined INTEL_AG6XX_UART
    int read;
    uint8_t* pData = NULL;
#endif
#if defined INTEL_WP2_USB
    int         pos;
#endif

    pthread_mutex_lock( &mutex );
    BTHWDBG("%s",__func__);

    p_evt_buf = (HC_BT_HDR *) p_mem;
    p = (uint8_t*)(p_evt_buf+1);
    status = check_event(p);

    BTHWDBG("p_evt_buf->event:0x%x p_evt_buf->len:0x%x p_evt_buf->offset:0x%x p_evt_buf->layer_specific:0x%x",\
        p_evt_buf->event, p_evt_buf->len, p_evt_buf->offset, p_evt_buf->layer_specific);
    BTHWDBG("status:%u p:%x",status, *p);
    /* Ask a new buffer big enough to hold any HCI commands sent in here */
    if ((status == SUCCESS) && bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_MAX_LEN);
        bzero(p_buf, BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
    }

    if (p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;

        p = (uint8_t *) (p_buf + 1);

        switch (hw_cfg_cb.state)
        {
#if defined INTEL_WP2_UART
            case HW_SET_BAUD_HS:
                BTHWDBG("HW_SET_BAUD_HS");
                ms_delay(10);
                UINT16_TO_STREAM(p, HCI_INTEL_SET_UART_BAUD);
                *p++ = HCI_INTEL_SET_UART_BAUD_PARAM_SIZE; /* parameter length */
                *p++ = 0x0A; //Baud rate 2M

                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_SET_UART_BAUD_PARAM_SIZE;
                hw_cfg_cb.state = HW_SET_HOST_BAUD;

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_SET_UART_BAUD, \
                                                    p_buf, NULL);

            // fall through intentional
           case HW_SET_HOST_BAUD:
                   BTHWDBG("HW_SET_HOST_BAUD");
                //TODO: change host baud rate.
                //break;
#endif
            case HW_CFG_MANUFACTURE_ON:
                BTHWDBG("HW_CFG_MANUFACTURE_ON");
                UINT16_TO_STREAM(p, HCI_INTEL_MANUFACTURE_MODE);
                *p++ = HCI_INTEL_MANUFACTURE_MODE_PARAM_SIZE; /* parameter length */
                *p++ = 01;
                *p++ = 00;

                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_MANUFACTURE_MODE_PARAM_SIZE;

#if defined INTEL_AG6XX_UART
                hw_cfg_cb.state = HW_CFG_BDDATA;
#elif defined INTEL_WP2_USB
                /*
                 * Previous command was HCI RESET. Skip command complete
                 * event. Respond to default bd data event.
                 */
                uint8_t *buf = (uint8_t*) (p_evt_buf+1);
                if (buf[2] == HCI_INTEL_DEFAULT_BD_DATA)
                    hw_cfg_cb.state = HW_CFG_SW_READ_VERSION;
                else
                {
                    if (bt_vendor_cbacks)
                        bt_vendor_cbacks->dealloc(p_buf);
                    is_proceeding = TRUE;
                    break;
                }
#endif
                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_MANUFACTURE_MODE, \
                                                    HCI_COMMAND_CMPL_EVT_CODE, \
                                                    p_buf, hw_config_cback);

                break;
#if defined INTEL_AG6XX_UART
            case HW_CFG_BDDATA:
                BTHWDBG("HW CFG_INF_BDDATA.");
                UINT16_TO_STREAM(p, HCI_INTEL_INF_BDDATA);
                *p++ = HCI_INTEL_INF_BDDATA_PARAM_SIZE; /* parameter length */
                int ret = open_bddata(p);
                if (ret == 0)
                {
                    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_INF_BDDATA_PARAM_SIZE;
                    hw_cfg_cb.state = HW_CFG_BDDATA_STATUS;

                    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_BDDATA, \
                                                    HCI_INTEL_WRITE_BD_DATA_CMPL, \
                                                    p_buf, hw_config_cback);

                } else
                {
                    //FIXME should turn off manufacture mode and gracefully fail
                    ALOGE("open_bddata FAILED.");
                    bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
                }
                break;
            case HW_CFG_BDDATA_STATUS:
                BTHWDBG("HW_CFG_BDDATA_STATUS");
                if (bt_vendor_cbacks)
                    bt_vendor_cbacks->dealloc(p_buf);
                hw_cfg_cb.state = HW_CFG_SW_READ_VERSION;
                is_proceeding = TRUE;
                break;
#endif
            case HW_CFG_SW_READ_VERSION:
                BTHWDBG("HW_READ_VERSION");
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_READ_SW_VERSION_PARAM_SIZE;
                UINT16_TO_STREAM(p, HCI_INTEL_READ_SW_VERSION);
                *p++ = HCI_INTEL_READ_SW_VERSION_PARAM_SIZE; /* parameter length */
                *p++ = 00;

                hw_cfg_cb.state = HW_CFG_SW_FIND_PATCH;

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_READ_SW_VERSION, \
                                                    HCI_COMMAND_CMPL_EVT_CODE, \
                                                    p_buf, hw_config_cback);
                break;
            case HW_CFG_SW_FIND_PATCH:
                BTHWDBG("HW_CFG_SW_FIND_PATCH");
                //Find the patch. and open the patch file
                char* p_patch_file = NULL;
                uint8_t* temp = (uint8_t*)(p_evt_buf + 1);
                uint8_t fw_patch_name[FW_PATCHFILE_PATH_MAXLEN];
                bzero(fw_patch_name, FW_PATCHFILE_PATH_MAXLEN);
#if defined INTEL_AG6XX_UART
                uint8_t hw_variant = temp[HCI_EVT_READ_HW_VARIANT];
                uint8_t hw_revision = temp[HCI_EVT_READ_HW_REVISION];
                uint16_t dev_id = (hw_variant<<8) | (hw_revision<<0);
                BTHWDBG("hw_varient:0x%x hw_revision:0x%x Device id:0x%x", hw_variant, hw_revision,dev_id);
                /* AG620 file name should be a00.pbn unless specified in conf file */
                sprintf(fw_patch_name, "%x\0", dev_id);
#endif
#if defined INTEL_WP2_USB
                sprintf(fw_patch_name, "%02x%02x%02x%02x%02x%02x%02x%02x%02x\0", temp[6], temp[7],
                temp[8], temp[9], temp[10], temp[11],
                temp[12], temp[13], temp[14]);
#endif
                //BTHWDBG("FW patch file name constructed:%s",fw_patch_name);
                if (hw_config_findpatch(fw_patch_name, &p_patch_file) == TRUE)
                {
                    //Open the patch file
                    BTHWDBG("Open patch file:%s",p_patch_file);
                    if ((hw_cfg_cb.fw_fd = fopen(p_patch_file, "rb")) == NULL) {
                        ALOGE("Cannot open %s", p_patch_file);
                        if (p_patch_file)
                        {
                            free (p_patch_file);
                            p_patch_file = NULL;
                        }
                        hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                        goto HW_CFG_MANUFACTURE_OFF;
                    }
                    else
                    {
                        BTHWDBG("Donwload FW will begin");
                        if (p_patch_file)
                        {
                            free (p_patch_file);
                            p_patch_file = NULL;
                        }
#if defined INTEL_AG6XX_UART
                        hw_cfg_cb.state = HW_CFG_DL_FW_PATCH;
#endif
#if defined INTEL_WP2_USB
                        hw_cfg_cb.state = HW_CFG_MEMWRITE;
#endif
                    }
                }
                else
                {
                    //Patch file not found. Nothing to do
                    BTHWDBG("Patch file not found");
                    hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                    goto HW_CFG_MANUFACTURE_OFF;
                }
            //Fall Through Intentional
#if defined INTEL_WP2_USB
        case HW_CFG_MEMWRITE:
                {
        char line[1024];
                memset(line, 0, 1024);
                BTHWDBG("HW_CFG_MEMWRITE");
                if(!feof(hw_cfg_cb.fw_fd)) {
                    BTHWDBG("file not empty");
                    fgets(line, sizeof(line), hw_cfg_cb.fw_fd);

                    while((line[0] == '*') || (line[0] == 0xd ) || (line[0] == 'F') || (line[1] == '2')){
                        if(feof(hw_cfg_cb.fw_fd)) {
                            BTHWDBG("End of file");
                            if(hw_cfg_cb.fw_fd != NULL)
                            {
                                fclose(hw_cfg_cb.fw_fd);
                                hw_cfg_cb.fw_fd = NULL;
                            }

                            if(fw_patchfile_empty != 0)
                            {
                                hw_cfg_cb.is_patch_enabled = 2;
                                hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                                goto HW_CFG_MANUFACTURE_OFF;
                            }
                            else
                            {
                                BTHWDBG("Patch file is empty");
                                hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                                goto HW_CFG_MANUFACTURE_OFF;
                            }
                            break;
                        }

                        fgets(line, sizeof(line), hw_cfg_cb.fw_fd);
                    }
                    if((line[0] == '0') && (line[1] == '1')){
                        int length = 0;
                        int parameter_length = 0;
                        uint8_t opcodeByte[2];
                        uint16_t opcode = 0;
                        opcodeByte[0] = form_byte(line[3], line[4]);
                        opcodeByte[1] = form_byte(line[5], line[6]);
                        opcode = form_word(opcodeByte[1], opcodeByte[0]);
                        length = strlen(line);

                        UINT16_TO_STREAM(p, opcode);
                        parameter_length = form_byte(line[8], line[9]);
                        *p = parameter_length;  /* parameter length */
                        p++;
                        p_buf->len = HCI_CMD_PREAMBLE_SIZE + parameter_length;
                        BTHWDBG("Length =%X, %d\n", parameter_length, length);

                        for(pos=0; pos < (length - 10); pos++){
                            *p = form_byte(line[10+pos], line[11+pos]);
                            p++;
                            pos++;
                        }

                        fw_patchfile_empty = 1;
                        hw_cfg_cb.state = HW_CFG_MEMWRITE;

                        is_proceeding = bt_vendor_cbacks->xmit_cb(opcode,
                HCI_COMMAND_CMPL_EVT_CODE,\
                            p_buf, hw_config_cback);
                    }
                    break;
                }
                else
                {
                    BTHWDBG("Patch file is empty");
                    hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                    goto HW_CFG_MANUFACTURE_OFF;
                }
                break;
        }
#endif
#if defined INTEL_AG6XX_UART
            case HW_CFG_DL_FW_PATCH:
                BTHWDBG("HW_CFG_DL_FW_PATCH");
                //Read the address to patch
                read = fread(&hw_cfg_cb.address, 1,sizeof(uint32_t), hw_cfg_cb.fw_fd);
                if(!read)
                {
                    //This should never happen
                    ALOGE("Read Fw patch file failed.");
                    hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                    goto HW_CFG_MANUFACTURE_OFF;
                }
                else if(hw_cfg_cb.address == 0xFFFFFFFF)
                {
                    //FW patch download DONE.
                    BTHWDBG("FW patch download DONE.");
                    hw_cfg_cb.is_patch_enabled = 2; //Patch is enabled
                    hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                    goto HW_CFG_MANUFACTURE_OFF;
                }
                else
                {
                    BTHWDBG("Address read:0x%x",hw_cfg_cb.address);
                    //Read the length of the patch
                    read = fread(&hw_cfg_cb.nr_of_bytes, 1 , sizeof(uint32_t), hw_cfg_cb.fw_fd);
                    if(!read)
                    {
                        hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                        goto HW_CFG_MANUFACTURE_OFF;
                    }
                    else
                    {
                        BTHWDBG("hw_cfg_cb.nr_of_bytes:%u", hw_cfg_cb.nr_of_bytes);
                        hw_cfg_cb.state = HW_CFG_DL_FW_PATCH1;
                    }
                }
            case HW_CFG_DL_FW_PATCH1:
                if (hw_cfg_cb.nr_of_bytes > 0)
                {
                    uint8_t data_length = (hw_cfg_cb.nr_of_bytes > PATCH_MAX_LENGTH) ?PATCH_MAX_LENGTH:hw_cfg_cb.nr_of_bytes;
                    pData = (unsigned char *) malloc(data_length);
                    read = fread(pData, data_length, sizeof(unsigned char), hw_cfg_cb.fw_fd);
                    if(!read)
                    {
                        hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF;
                    }
                    else
                    {
                        //BTHWDBG("hw_cfg_cb.address:0x%x data_length:%u", hw_cfg_cb.address, data_length);
                        UINT16_TO_STREAM(p, HCI_INTEL_INF_MEM_WRITE);
                        /* parameter length (address size + mode size + length + data size)*/
                        uint8_t param_length =  4 + 1 + 1 + data_length;
                        *p++ = param_length;
                        *p++ = (hw_cfg_cb.address & 0x000000FF) >> 0;
                        *p++ = (hw_cfg_cb.address & 0x0000FF00) >> 8;
                        *p++ = (hw_cfg_cb.address & 0x00FF0000) >> 16;
                        *p++ = (hw_cfg_cb.address & 0xFF000000) >> 24;
                        BTHWDBG("Address of p:%p",p);
                        *p++ = HCI_INTEL_MEM_WRITE_MODE_BYTE;
                        *p++ = data_length; //Length of the data buffer
                        memcpy (p, pData, data_length);
                        if (pData!=NULL)
                        {
                            free(pData);
                        }
                        //BTHWDBG("param_length:%u", param_length);

                        p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                                     param_length;

                        hw_cfg_cb.nr_of_bytes -= data_length;
                        if (hw_cfg_cb.nr_of_bytes > 0)
                        {
                            hw_cfg_cb.address += data_length;
                            hw_cfg_cb.state = HW_CFG_DL_FW_PATCH1; //More data to write in this location
                        }
                        else
                        {
                            hw_cfg_cb.state = HW_CFG_DL_FW_PATCH; //Retrieve new address
                        }

                        /****************************************************************/
#if (BTHW_DBG == TRUE)
                        BTHWDBG("p_buf->len:%u", p_buf->len);
                        hex_print("Before sending. CMD SENT", (uint8_t*)(p_buf+1), p_buf->len);
#endif
                        /****************************************************************/
                        is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_MEM_WRITE, \
                                                            HCI_COMMAND_CMPL_EVT_CODE,\
                                                            p_buf, hw_config_cback);
                    }
                }
                break;
#endif
            case HW_CFG_MANUFACTURE_OFF:
HW_CFG_MANUFACTURE_OFF:
                BTHWDBG("HW_CFG_MANUFACTURE_OFF");
                UINT16_TO_STREAM(p, HCI_INTEL_MANUFACTURE_MODE);
                *p++ = HCI_INTEL_MANUFACTURE_MODE_PARAM_SIZE; /* parameter length */
                *p++ = 00;
                *p++ = hw_cfg_cb.is_patch_enabled; //0: if patch is not enabled,

                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_MANUFACTURE_MODE_PARAM_SIZE;
                hw_cfg_cb.state = HW_CFG_MANUFACTURE_OFF_CMPL;

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_MANUFACTURE_MODE, \
                                                    HCI_INTEL_STARTUP, \
                                                    p_buf, hw_config_cback);
                break;
            case HW_CFG_MANUFACTURE_OFF_CMPL:
                BTHWDBG("HW_CFG_MANUFACTURE_OFF_CMPL");
                if (bt_vendor_cbacks)
                    bt_vendor_cbacks->dealloc(p_buf);
                hw_cfg_cb.state = HW_CFG_SUCCESS;
                is_proceeding = TRUE;
                break;
            case HW_CFG_SUCCESS:
                BTHWDBG("FIRMWARE INIT SUCCESS...");
                uint8_t *buf = (uint8_t*)(p_evt_buf+1);
                if (bt_vendor_cbacks)
                    bt_vendor_cbacks->dealloc(p_buf);
                /* Startup event is not received */
                if ( buf[2] != HCI_INTEL_STARTUP)
                    break;
                //Report the fw download success
                bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
                hw_cfg_cb.state = 0;

                if (hw_cfg_cb.fw_fd != -1)
                {
                    fclose(hw_cfg_cb.fw_fd);
                    hw_cfg_cb.fw_fd = -1;
                }

                is_proceeding = TRUE;
                break;
            default:
                BTHWDBG("SKIP");
                if (bt_vendor_cbacks)
                    bt_vendor_cbacks->dealloc(p_buf);
                is_proceeding = TRUE;
                break;
        } // switch(hw_cfg_cb.state)
    } // if (p_buf != NULL)

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (is_proceeding == FALSE)
    {
        ALOGE("vendor lib fwcfg aborted!!!");
        if (bt_vendor_cbacks)
        {
            if (p_buf != NULL)
                bt_vendor_cbacks->dealloc(p_buf);

            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }

        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }

        hw_cfg_cb.state = 0;
    }
    pthread_mutex_unlock( &mutex );
}

/*****************************************************************************
**   Hardware Configuration Interface Functions
*****************************************************************************/
#if defined INTEL_WP2_USB
/*******************************************************************************
**
** Function        hw_config_start
**
** Description     Kick off controller initialization process
**
** Returns         None
**
*******************************************************************************/
void hw_config_start(void)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;

    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.is_patch_enabled = 0; //Patch is not enabled

    /* Start from sending HCI_RESET */

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *) (p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0; /* parameter length */

        hw_cfg_cb.state = HW_CFG_MANUFACTURE_ON;

        bt_vendor_cbacks->xmit_cb(HCI_RESET, HCI_INTEL_DEFAULT_BD_DATA,\
                                                p_buf, hw_config_cback);
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib fw conf aborted [no buffer]");
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}
#endif

/*******************************************************************************
**
** Function        hw_lpm_enable
**
** Description     Enalbe/Disable LPM
**
** Returns         TRUE/FALSE
**
*******************************************************************************/
uint8_t hw_lpm_enable(uint8_t turn_on)
{
    uint8_t ret = FALSE;
#if defined INTEL_AG6XX_UART
    /*
     *Implement LPM enable sequence
     */
#elif defined INTEL_WP2_USB

#endif
    return ret;
}

/*******************************************************************************
**
** Function        hw_lpm_get_idle_timeout
**
** Description     Calculate idle time based on host stack idle threshold
**
** Returns         idle timeout value
**
*******************************************************************************/
uint32_t hw_lpm_get_idle_timeout(void)
{
    uint32_t timeout_ms;
    //TODO: read value from conf file. Now phase 1: hardcode it
    timeout_ms = 25;
    return timeout_ms;
}

/*******************************************************************************
**
** Function        hw_lpm_set_wake_state
**
** Description     Assert/Deassert BT_WAKE
**
** Returns         None
**
*******************************************************************************/
void hw_lpm_set_wake_state(uint8_t wake_assert)
{
    uint8_t state = (wake_assert) ? UPIO_ASSERT : UPIO_DEASSERT;

    //upio_set(UPIO_BT_WAKE, state, lpm_param.bt_wake_polarity);
}

#if (SCO_CFG_INCLUDED == TRUE)
/*******************************************************************************
**
** Function         hw_sco_config
**
** Description      Configure SCO related hardware settings
**
** Returns          None
**
*******************************************************************************/
void hw_sco_config(void)
{
    /* Implement if any vendor specific SCO init is needed. */
}
#endif  // SCO_CFG_INCLUDED

/*******************************************************************************
**
** Function        hw_set_patch_file_path
**
** Description     Set the location of firmware patch file
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_path(char *p_conf_name, char *p_conf_value, int param)
{

    strcpy(fw_patchfile_path, p_conf_value);

    return 0;
}

/*******************************************************************************
**
** Function        hw_set_patch_file_name
**
** Description     Give the specific firmware patch filename
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_name(char *p_conf_name, char *p_conf_value, int param)
{

    strcpy(fw_patchfile_name, p_conf_value);

    return 0;
}

#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
/*******************************************************************************
**
** Function        hw_set_patch_settlement_delay
**
** Description     Give the specific firmware patch settlement time in milliseconds
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_settlement_delay(char *p_conf_name, char *p_conf_value, int param)
{
    fw_patch_settlement_delay = atoi(p_conf_value);

    return 0;
}
#endif  //VENDOR_LIB_RUNTIME_TUNING_ENABLED

/*****************************************************************************
**   Sample Codes Section
*****************************************************************************/

#if (HW_END_WITH_HCI_RESET == TRUE)
/*******************************************************************************
**
** Function         hw_epilog_cback
**
** Description      Callback function for Command Complete Events from HCI
**                  commands sent in epilog process.
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_cback(void *p_mem)
{
    HC_BT_HDR   *p_evt_buf = (HC_BT_HDR *) p_mem;
    uint8_t     *p, status;
    uint16_t    opcode;

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode,p);

    BTHWDBG("%s Opcode:0x%04X Status: %d", __FUNCTION__, opcode, status);

    if (bt_vendor_cbacks)
    {
        /* Must free the RX event buffer */
        bt_vendor_cbacks->dealloc(p_evt_buf);

        /* Once epilog process is done, must call epilog_cb callback
           to notify caller */
        bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
    }
}

/*******************************************************************************
**
** Function         hw_epilog_process
**
** Description      Sample implementation of epilog process
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_process(void)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;

    BTHWDBG("hw_epilog_process");

    /* Sending a HCI_RESET */
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *) (p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0; /* parameter length */

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_epilog_cback);
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib epilog process aborted [no buffer]");
            bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}
#endif // (HW_END_WITH_HCI_RESET == TRUE)
