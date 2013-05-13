/******************************************************************************
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

#include <utils/Log.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <cutils/properties.h>
#include <stdlib.h>
#include "bt_hci_bdroid.h"
#include "bt_vendor_brcm.h"
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

#define FW_PATCHFILE_EXTENSION      ".pbn"
#define FW_PATCHFILE_EXTENSION_LEN  4
#define FW_PATCHFILE_PATH_MAXLEN    248 /* Local_Name length of return of
                                           HCI_Read_Local_Name */

#define HCI_CMD_MAX_LEN             258

#define HCI_RESET                               0x0C03
#define HCI_VSC_WRITE_UART_CLOCK_SETTING        0xFC45
#define HCI_VSC_UPDATE_BAUDRATE                 0xFC18
#define HCI_READ_LOCAL_NAME                     0x0C14
#define HCI_VSC_DOWNLOAD_MINIDRV                0xFC2E
#define HCI_VSC_WRITE_BD_ADDR                   0xFC01
#define HCI_VSC_WRITE_SLEEP_MODE                0xFC27
#define HCI_VSC_WRITE_SCO_PCM_INT_PARAM         0xFC1C
#define HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM     0xFC1E
#define HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM    0xFC6D
#define HCI_VSC_LAUNCH_RAM                      0xFC4E
#define HCI_READ_LOCAL_BDADDR                   0x1009

#define HCI_INTEL_INF_MANUFACTURE               0xFC11
#define HCI_INTEL_INF_BDDATA                    0xFC2F
#define HCI_INTEL_INF_SET_UART_BAUD             0xFC06
#define HCI_INTEL_INF_READ_SW_VERSION           0xFC05
#define HCI_INTEL_INF_MEM_WRITE                 0xFC8E

#define INTEL_MEM_WRITE_MODE_BYTE                0
#define HCI_INTEL_INF_MANUFACTURE_PARAM_SIZE     2
#define HCI_INTEL_INF_SET_UART_BAUD_PARAM_SIZE   1
#define HCI_INTEL_INF_READ_SW_VERSION_PARAM_SIZE 0
#define HCI_INTEL_INF_BDDATA_PARAM_SIZE          80

#define HCI_EVT_READ_HW_VARIANT                  7
#define HCI_EVT_READ_HW_REVISION                 8

#define HCI_EVT_CMD_CMPL_STATUS_RET_BYTE        5
#define HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING      6
#define HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY     6
#define HCI_EVT_CMD_CMPL_OPCODE                 3
#define LPM_CMD_PARAM_SIZE                      12
#define UPDATE_BAUDRATE_CMD_PARAM_SIZE          6
#define HCI_CMD_PREAMBLE_SIZE                   3
#define HCD_REC_PAYLOAD_LEN_BYTE                2
#define BD_ADDR_LEN                             6
#define LOCAL_NAME_BUFFER_LEN                   32
#define LOCAL_BDADDR_PATH_BUFFER_LEN            256

#define PATCH_MAX_LENGTH 244

#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}

/******************************************************************************
**  Local type definitions
******************************************************************************/

/* Hardware Configuration State */
enum {
    HW_CFG_START = 1,
    HW_CFG_INF_MANUFACTURE_ON,  /* Intel manufature mode on */
    HW_CFG_INF_MANUFACTURE_OFF, /* Intel manufature mode off */
    HW_CFG_INF_BDDATA,          /* Open bddata file and download the BD data */
    HW_SET_BAUD_HS,             /* Change the controller to higher baud rate*/
    HW_SET_HOST_BAUD,           /* Change the host baud rate */
    HW_CFG_SUCCESS,             /* Controller init finished */
    SW_READ_VERSION,            /* Read SW version from controller to construct required patch filename */
    HW_CFG_DL_MINIDRIVER,       /* Insert settlement delay before patch download. Tune in platform */
    SW_FIND_PATCH,              /* Find the required patch file in file system */
    HW_CFG_DL_FW_PATCH,         /* Donwload the firmware patch file */
    HW_CFG_DL_FW_PATCH1
#if (USE_CONTROLLER_BDADDR == TRUE)
    , HW_CFG_READ_BD_ADDR
#endif
};

/* h/w config control block */
typedef struct
{
    uint8_t state;                          /* Hardware configuration state */
    FILE*   fw_fd;                          /* FW patch file pointer */
    uint8_t is_patch_enabled;               /* Is patch is enabled? 2: enabled 1:not enabled */
    uint32_t address;                       /* Address in which data has to be written */
    uint32_t nr_of_bytes;                   /* No of bytes to be written field in pbn file */
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

/* Firmware re-launch settlement time */
typedef struct {
    const char *chipset_name;
    const uint32_t delay_time;
} fw_settlement_entry_t;


/******************************************************************************
**  Externs
******************************************************************************/

void hw_config_cback(void *p_evt_buf);
extern uint8_t vnd_local_bd_addr[BD_ADDR_LEN];


/******************************************************************************
**  Static variables
******************************************************************************/

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

#if (!defined(SCO_USE_I2S_INTERFACE) || (SCO_USE_I2S_INTERFACE == FALSE))
static uint8_t bt_sco_param[SCO_PCM_PARAM_SIZE] =
{
    SCO_PCM_ROUTING,
    SCO_PCM_IF_CLOCK_RATE,
    SCO_PCM_IF_FRAME_TYPE,
    SCO_PCM_IF_SYNC_MODE,
    SCO_PCM_IF_CLOCK_MODE
};

static uint8_t bt_pcm_data_fmt_param[PCM_DATA_FORMAT_PARAM_SIZE] =
{
    PCM_DATA_FMT_SHIFT_MODE,
    PCM_DATA_FMT_FILL_BITS,
    PCM_DATA_FMT_FILL_METHOD,
    PCM_DATA_FMT_FILL_NUM,
    PCM_DATA_FMT_JUSTIFY_MODE
};
#else
static uint8_t bt_sco_param[SCO_I2SPCM_PARAM_SIZE] =
{
    SCO_I2SPCM_IF_MODE,
    SCO_I2SPCM_IF_ROLE,
    SCO_I2SPCM_IF_SAMPLE_RATE,
    SCO_I2SPCM_IF_CLOCK_RATE
};
#endif

/*
 * The look-up table of recommended firmware settlement delay (milliseconds) on
 * known chipsets.
 */
static const fw_settlement_entry_t fw_settlement_table[] = {
    {"BCM43241", 200},
    {(const char *) NULL, 100}  // Giving the generic fw settlement delay setting.
};

/******************************************************************************
**  Static functions
******************************************************************************/

/******************************************************************************
**  Controller Initialization Static Functions
******************************************************************************/

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
                        //BTHWDBG("Found patchfile. Store location and name");
                        free(p_chip_id_str);
                        p_chip_id_str = NULL;
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
        ALOGE("vendor/firmware doesn't exist");
    }

    if (p_chip_id_str)
    {
        free(p_chip_id_str);
        p_chip_id_str = NULL;
    }
    return (retval);
}

unsigned char char_to_hex(char c)
{
    if (c == '0')
        return 0x00;
    else if (c == '1')
        return 0x01;
    else if (c == '2')
        return 0x02;
    else if (c == '3')
        return 0x03;
    else if (c == '4')
        return 0x04;
    else if (c == '5')
        return 0x05;
    else if (c == '6')
        return 0x06;
    else if (c == '7')
        return 0x07;
    else if (c == '8')
        return 0x08;
    else if (c == '9')
        return 0x09;
    else if (c == 'A' || c == 'a')
        return 0x0a;
    else if (c == 'B' || c == 'b')
        return 0x0b;
    else if (c == 'C' || c == 'c')
        return 0x0c;
    else if (c == 'D' || c == 'd')
        return 0x0d;
    else if (c == 'E' || c == 'e')
        return 0x0e;
    else if (c == 'F' || c == 'f')
        return 0x0f;
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
    FILE * fp;
    int i, ret;
    unsigned char c;
    char* line = NULL;
    int read;
    size_t cmd_size;

    BTHWDBG("%s",__func__);
    //FIXME: Replace with conf file option
    fp = fopen("/system/etc/bluetooth/bddata", "rb");
    if (fp == NULL)
    {
        ALOGE("cannot open: /etc/bluetooth/bddata");
        return -1;
       }
    line = (uint8_t*) malloc (1024 * sizeof (uint8_t));
    if (line == NULL)
    {
        BTHWDBG("Malloc failure");
        return -1;
    }
       read = fread(line, sizeof(unsigned char), 1024, fp);
       if (read < 0)
       {
           ALOGE("line is not read properly. read:%d errno:%d strerror:%s", read, errno, strerror(errno));
           free(line);
           return -1;
    }
    BTHWDBG("number characters read:%u line:%s", read, line);
       line[read] = '\0';
    cmd_size = read;
    BTHWDBG("cmd_size:%u bddata:%s",cmd_size, line);
    for(i=0; i<cmd_size; i++)
    {
        if (i%2 == 1)
        {
            c |= char_to_hex(line[i]);
            //ALOGE("%x",c);
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
    return 0; //success
}

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
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    char        *p_name, *p_tmp;
    uint8_t     *p, status;
    uint16_t    opcode;
    HC_BT_HDR  *p_buf=NULL;
    uint8_t     is_proceeding = FALSE;
    int read;
    uint8_t* pData = NULL;
#if (USE_CONTROLLER_BDADDR == TRUE)
    const uint8_t null_bdaddr[BD_ADDR_LEN] = {0,0,0,0,0,0};
#endif

ALOGE("%s",__func__);

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode,p);

/************************************
    HACK.
    FIXME: Need to check Command status event properly.
************************************/
    //*p = 0;
    status = 0;

    ALOGE("p_evt_buf->event:0x%x p_evt_buf->len:0x%x p_evt_buf->offset:0x%x p_evt_buf->layer_specific:0x%x",\
        p_evt_buf->event, p_evt_buf->len, p_evt_buf->offset, p_evt_buf->layer_specific);
    ALOGE("status:%u p:%x",status, *p);
    /* Ask a new buffer big enough to hold any HCI commands sent in here */
    if ((status == 0) && bt_vendor_cbacks)
    {
        ALOGE("Allocating p_buf");
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
            case HW_SET_BAUD_HS:
                BTHWDBG("HW_SET_BAUD_HS");
                ms_delay(10);
                UINT16_TO_STREAM(p, HCI_INTEL_INF_SET_UART_BAUD);
                *p++ = HCI_INTEL_INF_SET_UART_BAUD_PARAM_SIZE; /* parameter length */
                *p++ = 0x0A; //Baud rate 2M

                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_INF_SET_UART_BAUD_PARAM_SIZE;
                hw_cfg_cb.state = HW_SET_HOST_BAUD;

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_SET_UART_BAUD, \
                                                    p_buf, NULL);
                //ms_delay(100);

            // fall through intentional
           case HW_SET_HOST_BAUD:
                   BTHWDBG("HW_SET_HOST_BAUD");
                   //serial_setbaud(2000000);

                   //ms_delay(2500); //Delay of 2.5 sec. As SetUartBaudRate command in the controller takes 2 sec to get ready
                //bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
                //break;
            case HW_CFG_START:
                BTHWDBG("HW_CFG_START");
                //ms_delay(10);
                UINT16_TO_STREAM(p, HCI_INTEL_INF_MANUFACTURE);
                *p++ = HCI_INTEL_INF_MANUFACTURE_PARAM_SIZE; /* parameter length */
                *p++ = 01;
                *p++ = 00;

                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_INF_MANUFACTURE_PARAM_SIZE;

                hw_cfg_cb.state = HW_CFG_INF_BDDATA; //FIXME: Skippping baud rate change for the time being

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_MANUFACTURE, \
                                                    p_buf, hw_config_cback);

                break;
            case HW_CFG_INF_BDDATA:
                BTHWDBG("HW CFG_INF_BDDATA.");
                ms_delay(50);
                UINT16_TO_STREAM(p, HCI_INTEL_INF_BDDATA);
                *p++ = HCI_INTEL_INF_BDDATA_PARAM_SIZE; /* parameter length */
                int ret = open_bddata(p);
                if (ret == 0)
                {
                    ALOGE("open_bddata success. Sending HCI command...");
                    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_INF_BDDATA_PARAM_SIZE;
                    hw_cfg_cb.state = SW_READ_VERSION;

                    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_BDDATA, \
                                                    p_buf, hw_config_cback);

                } else
                {
                    //FIXME should turn off manufacture mode and gracefully fail
                    ALOGE("open_bddata FAILED.");
                    bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
                }
                break;
            case SW_READ_VERSION:
                BTHWDBG("HW_READ_VERSION");
                UINT16_TO_STREAM(p, HCI_INTEL_INF_READ_SW_VERSION);
                *p++ = HCI_INTEL_INF_READ_SW_VERSION_PARAM_SIZE; /* parameter length */
                *p++ = 00;

                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_INF_READ_SW_VERSION_PARAM_SIZE;
                hw_cfg_cb.state = SW_FIND_PATCH;

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_READ_SW_VERSION, \
                                                    p_buf, hw_config_cback);
                break;
            case SW_FIND_PATCH:
                BTHWDBG("SW_FIND_PATCH");
                //Find the patch. and open the patch file
                char* p_patch_file = NULL;
                uint8_t* temp = (p_evt_buf + 1);
                uint8_t hw_variant = temp[HCI_EVT_READ_HW_VARIANT];
                uint8_t hw_revision = temp[HCI_EVT_READ_HW_REVISION];
                uint16_t dev_id = (hw_variant<<8) | (hw_revision<<0);
                BTHWDBG("hw_varient:0x%x hw_revision:0x%x Device id:0x%x", hw_variant, hw_revision,dev_id);
                uint8_t* fw_patch_name = (uint8_t*)malloc (sizeof (uint8_t)*4); //AG620 file name should be a00.pbn unless specified in conf file
                sprintf(fw_patch_name, "%x\0", dev_id);

                //BTHWDBG("FW patch file name constructed:%s",fw_patch_name);

                if (hw_config_findpatch(fw_patch_name, &p_patch_file) == TRUE)
                {
                    //Open the patch file
                    BTHWDBG("Open the patch file:%s",p_patch_file);
                    if ((hw_cfg_cb.fw_fd = fopen(p_patch_file, "rb")) == NULL) {
                        ALOGE("Cannot open %s", p_patch_file);
                        if (p_patch_file)
                        {
                            free (p_patch_file);
                            p_patch_file = NULL;
                        }
                        hw_cfg_cb.state = HW_CFG_INF_MANUFACTURE_OFF;
                        goto HW_CFG_INF_MANUFACTURE_OFF;
                    }
                    else
                    {
                        BTHWDBG("Donwload FW will begin");
                        if (p_patch_file)
                        {
                            free (p_patch_file);
                            p_patch_file = NULL;
                        }
                        hw_cfg_cb.state = HW_CFG_DL_MINIDRIVER;
                    }
                }
                else
                {
                    //Patch file not found. Nothing to do
                    BTHWDBG("Patch file not found");
                    hw_cfg_cb.state = HW_CFG_INF_MANUFACTURE_OFF;
                    goto HW_CFG_INF_MANUFACTURE_OFF;
                }
            case HW_CFG_DL_MINIDRIVER:
                /* Give FW time to settle */
                ms_delay(100);
                hw_cfg_cb.state = HW_CFG_DL_FW_PATCH;
            case HW_CFG_DL_FW_PATCH:
                BTHWDBG("HW_CFG_DL_FW_PATCH");
                //Read the address to patch
                read = fread(&hw_cfg_cb.address, 1,sizeof(uint32_t), hw_cfg_cb.fw_fd);
                if(!read)
                {
                    //This should never happen
                    ALOGE("Read Fw patch file failed.");
                    hw_cfg_cb.state = HW_CFG_INF_MANUFACTURE_OFF;
                    goto HW_CFG_INF_MANUFACTURE_OFF;
                }
                else if(hw_cfg_cb.address == 0xFFFFFFFF)
                {
                    //FW patch download DONE.
                    BTHWDBG("FW patch download DONE.");
                    hw_cfg_cb.is_patch_enabled = 2; //Patch is enabled
                    hw_cfg_cb.state = HW_CFG_INF_MANUFACTURE_OFF;
                    goto HW_CFG_INF_MANUFACTURE_OFF;
                }
                else
                {
                    BTHWDBG("Address read:0x%x",hw_cfg_cb.address);
                    //Read the length of the patch
                    read = fread(&hw_cfg_cb.nr_of_bytes, 1 , sizeof(uint32_t), hw_cfg_cb.fw_fd);
                    if(!read)
                    {
                        hw_cfg_cb.state = HW_CFG_INF_MANUFACTURE_OFF;
                        goto HW_CFG_INF_MANUFACTURE_OFF;
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
                        hw_cfg_cb.state = HW_CFG_INF_MANUFACTURE_OFF;
                    }
                    else
                    {
                        //BTHWDBG("hw_cfg_cb.address:0x%x data_length:%u", hw_cfg_cb.address, data_length);
                        //hex_print("pData", pData, data_length);
                        UINT16_TO_STREAM(p, HCI_INTEL_INF_MEM_WRITE);
                        uint8_t param_length =  4 + 1 + 1 + data_length; /* parameter length (address size + mode size + length + data size)*/
                        *p++ = param_length;
                        *p++ = (hw_cfg_cb.address & 0x000000FF) >> 0;
                        *p++ = (hw_cfg_cb.address & 0x0000FF00) >> 8;
                        *p++ = (hw_cfg_cb.address & 0x00FF0000) >> 16;
                        *p++ = (hw_cfg_cb.address & 0xFF000000) >> 24;
                        BTHWDBG("Address of p:%p",p);
                        *p++ = INTEL_MEM_WRITE_MODE_BYTE;
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
                        BTHWDBG("p_buf->len:%u", p_buf->len);
                        hex_print("Before sending. CMD SENT", (uint8_t*)(p_buf+1), p_buf->len);
                        /****************************************************************/
                        ms_delay(10);
                        is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_MEM_WRITE, \
                                                            p_buf, hw_config_cback);
                    }
                }
                break;
            case HW_CFG_INF_MANUFACTURE_OFF:
HW_CFG_INF_MANUFACTURE_OFF:
                BTHWDBG("HW_CFG_INF_MANUFACTURE_OFF");
                ms_delay(100);
                UINT16_TO_STREAM(p, HCI_INTEL_INF_MANUFACTURE);
                *p++ = HCI_INTEL_INF_MANUFACTURE_PARAM_SIZE; /* parameter length */
                *p++ = 00;
                *p++ = hw_cfg_cb.is_patch_enabled; //1: if patch is not enabled,

                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             HCI_INTEL_INF_MANUFACTURE_PARAM_SIZE;
                hw_cfg_cb.state = HW_CFG_SUCCESS;

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_MANUFACTURE, \
                                                    p_buf, hw_config_cback);

                break;
            case HW_CFG_SUCCESS:
                BTHWDBG("FIRMWARE INIT SUCCESS...");
                //Report the fw download success
                //bt_vendor_cbacks->dealloc(p_buf); //TODO: It seems this is not needed. Analyze.....
                bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
                hw_cfg_cb.state = 0;

                if (hw_cfg_cb.fw_fd != -1)
                {
                    fclose(hw_cfg_cb.fw_fd);
                    hw_cfg_cb.fw_fd = -1;
                }

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
}

/******************************************************************************
**   LPM Static Functions
******************************************************************************/

/*******************************************************************************
**
** Function         hw_lpm_ctrl_cback
**
** Description      Callback function for lpm enable/disable rquest
**
** Returns          None
**
*******************************************************************************/
void hw_lpm_ctrl_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->lpm_cb(status);
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}


#if (SCO_CFG_INCLUDED == TRUE)
/*****************************************************************************
**   SCO Configuration Static Functions
*****************************************************************************/

/*******************************************************************************
**
** Function         hw_sco_cfg_cback
**
** Description      Callback function for SCO configuration rquest
**
** Returns          None
**
*******************************************************************************/
void hw_sco_cfg_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    uint8_t     *p;
    uint16_t    opcode;
    HC_BT_HDR  *p_buf=NULL;

    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode,p);

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

#if (!defined(SCO_USE_I2S_INTERFACE) || (SCO_USE_I2S_INTERFACE == FALSE))
    if (opcode == HCI_VSC_WRITE_SCO_PCM_INT_PARAM)
    {
        uint8_t ret = FALSE;

        /* Ask a new buffer to hold WRITE_PCM_DATA_FORMAT_PARAM command */
        if (bt_vendor_cbacks)
            p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                HCI_CMD_PREAMBLE_SIZE + \
                                                PCM_DATA_FORMAT_PARAM_SIZE);
        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + PCM_DATA_FORMAT_PARAM_SIZE;

            p = (uint8_t *) (p_buf + 1);
            UINT16_TO_STREAM(p, HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM);
            *p++ = PCM_DATA_FORMAT_PARAM_SIZE;
            memcpy(p, &bt_pcm_data_fmt_param, PCM_DATA_FORMAT_PARAM_SIZE);

            if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM,\
                                           p_buf, hw_sco_cfg_cback)) == FALSE)
            {
                bt_vendor_cbacks->dealloc(p_buf);
            }
            else
                return;
        }
    }
#endif  // !SCO_USE_I2S_INTERFACE

if (bt_vendor_cbacks)
    bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
}
#endif // SCO_CFG_INCLUDED

/*****************************************************************************
**   Hardware Configuration Interface Functions
*****************************************************************************/


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

BTHWDBG("+%s",__func__);
    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.is_patch_enabled = 1; //Patch is not enabled

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

        hw_cfg_cb.state = HW_CFG_START;

        bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib fw conf aborted [no buffer]");
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
BTHWDBG("-%s",__func__);
}

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
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;
    uint8_t     ret = FALSE;

    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_PREAMBLE_SIZE + \
                                                       LPM_CMD_PARAM_SIZE);

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + LPM_CMD_PARAM_SIZE;

        p = (uint8_t *) (p_buf + 1);
        UINT16_TO_STREAM(p, HCI_VSC_WRITE_SLEEP_MODE);
        *p++ = LPM_CMD_PARAM_SIZE; /* parameter length */

        if (turn_on)
        {
            memcpy(p, &lpm_param, LPM_CMD_PARAM_SIZE);
            upio_set(UPIO_LPM_MODE, UPIO_ASSERT, 0);
        }
        else
        {
            memset(p, 0, LPM_CMD_PARAM_SIZE);
            upio_set(UPIO_LPM_MODE, UPIO_DEASSERT, 0);
        }

        if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_SLEEP_MODE, p_buf, \
                                        hw_lpm_ctrl_cback)) == FALSE)
        {
            bt_vendor_cbacks->dealloc(p_buf);
        }
    }

    if ((ret == FALSE) && bt_vendor_cbacks)
        bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_FAIL);

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

    /* set idle time to be LPM_IDLE_TIMEOUT_MULTIPLE times of
     * host stack idle threshold (in 300ms/25ms)
     */
    timeout_ms = (uint32_t)lpm_param.host_stack_idle_threshold \
                            * LPM_IDLE_TIMEOUT_MULTIPLE;

/*    if (strstr(hw_cfg_cb.local_chip_name, "BCM4325") != NULL)
        timeout_ms *= 25; // 12.5 or 25 ?
    else
        timeout_ms *= 300;
*/
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

    upio_set(UPIO_BT_WAKE, state, lpm_param.bt_wake_polarity);
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
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p, ret;

#if (!defined(SCO_USE_I2S_INTERFACE) || (SCO_USE_I2S_INTERFACE == FALSE))
    uint16_t cmd_u16 = HCI_CMD_PREAMBLE_SIZE + SCO_PCM_PARAM_SIZE;
#else
    uint16_t cmd_u16 = HCI_CMD_PREAMBLE_SIZE + SCO_I2SPCM_PARAM_SIZE;
#endif

    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE+cmd_u16);

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = cmd_u16;

        p = (uint8_t *) (p_buf + 1);
#if (!defined(SCO_USE_I2S_INTERFACE) || (SCO_USE_I2S_INTERFACE == FALSE))
        UINT16_TO_STREAM(p, HCI_VSC_WRITE_SCO_PCM_INT_PARAM);
        *p++ = SCO_PCM_PARAM_SIZE;
        memcpy(p, &bt_sco_param, SCO_PCM_PARAM_SIZE);
        cmd_u16 = HCI_VSC_WRITE_SCO_PCM_INT_PARAM;
        BTHWDBG("SCO PCM configure {%d, %d, %d, %d, %d}",
           bt_sco_param[0], bt_sco_param[1], bt_sco_param[2], bt_sco_param[3], \
           bt_sco_param[4]);

#else
        UINT16_TO_STREAM(p, HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM);
        *p++ = SCO_I2SPCM_PARAM_SIZE;
        memcpy(p, &bt_sco_param, SCO_I2SPCM_PARAM_SIZE);
        cmd_u16 = HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM;
        BTHWDBG("SCO over I2SPCM interface {%d, %d, %d, %d}",
           bt_sco_param[0], bt_sco_param[1], bt_sco_param[2], bt_sco_param[3]);
#endif

        if ((ret=bt_vendor_cbacks->xmit_cb(cmd_u16, p_buf, hw_sco_cfg_cback)) \
             == FALSE)
        {
            bt_vendor_cbacks->dealloc(p_buf);
        }
        else
            return;
    }

    if (bt_vendor_cbacks)
    {
        ALOGE("vendor lib scocfg aborted");
        bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_FAIL);
    }
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
