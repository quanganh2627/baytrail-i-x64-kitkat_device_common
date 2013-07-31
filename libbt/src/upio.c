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
 *  Filename:      upio.c
 *
 *  Description:   Contains I/O functions, like
 *                      rfkill control
 *                      BT_WAKE/HOST_WAKE control
 *
 ******************************************************************************/

#define LOG_TAG "bt_upio"

#include <utils/Log.h>
#include <fcntl.h>
#include <errno.h>
#include <cutils/properties.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include <poll.h>
#include <pthread.h>
#include "bt_vendor.h"
#include "upio.h"
#include "userial_vendor.h"
#include "imc_idi_bt_ioctl.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef UPIO_DBG
#define UPIO_DBG TRUE
#endif

#if (UPIO_DBG == TRUE)
#define UPIODBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define UPIODBG(param, ...) {}
#endif

/******************************************************************************
**  Local type definitions
******************************************************************************/

#if (BT_WAKE_VIA_PROC == TRUE)

/* proc fs node for enable/disable lpm mode */
#ifndef VENDOR_LPM_PROC_NODE
#define VENDOR_LPM_PROC_NODE "/proc/bluetooth/sleep/lpm"
#endif

/* proc fs node for notifying write request */
#ifndef VENDOR_BTWRITE_PROC_NODE
#define VENDOR_BTWRITE_PROC_NODE "/proc/bluetooth/sleep/btwrite"
#endif

/*
 * Maximum btwrite assertion holding time without consecutive btwrite kicking.
 * This value is correlative(shorter) to the in-activity timeout period set in
 * the bluesleep LPM code. The current value used in bluesleep is 10sec.
 */
#ifndef PROC_BTWRITE_TIMER_TIMEOUT_MS
#define PROC_BTWRITE_TIMER_TIMEOUT_MS   8000
#endif

/* lpm proc control block */
typedef struct
{
    uint8_t btwrite_active;
    uint8_t timer_created;
    timer_t timer_id;
    uint32_t timeout_ms;
} vnd_lpm_proc_cb_t;

static vnd_lpm_proc_cb_t lpm_proc_cb;
#endif

#if INTEL_AG6XX_UART == TRUE
typedef struct
{
    pthread_t     netlink_thread;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    unsigned int netlink_fd;
    unsigned int  CTS_state;
} bt_netlink_cb_t;

static bt_netlink_cb_t netlink_cb;


#endif

/******************************************************************************
**  Static variables
******************************************************************************/

static uint8_t upio_state[UPIO_MAX_COUNT];
static int rfkill_id = -1;
static int bt_emul_enable = 0;
static char *rfkill_state_path = NULL;

static unsigned int netlink_running;
static struct sockaddr_nl dest_addr;
static struct nlmsghdr *nlh = NULL;
static struct iovec iov;
static struct msghdr msg;
/******************************************************************************
**  Static functions
******************************************************************************/

/* for friendly debugging outpout string */
static char *lpm_mode[] = {
    "UNKNOWN",
    "disabled",
    "enabled"
};

static char *lpm_state[] = {
    "UNKNOWN",
    "de-asserted",
    "asserted"
};

enum netlink_message_code { HWUP_HIGH, HWUP_LOW, CTS_HIGH, CTS_LOW };
enum {
    LOW,
    HIGH
};
/*****************************************************************************
**   Bluetooth On/Off Static Functions
*****************************************************************************/
static int is_emulator_context(void)
{
    char value[PROPERTY_VALUE_MAX];

    property_get("ro.kernel.qemu", value, "0");
    UPIODBG("is_emulator_context : %s", value);
    if (strcmp(value, "1") == 0) {
        return 1;
    }
    return 0;
}

static int is_rfkill_disabled(void)
{
    char value[PROPERTY_VALUE_MAX];

    property_get("ro.rfkilldisabled", value, "0");
    UPIODBG("is_rfkill_disabled ? [%s]", value);

    if (strcmp(value, "1") == 0) {
        return UPIO_BT_POWER_ON;
    }

    return UPIO_BT_POWER_OFF;
}

static int init_rfkill()
{
    char path[64];
    char buf[16];
    int fd, sz, id;

    if (is_rfkill_disabled())
        return -1;

    for (id = 0; ; id++)
    {
        snprintf(path, sizeof(path), "/sys/class/rfkill/rfkill%d/type", id);
        fd = open(path, O_RDONLY);
        if (fd < 0)
        {
            ALOGE("init_rfkill : open(%s) failed: %s (%d)\n", \
                 path, strerror(errno), errno);
            return -1;
        }

        sz = read(fd, &buf, sizeof(buf));
        close(fd);

        if (sz >= 9 && memcmp(buf, "bluetooth", 9) == 0)
        {
            rfkill_id = id;
            break;
        }
    }

    asprintf(&rfkill_state_path, "/sys/class/rfkill/rfkill%d/state", rfkill_id);
    return 0;
}

/*****************************************************************************
**   LPM Static Functions
*****************************************************************************/

#if (BT_WAKE_VIA_PROC == TRUE)
/*******************************************************************************
**
** Function        proc_btwrite_timeout
**
** Description     Timeout thread of proc/.../btwrite assertion holding timer
**
** Returns         None
**
*******************************************************************************/
static void proc_btwrite_timeout(union sigval arg)
{
    UPIODBG("..%s..", __FUNCTION__);
    lpm_proc_cb.btwrite_active = FALSE;
}
#endif

/*****************************************************************************
**   UPIO Interface Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        upio_init
**
** Description     Initialization
**
** Returns         None
**
*******************************************************************************/
void upio_init(void)
{
    memset(upio_state, UPIO_UNKNOWN, UPIO_MAX_COUNT);
#if (BT_WAKE_VIA_PROC == TRUE)
    memset(&lpm_proc_cb, 0, sizeof(vnd_lpm_proc_cb_t));
#endif
}

/*******************************************************************************
**
** Function        upio_cleanup
**
** Description     Clean up
**
** Returns         None
**
*******************************************************************************/
void upio_cleanup(void)
{
#if (BT_WAKE_VIA_PROC == TRUE)
    if (lpm_proc_cb.timer_created == TRUE)
        timer_delete(lpm_proc_cb.timer_id);

    lpm_proc_cb.timer_created = FALSE;
#endif
}

#if defined INTEL_WP2_UART
/*******************************************************************************
**
** Function        upio_set_bluetooth_power
**
** Description     Interact with low layer driver to set Bluetooth power
**                 on/off.
**
** Returns         0  : SUCCESS or Not-Applicable
**                 <0 : ERROR
**
*******************************************************************************/
int upio_set_bluetooth_power(int on)
{
    int sz;
    int fd = -1;
    int ret = -1;
    char buffer = '0';

    switch(on)
    {
        case UPIO_BT_POWER_OFF:
            buffer = '0';
            break;

        case UPIO_BT_POWER_ON:
            buffer = '1';
            break;
    }

    if (is_emulator_context())
    {
        /* if new value is same as current, return -1 */
        if (bt_emul_enable == on)
            return ret;

        UPIODBG("set_bluetooth_power [emul] %d", on);

        bt_emul_enable = on;
        return 0;
    }

    /* check if we have rfkill interface */
    if (is_rfkill_disabled())
        return 0;

    if (rfkill_id == -1)
    {
        if (init_rfkill())
            return ret;
    }

    fd = open(rfkill_state_path, O_WRONLY);

    if (fd < 0)
    {
        ALOGE("set_bluetooth_power : open(%s) for write failed: %s (%d)",
            rfkill_state_path, strerror(errno), errno);
        return ret;
    }

    sz = write(fd, &buffer, 1);

    if (sz < 0) {
        ALOGE("set_bluetooth_power : write(%s) failed: %s (%d)",
            rfkill_state_path, strerror(errno),errno);
    }
    else
        ret = 0;

    if (fd >= 0)
        close(fd);

    return ret;
}
#endif

#if defined INTEL_AG6XX_UART
/*******************************************************************************
**
** Function        upio_set_d_state
**
** Description     Set d states: d0,  d0i2, d0i3, d3
**
** Returns         None
**
*******************************************************************************/
void upio_set_d_state(uint8_t state)
{
    userial_vendor_ioctl(USERIAL_OP_SET_DEVICE_STATE, &state);
}

/*******************************************************************************
**
** Function        upio_set_bt_wake_state
**
** Description     Set bt_wake high or low
**
** Returns         None
**
*******************************************************************************/
uint8_t upio_set_bt_wake_state(uint8_t bt_wake_state)
{
    UPIODBG("--->%s..", __FUNCTION__);
    userial_vendor_ioctl(USERIAL_OP_SET_BT_WAKE_UP, &bt_wake_state);
    pthread_mutex_lock(&netlink_cb.mutex);
    do
    {
        pthread_cond_wait(&netlink_cb.cond, &netlink_cb.mutex);
        UPIODBG("%s netlink_cb.CTS_state:%d", __func__, netlink_cb.CTS_state);
    } while(netlink_cb.CTS_state != bt_wake_state);
    pthread_mutex_unlock(&netlink_cb.mutex);
    return netlink_cb.CTS_state;
}

/*******************************************************************************
**
** Function        upio_get_cts_state
**
** Description     Set gets current CTS state
**
** Returns         None
**
*******************************************************************************/
uint8_t upio_get_cts_state(void)
{
    UPIODBG("--->%s..", __FUNCTION__);
    return userial_vendor_ioctl(USERIAL_OP_GET_CTS, NULL);
}

/*******************************************************************************
**
** Function        upio_set_rts_state
**
** Description     Set sets RTS state to high/low
**
** Returns         None
**
*******************************************************************************/
void upio_set_rts_state(uint8_t rts_state)
{
    UPIODBG("--->%s..", __FUNCTION__);
    userial_vendor_ioctl(USERIAL_OP_SET_RTS, &rts_state);
}

/*******************************************************************************
**
** Function       upio_create_netlink_socket
**
** Description    Create a NETLINK Socket
**
** Returns        netlinksocket fd
**
*******************************************************************************/
#define MAX_PAYLOAD 2
#define POLL_TIMEOUT 1000

int upio_create_netlink_socket()
{
    struct sockaddr_nl src_addr;
    UPIODBG("--->%s..", __FUNCTION__);
    /* Grab a free socket */
    netlink_cb.netlink_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_USERSOCK);
    UPIODBG("%s: sock_fd:%d\n", __func__,netlink_cb.netlink_fd);
    /* Did we get a socket open? */
    if(netlink_cb.netlink_fd < 0)
    {
        UPIODBG("%s: sock_fd < 0 %d\n", __func__, errno);
        return -1;
    }

    /* Init the Netlink struct */
    memset(&src_addr, 0, sizeof(src_addr));

    /* Set up a Netlink using our own Process ID */
    src_addr.nl_family = AF_NETLINK;
    src_addr.nl_pid = getpid();
    src_addr.nl_groups = 0;
    /* Bind the Netlink to our Socket */
    bind(netlink_cb.netlink_fd, (struct sockaddr*)&src_addr, sizeof(src_addr));
    UPIODBG("%s: pid:%d\n", __func__,src_addr.nl_pid);

    return netlink_cb.netlink_fd;
}

/*******************************************************************************
**
** Function       upio_netlink_send_msg
**
** Description    send a msg to kernel
**
** Returns        None
**
*******************************************************************************/
void upio_netlink_send_msg()
{

    UPIODBG("--->%s..", __FUNCTION__);
    /* Init the destination Netlink struct */
    memset(&dest_addr, 0, sizeof(dest_addr));

    /* Set up the destination Netlink struct using unicast */
    dest_addr.nl_family = AF_NETLINK;
    dest_addr.nl_pid = 0;               /* For Linux Kernel */
    dest_addr.nl_groups = 0;            /* unicast */

    /* Allocate memory for the Netlink message struct */
    nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(MAX_PAYLOAD));
    memset(nlh, 0, NLMSG_SPACE(MAX_PAYLOAD));
    nlh->nlmsg_len = NLMSG_SPACE(MAX_PAYLOAD);
    nlh->nlmsg_pid = getpid();
    nlh->nlmsg_flags = 0;

    /* Inject our IPC message */
    strcpy(NLMSG_DATA(nlh), "1");

    iov.iov_base = (void *)nlh;
    iov.iov_len = nlh->nlmsg_len;
    memset(&msg, 0 ,sizeof(msg));
    msg.msg_name = (void *)&dest_addr;
    msg.msg_namelen = sizeof(dest_addr);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    UPIODBG("%s: sock_fd:%d\n", __func__,netlink_cb.netlink_fd);
    if(sendmsg(netlink_cb.netlink_fd, &msg, 0) < 0)
    {
        ALOGE("%s: sendmsg() failed [errno=%u]\n", __func__, errno);
    }
}

/*******************************************************************************
**
** Function       upio_netlink_listen_msg
**
** Description    start a thread to listen netlink packets from kernel
**
** Returns        status of thread creation
**
*******************************************************************************/
int upio_netlink_listen_thread(void)
{
    pthread_t recvmsgid;
    int status;
    UPIODBG("--->%s..", __FUNCTION__);
    status = pthread_create( &recvmsgid, NULL, upio_netlink_receive_message, NULL);
    if(status == 0)
    {
        netlink_running = TRUE;
        pthread_mutex_init(&netlink_cb.mutex, NULL);
        pthread_cond_init(&netlink_cb.cond, NULL);
        UPIODBG(" upio_netlink_listen_thread created succesfully");
    }else
    {
        UPIODBG(" upio_netlink_listen_thread creation failed")
    }
    return status;
}

/*******************************************************************************
**
** Function       upio_netlink_receive_message
**
** Description    thread to listen netlink packets from kernel
**
** Returns        None
**
*******************************************************************************/
void * upio_netlink_receive_message(void *ptr)
{

    int signal;
    struct pollfd fds[1];
    int n;
    UPIODBG("--->%s..", __FUNCTION__);
    /* Allocate memory for the Netlink message struct */
    nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(MAX_PAYLOAD));
    memset(nlh, 0, NLMSG_SPACE(MAX_PAYLOAD));
    nlh->nlmsg_len = NLMSG_SPACE(MAX_PAYLOAD);
    nlh->nlmsg_pid = getpid();
    nlh->nlmsg_flags = 0;

    iov.iov_base = (void *)nlh;
    iov.iov_len = nlh->nlmsg_len;
    msg.msg_name = NULL;
    msg.msg_namelen = 0;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    fds[0].fd = netlink_cb.netlink_fd;
    fds[0].events = POLLIN | POLLERR | POLLRDNORM;


    while(TRUE)
    {
        n = poll(fds, 1, POLL_TIMEOUT);
        if(0 == n)
        {
            if(netlink_running == FALSE)
                return NULL;
            else
                continue;
        }else if(n < 0)
        {
            ALOGE("Netlink thread exiting ");
            return NULL;
        }

        if( recvmsg(netlink_cb.netlink_fd, &msg, 0) < 0 )
        {
            ALOGE("%s: recvmsg() failed [errno=%u]\n", __func__, errno);
        }
        else
        {
            memcpy((void*)&signal,NLMSG_DATA(nlh),sizeof(unsigned int));
            switch(signal)
            {
                case CTS_HIGH :
                    {
                        pthread_mutex_lock(&netlink_cb.mutex);
                        pthread_cond_signal(&netlink_cb.cond);
                        pthread_mutex_unlock(&netlink_cb.mutex);
                        netlink_cb.CTS_state = HIGH;
                        UPIODBG("%s  netlink_cb.CTS_state:%d", __func__, netlink_cb.CTS_state);
                    }
                break;
                case HWUP_HIGH:
                    if(bt_vendor_cbacks)
                        bt_vendor_cbacks->set_host_wake_state_cb(HIGH);
                break;
                case HWUP_LOW:
                    if(bt_vendor_cbacks)
                        bt_vendor_cbacks->set_host_wake_state_cb(LOW);
                break;
                case CTS_LOW:
                    {
                        pthread_mutex_lock(&netlink_cb.mutex);
                        pthread_cond_signal(&netlink_cb.cond);
                        pthread_mutex_unlock(&netlink_cb.mutex);
                        netlink_cb.CTS_state = LOW;
                        UPIODBG("%s  netlink_cb.CTS_state:%d", __func__, netlink_cb.CTS_state);
                    }
                break;
            }
        }
    }
}

/*******************************************************************************
**
** Function       upio_close_netlink_socket
**
** Description    thread to listen netlink packets from kernel
**
** Returns        None
**
*******************************************************************************/
void upio_close_netlink_socket()
{
    /* Cleanup the socket */
    UPIODBG("--->%s..", __FUNCTION__);
    netlink_running = FALSE;
    close(netlink_cb.netlink_fd);
}
#endif

