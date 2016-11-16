/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"
#include "bsp/bsp.h"
#include "os/os.h"

#include "hal/hal_gpio.h"

/* BLE */
#include "nimble/ble.h"
#include "controller/ble_ll.h"
#include "host/ble_hs.h"

/* RAM HCI transport. */
#include "transport/ram/ble_hci_ram.h"

/* RAM persistence layer. */
#include "store/ram/ble_store_ram.h"

/* Mandatory services. */
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Application-specified header. */
#include "blecent.h"

/** Log data. */
struct log blecent_log;

/* For LED toggling */
static int g_led_pin;

/** blecent task settings. */
#define BLECENT_TASK_PRIO           1
#define BLECENT_STACK_SIZE          (OS_STACK_ALIGN(336))

struct os_eventq blecent_evq;
struct os_task blecent_task;
bssnz_t os_stack_t blecent_stack[BLECENT_STACK_SIZE];

/** Our global device address (public) */
uint8_t g_dev_addr[BLE_DEV_ADDR_LEN] = {0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c};

/** Our random address (in case we need it) */
uint8_t g_random_addr[BLE_DEV_ADDR_LEN];

static int blecent_gap_event(struct ble_gap_event *event, void *arg);

const struct ble_gap_white_entry peer_white_list[] =
    {
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x00}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x01}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x02}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x03}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x04}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x05}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x06}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x07}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x08}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x09}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x10}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x11}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x12}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x13}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x14}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x15}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x16}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x17}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x18}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x19}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x20}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x21}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x22}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x23}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x24}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x25}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x26}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x27}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x28}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x29}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x30}},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x31}}
    };

static const struct ble_gap_conn_params ble_gap_conn_params = {
    .scan_itvl = 0x0010,                                         /*  0x0010  */
    .scan_window = 0x0010,                                       /*  0x0010  */
    .itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN,                   /*  30ms    */
    .itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX * 2,               /*  100ms   */
    .latency = BLE_GAP_INITIAL_CONN_LATENCY,                     /*  0       */
    .supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT,  /*  0x0100  */
    .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,               /*  0x0010  */
    .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,               /*  0x0300  */
};
#if 0
static char uart_rx_buf[15];
static char uart_tx_buf[128];
static int uart_rx_idx;
static int uart_tx_idx;
static int cmd_len;
static struct uart_dev *udv;

/*
 * Called by UART driver to send out next character.
 *
 * Interrupts disabled when nmgr_uart_tx_char/nmgr_uart_rx_char are called.
 */
static int
app_uart_tx_char(void *arg)
{
    if (uart_tx_idx < cmd_len) {
        return uart_tx_buf[uart_tx_idx++];
    } else {
        return -1;
    }
}

/*
 * Receive a character from UART.
 */
static int
app_uart_rx_char(void *arg, uint8_t data)
{
    int rc;
    int cmd;

    uart_rx_buf[uart_rx_idx++] = data;
    if (uart_rx_idx >= 7) {
        cmd = uart_rx_buf[2];
        if (cmd == 2) {
            os_eventq_put(&blecent_evq, &ble_scan_ev);
        } else if (cmd == 3) {
            os_eventq_put(&blecent_evq, &ble_tx_ev);
        }
        uart_rx_idx = 0;
        //memset(uart_rx_buf, 0, 7);
    }

    rc = 0;
    return rc;
}

static void
app_uart_init(void)
{
    struct uart_conf uc = {
        .uc_speed = 9600,
        .uc_databits = 8,
        .uc_stopbits = 1,
        .uc_parity = UART_PARITY_NONE,
        .uc_flow_ctl = UART_FLOW_CTL_NONE,
        .uc_tx_char = app_uart_tx_char,
        .uc_rx_char = app_uart_rx_char,
        .uc_cb_arg = NULL
    };

    udv = (struct uart_dev *)os_dev_open("uart0", 0, &uc);
    uart_rx_idx = 0;
}
#endif
/**
 * Initiates the GAP general discovery procedure.
 */
static int
blecent_scan(void)
{
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(BLE_ADDR_TYPE_PUBLIC, BLE_HS_FOREVER, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0) {
        BLECENT_LOG(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }

    return rc;
}

/**
 * Indicates whether we should tre to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and support for the Alert Notification service.
 */
static int
blecent_should_connect(const struct ble_gap_disc_desc *disc)
{
    int i;

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {

        return 0;
    }

    /* The device has to advertise support for the Alert Notification
     * service (0x1811).
     */
    for (i = 0; i < disc->fields->num_uuids16; i++) {
        if (disc->fields->uuids16[i] == BLECENT_SVC_ALERT_UUID) {
            return 1;
        }
    }

    return 0;
}

/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void
blecent_connect_if_interesting(const struct ble_gap_disc_desc *disc)
{
    int rc;
    int i;

    /* Don't do anything if we don't care about this advertiser. */
    if (!blecent_should_connect(disc)) {
        return;
    }

    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        BLECENT_LOG(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }

    /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
     * timeout.
     */

    for (i=0; i<MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {

        if (!memcmp(peer_white_list[i].addr, disc->addr, 6)) {
            ble_gap_connect(BLE_ADDR_TYPE_PUBLIC, BLE_HCI_CONN_PEER_ADDR_PUBLIC, peer_white_list[i].addr,
                            30000, &ble_gap_conn_params, blecent_gap_event, NULL);
        }
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  blecent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        /* An advertisment report was received during GAP discovery. */
        print_adv_fields(event->disc.fields);

        /* Try to connect to the advertiser if it looks interesting. */
        blecent_connect_if_interesting(&event->disc);
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0) {
            /* Connection successfully established. */
            BLECENT_LOG(INFO, "Connection established ");

            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            print_conn_desc(&desc);
            BLECENT_LOG(INFO, "\n");

            /* Remember peer. */
            rc = peer_add(event->connect.conn_handle);
            if (rc != 0) {
                BLECENT_LOG(ERROR, "Failed to add peer; rc=%d\n", rc);
                return 0;
            }
        } else {
            /* Connection attempt failed; resume scanning. */
            BLECENT_LOG(ERROR, "Error: Connection failed; status=%d\n",
                        event->connect.status);
        }

        rc = blecent_scan();
        assert(rc == 0);
        /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
         * timeout.
         */
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
        BLECENT_LOG(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        BLECENT_LOG(INFO, "\n");

        /* Forget about peer. */
        peer_delete(event->disconnect.conn.conn_handle);
        /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
         * timeout.
         */
        rc = blecent_scan();
        assert(rc == 0 || rc == BLE_HS_EBUSY || rc == BLE_HS_EALREADY);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        BLECENT_LOG(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        /* Peer sent us a notification or indication. */
        BLECENT_LOG(INFO, "received %s; conn_handle=%d attr_handle=%d "
                          "attr_len=%d\n",
                    event->notify_rx.indication ?
                        "indication" :
                        "notification",
                    event->notify_rx.conn_handle,
                    event->notify_rx.attr_handle,
                    OS_MBUF_PKTLEN(event->notify_rx.om));

        /* Attribute data is contained in event->notify_rx.attr_data. */
        return 0;

    case BLE_GAP_EVENT_MTU:
        BLECENT_LOG(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void
blecent_on_reset(int reason)
{
    BLECENT_LOG(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
blecent_on_sync(void)
{
    /* Begin scanning for a peripheral to connect to. */
    blecent_scan();
}

/**
 * Event loop for the main blecent task.
 */
static void
blecent_task_handler(void *unused)
{
    /* Set the led pin for the devboard */
    g_led_pin = LED_BLINK_PIN;

    hal_gpio_init_out(g_led_pin, 1);
#if defined(BSP_nrf51_blenano)||defined(BSP_nrf52dk)
    hal_gpio_write(g_led_pin, 0);
#else
    hal_gpio_write(g_led_pin, 1);
#endif

//    app_uart_init();

    while (1) {
        os_eventq_run(&blecent_evq);
    }
}

/**
 * main
 *
 * The main function for the project. This function initializes the os, calls
 * init_tasks to initialize tasks (and possibly other objects), then starts the
 * OS. We should not return from os start.
 *
 * @return int NOTE: this function should never return!
 */
int
main(void)
{
    int rc;

    /* Initialize OS */
    sysinit();

    /* Initialize the blecent log. */
    log_register("blecent", &blecent_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    /* Initialize the eventq for the application task. */
    os_eventq_init(&blecent_evq);

    /* Create the blecent task.  All application logic and NimBLE host
     * operations are performed in this task.
     */
    os_task_init(&blecent_task, "blecent", blecent_task_handler,
                 NULL, BLECENT_TASK_PRIO, OS_WAIT_FOREVER,
                 blecent_stack, BLECENT_STACK_SIZE);

    /* Configure the host. */
    ble_hs_cfg.reset_cb = blecent_on_reset;
    ble_hs_cfg.sync_cb = blecent_on_sync;
    ble_hs_cfg.store_read_cb = ble_store_ram_read;
    ble_hs_cfg.store_write_cb = ble_store_ram_write;

    /* Initialize data structures to track connected peers. */
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-blecent");
    assert(rc == 0);

    /* Set the default eventq for packages that lack a dedicated task. */
    os_eventq_dflt_set(&blecent_evq);

    /* Start the OS */
    os_start();

    /* os start should never return. If it does, this should be an error */
    assert(0);

    return 0;
}
