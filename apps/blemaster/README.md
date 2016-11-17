## Overview

This app is a blemaster app which connects to 32 peripherals
simultaneously. It is a copy of Apache Mynewt's blecent app with some
modifications.


#### Changing the name to blemaster to avoid confusion in functionality ####
diff -r apps/blemaster/pkg.yml /Users/vipul/myproj/repos/apache-mynewt-core/apps/blecent/pkg.yml
'
18c18
< pkg.name: apps/blemaster
---
> pkg.name: apps/blecent
25,35c25,35
'

#### Changing dependencies to point to Apache Mynewt core repo ####
'
< pkg.deps:
<     - "@apache-mynewt-core/kernel/os"
<     - "@apache-mynewt-core/sys/log"
<     - "@apache-mynewt-core/net/nimble/controller"
<     - "@apache-mynewt-core/net/nimble/host"
<     - "@apache-mynewt-core/net/nimble/host/services/gap"
<     - "@apache-mynewt-core/net/nimble/host/services/gatt"
<     - "@apache-mynewt-core/net/nimble/host/store/ram"
<     - "@apache-mynewt-core/net/nimble/transport/ram"
<     - "@apache-mynewt-core/sys/console/full"
<     - "@apache-mynewt-core/libc/baselibc"
---
> pkg.deps: 
>     - kernel/os 
>     - sys/log
>     - net/nimble/controller
>     - net/nimble/host
>     - net/nimble/host/services/gap
>     - net/nimble/host/services/gatt
>     - net/nimble/host/store/ram
>     - net/nimble/transport/ram
>     - sys/console/full
>     - libc/baselibc
'

diff -r apps/blemaster/src/main.c /Users/vipul/myproj/repos/apache-mynewt-core/apps/blecent/src/main.c
/* LED GPIO for indicating connection status */
'
1c1
< /**
---
> /*
27,28d26
< #include "hal/hal_gpio.h"
< 
50,52d47
< /* For LED toggling */
< static int g_led_pin;
'
#### Address list to allow connection to specific peers ####
'
69,126c64,66
< const struct ble_gap_white_entry peer_white_list[] =
<     {
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x00}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x01}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x02}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x03}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x04}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x05}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x06}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x07}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x08}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x09}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x10}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x11}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x12}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x13}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x14}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x15}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x16}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x17}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x18}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x19}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x20}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x21}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x22}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x23}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x24}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x25}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x26}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x27}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x28}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x29}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x30}},
<      {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x31}}
<     };
< 

#### Connection parameters ####
'
< static const struct ble_gap_conn_params ble_gap_conn_params = {
<     .scan_itvl = 0x0010,                                         /*  0x0010  */
<     .scan_window = 0x0010,                                       /*  0x0010  */
<     .itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN,                   /*  30ms    */
<     .itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX * 2,               /*  100ms   */
<     .latency = BLE_GAP_INITIAL_CONN_LATENCY,                     /*  0       */
<     .supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT,  /*  0x0100  */
<     .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,               /*  0x0010  */
<     .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,               /*  0x0300  */
< };
'


#### UART configuration and buffers and driver initialization for talking to Nextion displays, Also removing BLE specific APIs ####
'
< #if 0
< static char uart_rx_buf[15];
< static char uart_tx_buf[128];
< static int uart_rx_idx;
< static int uart_tx_idx;
< static int cmd_len;
< static struct uart_dev *udv;
< 
< /*
<  * Called by UART driver to send out next character.
<  *
<  * Interrupts disabled when nmgr_uart_tx_char/nmgr_uart_rx_char are called.
---
> /**
>  * Application callback.  Called when the read of the ANS Supported New Alert
>  * Category characteristic has completed.
129,134c69,78
< app_uart_tx_char(void *arg)
< {
<     if (uart_tx_idx < cmd_len) {
<         return uart_tx_buf[uart_tx_idx++];
<     } else {
<         return -1;
---
> blecent_on_read(uint16_t conn_handle,
>                 const struct ble_gatt_error *error,
>                 struct ble_gatt_attr *attr,
>                 void *arg)
> {
>     BLECENT_LOG(INFO, "Read complete; status=%d conn_handle=%d", error->status,
>                 conn_handle);
>     if (error->status == 0) {
>         BLECENT_LOG(INFO, " attr_handle=%d value=", attr->handle);
>         print_mbuf(attr->om);
135a80,82
>     BLECENT_LOG(INFO, "\n");
> 
>     return 0;
138,139c85,87
< /*
<  * Receive a character from UART.
---
> /**
>  * Application callback.  Called when the write to the ANS Alert Notification
>  * Control Point characteristic has completed.
142c90,132
< app_uart_rx_char(void *arg, uint8_t data)
---
> blecent_on_write(uint16_t conn_handle,
>                  const struct ble_gatt_error *error,
>                  struct ble_gatt_attr *attr,
>                  void *arg)
> {
>     BLECENT_LOG(INFO, "Write complete; status=%d conn_handle=%d "
>                       "attr_handle=%d\n",
>                 error->status, conn_handle, attr->handle);
> 
>     return 0;
> }
> 
> /**
>  * Application callback.  Called when the attempt to subscribe to notifications
>  * for the ANS Unread Alert Status characteristic has completed.
>  */
> static int
> blecent_on_subscribe(uint16_t conn_handle,
>                      const struct ble_gatt_error *error,
>                      struct ble_gatt_attr *attr,
>                      void *arg)
> {
>     BLECENT_LOG(INFO, "Subscribe complete; status=%d conn_handle=%d "
>                       "attr_handle=%d\n",
>                 error->status, conn_handle, attr->handle);
> 
>     return 0;
> }
> 
> /**
>  * Performs three concurrent GATT operations against the specified peer:
>  * 1. Reads the ANS Supported New Alert Category characteristic.
>  * 2. Writes the ANS Alert Notification Control Point characteristic.
>  * 3. Subscribes to notifications for the ANS Unread Alert Status
>  *    characteristic.
>  *
>  * If the peer does not support a required service, characteristic, or
>  * descriptor, then the peer lied when it claimed support for the alert
>  * notification service!  When this happens, or if a GATT procedure fails,
>  * this function immediately terminates the connection.
>  */
> static void
> blecent_read_write_subscribe(const struct peer *peer)
143a134,136
>     const struct peer_chr *chr;
>     const struct peer_dsc *dsc;
>     uint8_t value[2];
145d137
<     int cmd;
147,156c139,154
<     uart_rx_buf[uart_rx_idx++] = data;
<     if (uart_rx_idx >= 7) {
<         cmd = uart_rx_buf[2];
<         if (cmd == 2) {
<             os_eventq_put(&blecent_evq, &ble_scan_ev);
<         } else if (cmd == 3) {
<             os_eventq_put(&blecent_evq, &ble_tx_ev);
<         }
<         uart_rx_idx = 0;
<         //memset(uart_rx_buf, 0, 7);
---
>     /* Read the supported-new-alert-category characteristic. */
>     chr = peer_chr_find_uuid(peer,
>                              BLE_UUID16(BLECENT_SVC_ALERT_UUID),
>                              BLE_UUID16(BLECENT_CHR_SUP_NEW_ALERT_CAT_UUID));
>     if (chr == NULL) {
>         BLECENT_LOG(ERROR, "Error: Peer doesn't support the Supported New "
>                            "Alert Category characteristic\n");
>         goto err;
>     }
> 
>     rc = ble_gattc_read(peer->conn_handle, chr->chr.val_handle,
>                         blecent_on_read, NULL);
>     if (rc != 0) {
>         BLECENT_LOG(ERROR, "Error: Failed to read characteristic; rc=%d\n",
>                     rc);
>         goto err;
159,160c157,206
<     rc = 0;
<     return rc;
---
>     /* Write two bytes (99, 100) to the alert-notification-control-point
>      * characteristic.
>      */
>     chr = peer_chr_find_uuid(peer,
>                              BLE_UUID16(BLECENT_SVC_ALERT_UUID),
>                              BLE_UUID16(BLECENT_CHR_ALERT_NOT_CTRL_PT));
>     if (chr == NULL) {
>         BLECENT_LOG(ERROR, "Error: Peer doesn't support the Alert "
>                            "Notification Control Point characteristic\n");
>         goto err;
>     }
> 
>     value[0] = 99;
>     value[1] = 100;
>     rc = ble_gattc_write_flat(peer->conn_handle, chr->chr.val_handle,
>                               value, sizeof value, blecent_on_write, NULL);
>     if (rc != 0) {
>         BLECENT_LOG(ERROR, "Error: Failed to write characteristic; rc=%d\n",
>                     rc);
>     }
> 
>     /* Subscribe to notifications for the Unread Alert Status characteristic.
>      * A central enables notifications by writing two bytes (1, 0) to the
>      * characteristic's client-characteristic-configuration-descriptor (CCCD).
>      */
>     dsc = peer_dsc_find_uuid(peer,
>                              BLE_UUID16(BLECENT_SVC_ALERT_UUID),
>                              BLE_UUID16(BLECENT_CHR_UNR_ALERT_STAT_UUID),
>                              BLE_UUID16(BLE_GATT_DSC_CLT_CFG_UUID16));
>     if (dsc == NULL) {
>         BLECENT_LOG(ERROR, "Error: Peer lacks a CCCD for the Unread Alert "
>                            "Status characteristic\n");
>         goto err;
>     }
> 
>     value[0] = 1;
>     value[1] = 0;
>     rc = ble_gattc_write_flat(peer->conn_handle, dsc->dsc.handle,
>                               value, sizeof value, blecent_on_subscribe, NULL);
>     if (rc != 0) {
>         BLECENT_LOG(ERROR, "Error: Failed to subscribe to characteristic; "
>                            "rc=%d\n", rc);
>         goto err;
>     }
> 
>     return;
> 
> err:
>     /* Terminate the connection. */
>     ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
162a209,211
> /**
>  * Called when service discovery of the specified peer has completed.
>  */
164c213
< app_uart_init(void)
---
> blecent_on_disc_complete(const struct peer *peer, int status, void *arg)
166,175d214
<     struct uart_conf uc = {
<         .uc_speed = 9600,
<         .uc_databits = 8,
<         .uc_stopbits = 1,
<         .uc_parity = UART_PARITY_NONE,
<         .uc_flow_ctl = UART_FLOW_CTL_NONE,
<         .uc_tx_char = app_uart_tx_char,
<         .uc_rx_char = app_uart_rx_char,
<         .uc_cb_arg = NULL
<     };
177,178c216,234
<     udv = (struct uart_dev *)os_dev_open("uart0", 0, &uc);
<     uart_rx_idx = 0;
---
>     if (status != 0) {
>         /* Service discovery failed.  Terminate the connection. */
>         BLECENT_LOG(ERROR, "Error: Service discovery failed; status=%d "
>                            "conn_handle=%d\n", status, peer->conn_handle);
>         ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
>         return;
>     }
> 
>     /* Service discovery has completed successfully.  Now we have a complete
>      * list of services, characteristics, and descriptors that the peer
>      * supports.
>      */
>     BLECENT_LOG(ERROR, "Service discovery complete; status=%d "
>                        "conn_handle=%d\n", status, peer->conn_handle);
> 
>     /* Now perform three concurrent GATT procedures against the peer: read,
>      * write, and subscribe to notifications.
>      */
>     blecent_read_write_subscribe(peer);
180c236
< #endif
---

'

#### Adding return code to look for errrors ####
'
> 
184c240
< static int
---
> static void
213,214d268
< 
<     return rc;
'

#### Using connection parameters for connecting ####
'
255d308
<     int i;
272,278c325,330
< 
<     for (i=0; i<MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
< 
<         if (!memcmp(peer_white_list[i].addr, disc->addr, 6)) {
<             ble_gap_connect(BLE_ADDR_TYPE_PUBLIC, BLE_HCI_CONN_PEER_ADDR_PUBLIC, peer_white_list[i].addr,
<                             30000, &ble_gap_conn_params, blecent_gap_event, NULL);
<         }
---
>     rc = ble_gap_connect(BLE_ADDR_TYPE_PUBLIC, disc->addr_type, disc->addr,
>                          30000, NULL, blecent_gap_event, NULL);
>     if (rc != 0) {
>         BLECENT_LOG(ERROR, "Error: Failed to connect to device; addr_type=%d "
>                            "addr=%s\n", disc->addr_type, addr_str(disc->addr));
>         return;
327a380,387
> 
>             /* Perform service discovery. */
>             rc = peer_disc_all(event->connect.conn_handle,
>                                blecent_on_disc_complete, NULL);
>             if (rc != 0) {
>                 BLECENT_LOG(ERROR, "Failed to discover services; rc=%d\n", rc);
>                 return 0;
>             }
'
#### BLE Scan can fail if it is already scanning, hence we do not want to look at the return code here ####
'
331a392
>             blecent_scan();
334,338d394
<         rc = blecent_scan();
<         assert(rc == 0);
<         /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
<          * timeout.
<          */
349,353c405,407
<         /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
<          * timeout.
<          */
<         rc = blecent_scan();
<         assert(rc == 0 || rc == BLE_HS_EBUSY || rc == BLE_HS_EALREADY);
---
> 
>         /* Resume scanning. */
>         blecent_scan();
410,425d463
'

#### Using configurable LED pin if someone wants to do a custom LED hookup ####
'
<     /* Set the led pin for the devboard */
< #ifdef MYNEWT_VAL(LEDPIN)
<     g_led_pin = MYNEWT_VAL(LEDPIN);
< #else
<     g_led_pin = LED_BLINK_PIN;
< #endif
< 
<     hal_gpio_init_out(g_led_pin, 1);
< #if defined(BSP_nrf51_blenano)||defined(BSP_nrf52dk)
<     hal_gpio_write(g_led_pin, 0);
< #else
<     hal_gpio_write(g_led_pin, 1);
< #endif
< 
< //    app_uart_init();
<
'
#### Fixing indentation ####
449c487,488
<     log_register("blecent", &blecent_log, &log_console_handler, NULL, LOG_SYSLEVEL);
---
>     log_register("blecent", &blecent_log, &log_console_handler, NULL,
>                  LOG_SYSLEVEL);
461a501,502
>     log_register("ble_hs", &ble_hs_log, &log_console_handler, NULL,
>                  LOG_SYSLEVEL);
'

'

diff -r apps/blemaster/src/misc.c /Users/vipul/myproj/repos/apache-mynewt-core/apps/blecent/src/misc.c
#### Some cleanup ####

'
1c1
< /**
---
> /*
diff -r apps/blemaster/src/peer.c /Users/vipul/myproj/repos/apache-mynewt-core/apps/blecent/src/peer.c
1c1
< /**
---
> /*
'

#### Adding syscfg to allow for more number fo connections and reducing the log level ####

'
diff -r apps/blemaster/syscfg.yml /Users/vipul/myproj/repos/apache-mynewt-core/apps/blecent/syscfg.yml
1c1,19
< # Package: apps/blemaster
---
> # Licensed to the Apache Software Foundation (ASF) under one
> # or more contributor license agreements.  See the NOTICE file
> # distributed with this work for additional information
> # regarding copyright ownership.  The ASF licenses this file
> # to you under the Apache License, Version 2.0 (the
> # "License"); you may not use this file except in compliance
> # with the License.  You may obtain a copy of the License at
> #
> #  http://www.apache.org/licenses/LICENSE-2.0
> #
> # Unless required by applicable law or agreed to in writing,
> # software distributed under the License is distributed on an
> # "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
> # KIND, either express or implied.  See the License for the
> # specific language governing permissions and limitations
> # under the License.
> #
> 
> # Package: apps/blecent
5,7c23
<     LOG_LEVEL: 0
<     BLE_MAX_CONNECTIONS: 32
<     BLE_LL_WHITELIST_SIZE: 8
---
>     LOG_LEVEL: 1
'




