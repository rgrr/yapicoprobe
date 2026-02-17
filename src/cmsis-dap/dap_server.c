/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <pico/stdlib.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "tusb.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "picoprobe_config.h"
#include "dap_server.h"
#include "dap_util.h"
#include "DAP_config.h"
#include "DAP.h"
#include "led.h"
#include "sw_lock.h"
#include "minIni/minIni.h"

/*
 * The following is part of a hack to make DAP_PACKET_COUNT a variable.
 * CMSIS-DAPv2 has better performance with 2 packets while
 * CMSIS-DAPv1 only works with one packet, at least with openocd which throws a
 *     "CMSIS-DAP transfer count mismatch: expected 12, got 8" on flashing.
 * The correct packet count has to be set on connection.
 *
 * More notes: pyocd works with large packets only, if the packet count is one.
 * Additionally pyocd is instable if packet count > 1.  Valid for pyocd 0.34.3.
 *
 * OpenOCD 0.11: packet size of 1024 and 2 buffers ok
 * OpenOCD 0.12: 1024 no longer working, but 512 and 2 buffers is ok
 *
 * 2024-10-13 - confusing... openocd and so on work only with 1 packet
 *
 * 2025-12-11            PyOCD 0.41.0      OpenOCD 0.12.0+dev-gebec950-dirty
 *           64/1             -/-                     +/+                       (flash/debug)
 *          128/1             +/+                     +/+
 *          256/1             -/+                     +/+
 *          512/1             -/+                     +/+
 *
 *           64/2             -/-                     +/+
 *          128/2             +/+                     -/+
 *          256/2             X/+                     -/+
 *
 *           64/4              -                       -
 *          128/4             +/+                     -/+
 *          128/8             +/+                     -/+
 *          256/8             X/+                     -/+
 *
 *     - openocd
 *       - uses full buffer
 *       - question is if openocd is using more than one buffer for debugging
 *     - pyocd
 *       - only 334 bytes on transmit
 *       - doing read_ap_multi() emits maximum of 56 requests (so that the request fits into 64 bytes), response has only 227 bytes
 *       - pyocd transmits obscure zero byte during certain flash phases if buffer >= 256, see https://github.com/pyocd/pyOCD/issues/1871
 *       - I doubt that pyocd benefits from DAP_PACKET_COUNT > 2, even ==2 is questionable
 *       - does not use multi buffers for debugging
 *
 * 2026-02-17  Problem areas
 *     - pyocd
 *       - does no disconnect on end of debug sessions (small piece of code is missing, see
 *         https://github.com/pyocd/pyOCD/issues/1531 and
 *         https://github.com/pyocd/pyOCD/pull/1869 (still open)
 *       - this means that the tool detection does not work in this case.  After introducing a timeout in
 *         finger printing, only the connect message is missing
 *       - BUT does not access the SW if debugging is halted and seems to restore DP state
 *       - "pyocd list" does only DAP_Info, no connect/disconnect -> not simple to recover from this tool detection
 *     - openocd
 *       - only small pauses if debugging is halted
 *       - DP is not completely restore
 *     - probe-rs
 *       - very slow
 *       - no good integration into eclipse
 */

// new implementation has been stolen from the original picoprobe
#define NEW_DAP

#ifndef NEW_DAP
    #define _DAP_PACKET_COUNT_OPENOCD   1
    #define _DAP_PACKET_SIZE_OPENOCD    1024
    #define _DAP_PACKET_COUNT_PROBERS   2
    #define _DAP_PACKET_SIZE_PROBERS    1024
    #define _DAP_PACKET_COUNT_PYOCD     2
    #define _DAP_PACKET_SIZE_PYOCD      1024
    #define _DAP_PACKET_COUNT_UNKNOWN   1
    #define _DAP_PACKET_SIZE_UNKNOWN    64

    #define _DAP_PACKET_COUNT_HID       1
    #define _DAP_PACKET_SIZE_HID        64
#else
    //
    // Data transfer:
    // - pyocd:    13 used buffers seen, but values >= 8 bring actually no performance gain
    // - openocd:  4 used buffers seen
    // - probe-rs: does not use multi buffers at all!?
    //
    // Debugging sessions do not use multi buffers at all
    //
    // !!!! pyocd does not work with _DAP_PACKET_SIZE_NEW!=64 !!!!
    //
    // _DAP_PACKET_SIZE_NEW does not seem to be the decisive parameter, neither _DAP_PACKET_COUNT_NEW.
    // So below are good values
    //
    #define _DAP_PACKET_COUNT_NEW       16
    #define _DAP_PACKET_SIZE_NEW        64

//    #define DAP_DEBUG                   1

    #if (_DAP_PACKET_COUNT_NEW & (_DAP_PACKET_COUNT_NEW - 1)) != 0
        // no more restrictions here
//        #error "_DAP_PACKET_COUNT_NEW must be a power of 2"
    #endif
    #if (_DAP_PACKET_SIZE_NEW & (_DAP_PACKET_SIZE_NEW - 1)) != 0
        #warning "_DAP_PACKET_SIZE_NEW should be a power of 2"
    #endif
#endif

#ifndef NEW_DAP
    static TaskHandle_t           dap_taskhandle = NULL;
    static EventGroupHandle_t     dap_events;
    static StreamBufferHandle_t   dap_stream;
    static TimerHandle_t          timer_clear_dap_tool;

    uint8_t  dap_packet_count = _DAP_PACKET_COUNT_UNKNOWN;
    uint16_t dap_packet_size  = _DAP_PACKET_SIZE_UNKNOWN;
#else
    uint8_t  dap_packet_count = _DAP_PACKET_COUNT_NEW;
    uint16_t dap_packet_size  = _DAP_PACKET_SIZE_NEW;
#endif

#if defined(NEW_DAP)  &&  OPT_CMSIS_DAPV1
    #define _DAP_PACKET_COUNT_HID       1
    #define _DAP_PACKET_SIZE_HID        64
#endif

#ifndef NEW_DAP
    #define BUFFER_MAXSIZE_1 MAX(_DAP_PACKET_COUNT_OPENOCD*_DAP_PACKET_SIZE_OPENOCD, _DAP_PACKET_COUNT_PROBERS*_DAP_PACKET_SIZE_PROBERS)
    #define BUFFER_MAXSIZE_2 MAX(_DAP_PACKET_COUNT_PYOCD  *_DAP_PACKET_SIZE_PYOCD,   _DAP_PACKET_COUNT_UNKNOWN*_DAP_PACKET_SIZE_UNKNOWN)
    #define BUFFER_MAXSIZE   MAX(BUFFER_MAXSIZE_1, BUFFER_MAXSIZE_2)

    #define PACKET_MAXSIZE_1 MAX(_DAP_PACKET_SIZE_OPENOCD, _DAP_PACKET_SIZE_PROBERS)
    #define PACKET_MAXSIZE_2 MAX(_DAP_PACKET_SIZE_PYOCD,   _DAP_PACKET_SIZE_UNKNOWN)
    #define PACKET_MAXSIZE   MAX(PACKET_MAXSIZE_1, PACKET_MAXSIZE_2)

    #if (CFG_TUD_VENDOR_RX_BUFSIZE < PACKET_MAXSIZE)
        #error "increase CFG_TUD_VENDOR_RX_BUFSIZE"
    #endif

    #if OPT_CMSIS_DAPV1  ||  OPT_CMSIS_DAPV2
        static uint8_t TxDataBuffer[PACKET_MAXSIZE];
    #endif
    #if OPT_CMSIS_DAPV2
        static uint8_t RxDataBuffer[PACKET_MAXSIZE];

        static bool swd_connected = false;
        static uint32_t request_len;
        static uint32_t last_request_us = 0;
        static uint32_t rx_len = 0;
        static daptool_t dap_tool = E_DAPTOOL_UNKNOWN;
    #endif
#endif


#ifdef NEW_DAP
    #define DAP_INTERFACE_SUBCLASS 0x00
    #define DAP_INTERFACE_PROTOCOL 0x00

    #define MOD_PACKET_COUNT(x)     (((x) + _DAP_PACKET_COUNT_NEW) % _DAP_PACKET_COUNT_NEW)

    #define WR_SLOT_PTR(x)      (&(x.data[x.wr_idx][0]))
    #define RD_SLOT_PTR(x)      (&(x.data[x.rd_idx][0]))
    #define WR_SLOT_LEN(x)      (x.data_len[x.wr_idx])
    #define RD_SLOT_LEN(x)      (x.data_len[x.rd_idx])
    #define WR_NEXT_SLOT_LEN(x) (x.data_len[MOD_PACKET_COUNT(x.wr_idx + 1)])
    typedef struct {
        uint8_t  data[_DAP_PACKET_COUNT_NEW][_DAP_PACKET_SIZE_NEW];
        uint16_t data_len[_DAP_PACKET_COUNT_NEW];
        volatile uint32_t wr_idx;
        volatile uint32_t rd_idx;
        volatile bool     rcvStartDelayed;
        volatile bool     xmtRunning;
#if DAP_DEBUG
        uint16_t dmax;
#endif
    } queue_t;

    static queue_t requestQueue;
    static queue_t responseQueue;

    static uint8_t itf_num;             // actually not used
    static uint8_t rhport;

    static uint8_t out_ep_addr;
    static uint8_t in_ep_addr;

    static TaskHandle_t      dap_taskhandle = NULL;
    static SemaphoreHandle_t edpt_spoon;

    static bool swd_connected;
    static daptool_t dap_tool = E_DAPTOOL_UNKNOWN;

#if DAP_DEBUG
    static const char * dap_cmd_string[] = {
        [ID_DAP_Info               ] = "DAP_Info",
        [ID_DAP_HostStatus         ] = "DAP_HostStatus",
        [ID_DAP_Connect            ] = "DAP_Connect",
        [ID_DAP_Disconnect         ] = "DAP_Disconnect",
        [ID_DAP_TransferConfigure  ] = "DAP_TransferConfigure",
        [ID_DAP_Transfer           ] = "DAP_Transfer",
        [ID_DAP_TransferBlock      ] = "DAP_TransferBlock",
        [ID_DAP_TransferAbort      ] = "DAP_TransferAbort",
        [ID_DAP_WriteABORT         ] = "DAP_WriteABORT",
        [ID_DAP_Delay              ] = "DAP_Delay",
        [ID_DAP_ResetTarget        ] = "DAP_ResetTarget",
        [ID_DAP_SWJ_Pins           ] = "DAP_SWJ_Pins",
        [ID_DAP_SWJ_Clock          ] = "DAP_SWJ_Clock",
        [ID_DAP_SWJ_Sequence       ] = "DAP_SWJ_Sequence",
        [ID_DAP_SWD_Configure      ] = "DAP_SWD_Configure",
        [ID_DAP_SWD_Sequence       ] = "DAP_SWD_Sequence",
        [ID_DAP_JTAG_Sequence      ] = "DAP_JTAG_Sequence",
        [ID_DAP_JTAG_Configure     ] = "DAP_JTAG_Configure",
        [ID_DAP_JTAG_IDCODE        ] = "DAP_JTAG_IDCODE",
        [ID_DAP_SWO_Transport      ] = "DAP_SWO_Transport",
        [ID_DAP_SWO_Mode           ] = "DAP_SWO_Mode",
        [ID_DAP_SWO_Baudrate       ] = "DAP_SWO_Baudrate",
        [ID_DAP_SWO_Control        ] = "DAP_SWO_Control",
        [ID_DAP_SWO_Status         ] = "DAP_SWO_Status",
        [ID_DAP_SWO_ExtendedStatus ] = "DAP_SWO_ExtendedStatus",
        [ID_DAP_SWO_Data           ] = "DAP_SWO_Data",
        [ID_DAP_QueueCommands      ] = "DAP_QueueCommands",
        [ID_DAP_ExecuteCommands    ] = "DAP_ExecuteCommands",
    };
#endif

#endif

#if defined(NEW_DAP)  &&  OPT_CMSIS_DAPV1
    static uint8_t TxDataBuffer[_DAP_PACKET_SIZE_HID];
#endif



#if OPT_CMSIS_DAPV2  &&  !defined(NEW_DAP)

#if TUSB_VERSION_NUMBER <= 2000  // 0.20.0
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize)
#else
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint32_t bufsize)
#endif
/**
 * Put all the received data into a stream.  Actual execution is done in dap_task()
 */
{
    bool send_event = false;

//    picoprobe_info("rx: %d, %p, %d\n", itf, buffer, (int)bufsize);

    if (itf != 0) {
        return;
    }

    while (tud_vendor_available()) {
        uint8_t  buf[64];
        uint32_t n = tud_vendor_read(buf, sizeof(buf));

        if (n == 1  &&  buf[0] == 0  &&  dap_tool == E_DAPTOOL_PYOCD) {
            // this is a special pyocd (<= 0.42.0) hack (and of course openocd does not like it)
            // see https://github.com/pyocd/pyOCD/issues/1871
            picoprobe_info("-----------pyocd hack\n");
        }
        else {
            xStreamBufferSend(dap_stream, buf, n, 0);
            send_event = true;
        }
    }

    if (send_event) {
        xEventGroupSetBits(dap_events, 0x01);
    }
}   // tud_vendor_rx_cb
#endif



#if OPT_CMSIS_DAPV2  &&  !defined(NEW_DAP)
void dap_task(void *ptr)
/**
 * CMSIS-DAP task.
 * Receive DAP requests, execute them via DAP_ExecuteCommand() and transmit the response.
 *
 * Problem zones:
 * - connect / disconnect: pyOCD does not send permanently requests if in gdbserver mode, OpenOCD does.
 *   As a consequence "disconnect" has to be detected via the command stream.  If the tool on host side
 *   fails without a disconnect, the SWD connection is not freed (for MSC or RTT).  To recover from this
 *   situation either reset the probe or issue something like "pyocd reset -t rp2040"
 * - ID_DAP_Disconnect / ID_DAP_Info / ID_DAP_HostStatus leads to an SWD disconnect if there is no other
 *   command following.  This is required, because "pyocd list" leads to tool detection without
 *   connect/disconnect and thus otherwise tool detection would be stuck to "pyocd" for the next connection.
 * - trying to multiplex between CMSIS-DAP and RTT
 *   - if there are no DAP commands for a certain time, then dap_task() gives SWD control to RTT
 *   - pyocd sends every 11ms a command while target is executing, so it is waited if there is a 8ms gap
 *     in command stream
 */
{
    dap_packet_count = _DAP_PACKET_COUNT_UNKNOWN;
    dap_packet_size  = _DAP_PACKET_SIZE_UNKNOWN;

    for (;;) {
        EventBits_t ev = xEventGroupWaitBits(dap_events, 0x01, pdTRUE, pdFALSE,
                                             swd_connected ? pdMS_TO_TICKS(8) : pdMS_TO_TICKS(1000));

        size_t n = xStreamBufferBytesAvailable(dap_stream);
        n = MIN(n, sizeof(RxDataBuffer) - rx_len);
        xStreamBufferReceive(dap_stream, RxDataBuffer + rx_len, n, 0);
        rx_len += n;
        // post: data fetched from stream

#if OPT_RTT_WHILE_DEBUGGING  // EXPERIMENTAL FEATURE
        if (    rx_len == 0  &&  ev == 0
            &&  dap_tool == E_DAPTOOL_PYOCD
           ) {   // do not do it for probe-rs
            if (swd_connected) {
                //
                // try to receive RTT data while debugging if there is no DAP command pending
                // read more in rtt_io_thread()
                //
                sw_unlock(E_SWLOCK_DAPV2);

                do {
                    ev = xEventGroupWaitBits(dap_events, 0x01, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
                    n = xStreamBufferBytesAvailable(dap_stream);
                } while (n == 0  &&  ev == 0);

                sw_lock(E_SWLOCK_DAPV2);
            }
            continue;
        }
#endif

        for (;;) {
            if (rx_len == 0) {
                break;
            }

            request_len = DAP_GetCommandLength(RxDataBuffer, rx_len);
            if (rx_len < request_len) {
//                picoprobe_info("......... %d > %d\n", request_len, rx_len);
                break;
            }

            if (rx_len != request_len) {
                // actually this means that there are more bytes received than expected (handle as multi commands?)
                picoprobe_info("!!!!!!!!! %d != %d\n", (int)request_len, (int)rx_len);
            }

            //
            // now we have at least one request in the buffer
            //
            last_request_us = time_us_32();
//            picoprobe_info("<<<(%ld,%ld) %d %d\n", request_len, rx_len, RxDataBuffer[0], RxDataBuffer[1]);

            if (dap_tool == E_DAPTOOL_UNKNOWN) {
                //
                // try to find out which tool is connecting
                //
                uint32_t psize;
                uint32_t pcnt;

                psize = ini_getl(MININI_SECTION, MININI_VAR_DAP_PSIZE, 0, MININI_FILENAME);
                pcnt  = ini_getl(MININI_SECTION, MININI_VAR_DAP_PCNT,  0, MININI_FILENAME);
                if (psize != 0  ||  pcnt != 0)
                {
                    dap_packet_count = (pcnt  != 0) ? pcnt  : _DAP_PACKET_COUNT_UNKNOWN;
                    dap_packet_size  = (psize != 0) ? psize : _DAP_PACKET_SIZE_UNKNOWN;
                    dap_packet_size  = MIN(dap_packet_size, PACKET_MAXSIZE);
                    if (dap_packet_count * dap_packet_size > BUFFER_MAXSIZE) {
                        dap_packet_size  = MIN(dap_packet_size, BUFFER_MAXSIZE);
                        dap_packet_count = BUFFER_MAXSIZE / dap_packet_size;
                    }
                    dap_tool = E_DAPTOOL_USER;
                }
                else
                {
                    dap_tool = DAP_FingerprintTool(RxDataBuffer, request_len);
                    if (dap_tool == E_DAPTOOL_OPENOCD) {
                        dap_packet_count = _DAP_PACKET_COUNT_OPENOCD;
                        dap_packet_size  = _DAP_PACKET_SIZE_OPENOCD;
                    }
                    else if (dap_tool == E_DAPTOOL_PYOCD) {
                        dap_packet_count = _DAP_PACKET_COUNT_PYOCD;
                        dap_packet_size  = _DAP_PACKET_SIZE_PYOCD;
                    }
                    else if (dap_tool == E_DAPTOOL_PROBERS) {
                        dap_packet_count = _DAP_PACKET_COUNT_PROBERS;
                        dap_packet_size  = _DAP_PACKET_SIZE_PROBERS;
                    }
                    else {
                        dap_packet_count = _DAP_PACKET_COUNT_UNKNOWN;
                        dap_packet_size  = _DAP_PACKET_SIZE_UNKNOWN;
                    }
                }

                // clear dap_tool after some time, required for "pyocd list" which neither does connect nor disconnect
                xTimerReset(timer_clear_dap_tool, 100);
            }

            //
            // initiate SWD connect / disconnect
            //
            if ( !swd_connected) {
                if ( !DAP_OfflineCommand(RxDataBuffer)) {
                    xTimerStop(timer_clear_dap_tool, 100);
                    if (sw_lock(E_SWLOCK_DAPV2)) {
                        swd_connected = true;
                        picoprobe_info("=================================== DAPv2 connect target, host %s, buffer: %dx%dbytes\n",
                                (dap_tool == E_DAPTOOL_OPENOCD) ? "OpenOCD"    :
                                    (dap_tool == E_DAPTOOL_PYOCD) ? "pyOCD"       :
                                        (dap_tool == E_DAPTOOL_PROBERS) ? "probe-rs" :
                                            (dap_tool == E_DAPTOOL_USER) ? "user-set"   : "UNKNOWN", dap_packet_count, dap_packet_size);
                        picoprobe_debug("------------ %d (command leading to online)\n", RxDataBuffer[0]);
                        led_state(LS_DAPV2_CONNECTED);
                    }
                    else {
                        // TODO very obscure... did not get lock!
                        // happens with "probe-rs info", see https://github.com/rgrr/yapicoprobe/issues/162
                        // if we do not execute the command here, then the whole probe stucks (forever)
                        uint32_t resp_len;

                        picoprobe_debug("------------ %d (command leading to this)\n", RxDataBuffer[0]);
                        picoprobe_error("sw_lock(%d): did not get lock\n", E_SWLOCK_DAPV2);
                        resp_len = DAP_ExecuteCommand(RxDataBuffer, TxDataBuffer);
                        tud_vendor_write(TxDataBuffer, resp_len & 0xffff);
                        tud_vendor_flush();
                    }
                }
                else {
                    // ID_DAP_Info must/can be done without a lock
                    picoprobe_debug("-----------__ %d (execute offline)\n", RxDataBuffer[0]);
                }
            }
            else {
                // connected:
                if (RxDataBuffer[0] == ID_DAP_Disconnect) {
                    swd_connected = false;
                    picoprobe_info("=================================== DAPv2 disconnect target\n");
                    led_state(LS_DAPV2_DISCONNECTED);
                    sw_unlock(E_SWLOCK_DAPV2);

                    // after disconnect wait some time before clearing dap_tool, required for probe-rs which does short disconnect/connect sequences
                    xTimerReset(timer_clear_dap_tool, 100);
                }
            }

            //
            // execute request and send back response
            //
            if (swd_connected  ||  DAP_OfflineCommand(RxDataBuffer))
            {
                uint32_t resp_len;

#if 0
                // heavy debug output, set dap_packet_count=2 to stumble into the bug
                const uint16_t bufsize = 64;
                picoprobe_info("-----------------------------------------------\n");
                picoprobe_info("<< (%lx) ", request_len);
                for (int i = 0;  i < bufsize;  ++i) {
                    picoprobe_info_out(" %02x", RxDataBuffer[i]);
                    if (i == request_len - 1) {
                        picoprobe_info_out(" !!!!");
                    }
                }
                picoprobe_info_out("\n");
                vTaskDelay(pdMS_TO_TICKS(5));
                resp_len = DAP_ExecuteCommand(RxDataBuffer, TxDataBuffer);
                picoprobe_info(">> (%lx) ", resp_len);
                for (int i = 0;  i < bufsize;  ++i) {
                    picoprobe_info_out(" %02x", TxDataBuffer[i]);
                    if (i == (resp_len & 0xffff) - 1) {
                        picoprobe_info_out(" !!!!");
                    }
                }
                picoprobe_info_out("\n");
#else
                resp_len = DAP_ExecuteCommand(RxDataBuffer, TxDataBuffer);
#endif

                //        picoprobe_info(">>>(%d) %d %d %d %d %d\n", request_len, resp_len & 0xffff, TxDataBuffer[0], TxDataBuffer[1], TxDataBuffer[2], TxDataBuffer[3]);

                tud_vendor_write(TxDataBuffer, resp_len & 0xffff);
                tud_vendor_flush();

                if (request_len != (resp_len >> 16))
                {
                    // there is a bug in CMSIS-DAP, see https://github.com/ARM-software/CMSIS_5/pull/1503
                    // but we trust our own length calculation
                    picoprobe_error("   !!!!!!!! request (%u) and executed length (%u) differ\n",
                                    (unsigned)request_len, (unsigned)(resp_len >> 16));
                }
            }

            if (rx_len == request_len)
            {
                rx_len = 0;
            }
            else
            {
                memmove(RxDataBuffer, RxDataBuffer + request_len, rx_len - request_len);
                rx_len -= request_len;
            }
        }
    }
}   // dap_task
#endif



#if OPT_CMSIS_DAPV2
extern uint8_t const desc_ms_os_20[];

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
/**
 * Control handshake during USB setup
 */
{
    // nothing to do with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    switch (request->bmRequestType_bit.type) {
        case TUSB_REQ_TYPE_VENDOR:
            switch (request->bRequest) {
                case 1:
                    if (request->wIndex == 7) {
                        // Get Microsoft OS 2.0 compatible descriptor
                        uint16_t total_len;
                        memcpy(&total_len, desc_ms_os_20 + 8, 2);

                        return tud_control_xfer(rhport, request, (void*) desc_ms_os_20, total_len);
                    }
                    else {
                        return false;
                    }

                default:
                    break;
            }
            break;

        default:
            break;
    }

    // stall unknown request
    return false;
}   // tud_vendor_control_xfer_cb
#endif



#if OPT_CMSIS_DAPV1
static bool hid_swd_connected;
static bool hid_swd_disconnect_requested;
static TimerHandle_t     timer_hid_disconnect = NULL;
static void             *timer_hid_disconnect_id;


static void hid_disconnect(TimerHandle_t xTimer)
{
    if (hid_swd_disconnect_requested  &&  hid_swd_connected) {
        hid_swd_connected = false;
        picoprobe_info("=================================== DAPv1 disconnect target\n");
        led_state(LS_DAPV1_DISCONNECTED);
        sw_unlock(E_SWLOCK_DAPV1);
    }
}   // hid_disconnect



uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    // TODO not Implemented
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}



void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* RxDataBuffer, uint16_t bufsize)
{
    // This doesn't use multiple report and report ID
    (void) itf;
    (void) report_id;
    (void) report_type;

    if (timer_hid_disconnect == NULL) {
        timer_hid_disconnect = xTimerCreate("timer_hid_disconnect", pdMS_TO_TICKS(1000), pdFALSE, timer_hid_disconnect_id,
                                            hid_disconnect);
        if (timer_hid_disconnect == NULL) {
            picoprobe_error("tud_hid_set_report_cb: cannot create timer_hid_disconnect\n");
        }
    }
    else {
        xTimerReset(timer_hid_disconnect, pdMS_TO_TICKS(1000));
    }

    //
    // initiate SWD connect / disconnect
    //
    if ( !hid_swd_connected  &&  RxDataBuffer[0] != ID_DAP_Info) {
        if (sw_lock(E_SWLOCK_DAPV1)) {
            hid_swd_connected = true;
            picoprobe_info("=================================== DAPv1 connect target\n");
            led_state(LS_DAPV1_CONNECTED);
        }
    }
    if (RxDataBuffer[0] == ID_DAP_Disconnect) {
        hid_swd_disconnect_requested = true;

        // this is the minimum version which should always work
        dap_packet_count = _DAP_PACKET_COUNT_HID;
        dap_packet_size  = _DAP_PACKET_SIZE_HID;
    }
    else if (RxDataBuffer[0] == ID_DAP_Info  ||  RxDataBuffer[0] == ID_DAP_HostStatus) {
        // ignore
    }
    else {
        hid_swd_disconnect_requested = false;
    }

    //
    // execute request and send back response
    //
    if (hid_swd_connected  ||  DAP_OfflineCommand(RxDataBuffer)) {
#if 0
        // heavy debug output, set dap_packet_count=2 to stumble into the bug
        uint32_t request_len = DAP_GetCommandLength(RxDataBuffer, bufsize);
        picoprobe_info("-----------------------------------------------\n");
        picoprobe_info("< (%lx) ", request_len);
        for (int i = 0;  i < bufsize;  ++i) {
            picoprobe_info_out(" %02x", RxDataBuffer[i]);
            if (i == request_len - 1) {
                picoprobe_info_out(" !!!!");
            }
        }
        picoprobe_info_out("\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        uint32_t res = DAP_ExecuteCommand(RxDataBuffer, TxDataBuffer);
        picoprobe_info("> (%lx) ", res);
        for (int i = 0;  i < bufsize;  ++i) {
            picoprobe_info_out(" %02x", TxDataBuffer[i]);
            if (i == (res & 0xffff) - 1) {
                picoprobe_info_out(" !!!!");
            }
        }
        picoprobe_info_out("\n");
#else
        uint32_t res = DAP_ExecuteCommand(RxDataBuffer, TxDataBuffer);
#endif
        tud_hid_report(0, TxDataBuffer, res & 0xffff);
    }
}   // tud_hid_set_report_cb
#endif



bool dap_is_connected(void)
{
    bool r = false;

#if OPT_CMSIS_DAPV1
    r = r || hid_swd_connected;
#endif
#if OPT_CMSIS_DAPV2
    r = r || swd_connected;
#endif

    return r;
}   // dap_is_connected



#ifdef NEW_DAP

static void HandleDapConnectDisconnect(const uint8_t *cmd, uint32_t cmdlen)
/**
 * Handle DAP connect & disconnect.
 * - get and release sw_lock accordingly
 * - set LED state
 */
{
    dap_tool = DAP_FingerprintTool(cmd, cmdlen);

    if ( !swd_connected) {
        if ( !DAP_OfflineCommand(cmd)) {
            if (sw_lock(E_SWLOCK_DAPV2)) {
                swd_connected = true;
                picoprobe_info("=================================== DAPv2 connect target, host %s, buffer: %dx%dbytes\n",
                        (dap_tool == E_DAPTOOL_OPENOCD) ? "OpenOCD"  :
                        (dap_tool == E_DAPTOOL_PYOCD)   ? "pyOCD"    :
                        (dap_tool == E_DAPTOOL_PROBERS) ? "probe-rs" : "UNKNOWN", dap_packet_count, dap_packet_size);
//                picoprobe_debug("------------ %d (command leading to online)\n", RxDataBuffer[0]);
                led_state(LS_DAPV2_CONNECTED);
            }

            // there was an obscure case with probe-rs which did not get the lock on "probe-rs info"
            // this seems to be no longer the case, but this is kept as a reminder that there exists some
            // code in ancient branches to handle the situation
        }
        else {
            // ID_DAP_Info must/can be done without a lock
#if DAP_DEBUG
            picoprobe_info("-----------__ %s (execute offline)\n", dap_cmd_string[RD_SLOT_PTR(requestQueue)[0]]);
#endif
        }
    }
    else {
        // connected:
        if (*cmd == ID_DAP_Disconnect) {
            swd_connected = false;
            picoprobe_info("=================================== DAPv2 disconnect target\n");
            led_state(LS_DAPV2_DISCONNECTED);
            sw_unlock(E_SWLOCK_DAPV2);
        }
    }
}   // HandleDapConnectDisconnect



void dap_edpt_init(void)
{
#if DAP_DEBUG
    picoprobe_info("dap_edpt_init\n");
#endif
    edpt_spoon = xSemaphoreCreateMutex();
    xSemaphoreGive(edpt_spoon);
}   // dap_edpt_init



bool dap_edpt_deinit(void)
{
#if DAP_DEBUG
    picoprobe_info("dap_edpt_deinit\n");
#endif
    vSemaphoreDelete(edpt_spoon);
    return true;
}   // dap_edpt_deinit



void dap_edpt_reset(uint8_t __unused rhport)
{
#if DAP_DEBUG
    picoprobe_info("dap_edpt_reset %d\n", rhport);
#endif
    itf_num = 0;
}   // dap_edpt_reset



uint16_t dap_edpt_open(uint8_t __unused _rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
    TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass &&
              DAP_INTERFACE_SUBCLASS == itf_desc->bInterfaceSubClass &&
              DAP_INTERFACE_PROTOCOL == itf_desc->bInterfaceProtocol, 0);

#if DAP_DEBUG
    picoprobe_info("dap_edpt_open\n");
#endif

    memset( &requestQueue,  0, sizeof(requestQueue));
    memset( &responseQueue, 0, sizeof(responseQueue));

    uint16_t const drv_len = sizeof(tusb_desc_interface_t) + (itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
    TU_VERIFY(max_len >= drv_len, 0);
    itf_num = itf_desc->bInterfaceNumber;
    rhport = _rhport;

    //
    // Initialize the OUT endpoint (host -> device)
    //
    tusb_desc_endpoint_t *edpt_desc = (tusb_desc_endpoint_t *) (itf_desc + 1);
    uint8_t ep_addr = edpt_desc->bEndpointAddress;
    out_ep_addr = ep_addr;

    // The OUT endpoint requires a call to usbd_edpt_xfer to initialize the endpoint, giving TinyUSB a buffer to consume when a transfer occurs at the endpoint
    usbd_edpt_open(rhport, edpt_desc);

#if TUSB_VERSION_NUMBER <= 2000
    usbd_edpt_xfer(rhport, out_ep_addr, WR_SLOT_PTR(requestQueue), _DAP_PACKET_SIZE_NEW);
#else
    usbd_edpt_xfer(rhport, out_ep_addr, WR_SLOT_PTR(requestQueue), _DAP_PACKET_SIZE_NEW, true);
#endif

    //
    // Initialize the IN endpoint (device -> host)
    //
    edpt_desc++;
    ep_addr = edpt_desc->bEndpointAddress;
    in_ep_addr = ep_addr;

    // The IN endpoint doesn't need a transfer to initialize it, as this will be done by the main loop of dap_thread
    usbd_edpt_open(rhport, edpt_desc);

    return drv_len;
}   // dap_edpt_open



bool dap_edpt_control_xfer_cb(uint8_t __unused rhport, uint8_t stage, tusb_control_request_t const *request)
{
#if DAP_DEBUG
    picoprobe_info("dap_edpt_control_xfer_cb\n");
#endif
    return false;
}   // dap_edpt_control_xfer_cb



bool dap_edpt_xfer_isr_cb(uint8_t __unused rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
//    picoprobe_info("dap_edpt_xfer_isr_cb !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    return false;
}   // dap_edpt_xfer_isr_cb



bool dap_edpt_xfer_cb(uint8_t __unused rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    const uint8_t ep_dir = tu_edpt_dir(ep_addr);
    bool r = false;

    xSemaphoreTake(edpt_spoon, portMAX_DELAY);

    if (ep_dir == TUSB_DIR_IN)
    {
//        printf(">\n");
        if (xferred_bytes >= 0u  &&  xferred_bytes <= _DAP_PACKET_SIZE_NEW)
        {
            if (xferred_bytes != RD_SLOT_LEN(responseQueue))
            {
                picoprobe_error("dap_edpt_xfer_cb(): did not transmit complete response!?\n");
            }

            // mark the buffer as empty & advance to next buffer
            RD_SLOT_LEN(responseQueue) = 0;
            responseQueue.rd_idx = MOD_PACKET_COUNT(responseQueue.rd_idx + 1);

            // start transmission of next pending response
            if (RD_SLOT_LEN(responseQueue) != 0)
            {
#if TUSB_VERSION_NUMBER <= 2000
                usbd_edpt_xfer(rhport, ep_addr, RD_SLOT_PTR(responseQueue), RD_SLOT_LEN(responseQueue));
#else
                usbd_edpt_xfer(rhport, ep_addr, RD_SLOT_PTR(responseQueue), RD_SLOT_LEN(responseQueue), true);
#endif
            }
            else
            {
                responseQueue.xmtRunning = false;
            }

            r = true;
        }
    }
    else if (ep_dir == TUSB_DIR_OUT)
    {
//        printf("<\n");
        if (xferred_bytes >= 0u  &&  xferred_bytes <= _DAP_PACKET_SIZE_NEW)
        {
            // validate buffer for reading
            WR_SLOT_LEN(requestQueue) = xferred_bytes;

#if _DAP_PACKET_SIZE_NEW != 64
            if (xferred_bytes == DAP_GetCommandLength(RD_SLOT_PTR(requestQueue), xferred_bytes) + 1)
            {
                // this is a special pyocd (<= 0.42.0) hack (and of course openocd does not like it)
                // see https://github.com/pyocd/pyOCD/issues/1871
                picoprobe_error("dap_edpt_xfer_cb: zero packet received.  This is from pyocd <= 0.42.0.  Please update.\n");
                --WR_SLOT_LEN(requestQueue);
            }
#endif

            // Only queue the next buffer in the out callback if the queue is not full
            // If full, we set the rcvStartDelayed flag, which will be checked by dap thread
            if (WR_NEXT_SLOT_LEN(requestQueue) == 0)
            {
                requestQueue.wr_idx = MOD_PACKET_COUNT(requestQueue.wr_idx + 1);
#if TUSB_VERSION_NUMBER <= 2000
                usbd_edpt_xfer(rhport, ep_addr, WR_SLOT_PTR(requestQueue), _DAP_PACKET_SIZE_NEW);
#else
                usbd_edpt_xfer(rhport, ep_addr, WR_SLOT_PTR(requestQueue), _DAP_PACKET_SIZE_NEW, true);
#endif
                requestQueue.rcvStartDelayed = false;
            }
            else
            {
#if DAP_DEBUG
                printf("!!!!!!!! %d %d\n", (int)requestQueue.rd_idx, (int)requestQueue.wr_idx);
#endif
                requestQueue.rcvStartDelayed = true;
            }

#if DAP_DEBUG
            {
                int diff = MOD_PACKET_COUNT(requestQueue.wr_idx - requestQueue.rd_idx);

                if (diff > requestQueue.dmax)
                {
                    requestQueue.dmax = diff;
                    picoprobe_info("dap_edpt_xfer_cb, dmax request %d\n", diff);
                }
            }
#endif

            r = true;
        }
    }

    xSemaphoreGive(edpt_spoon);

    //  Wake up DAP thread after processing the callback
    xTaskNotify(dap_taskhandle, 0, eSetValueWithOverwrite);
    return r;
}   // dap_edpt_xfer_cb



void dap_thread(void *ptr)
{
    uint32_t cmd;
    uint16_t resp_len;

    for (;;)
    {
        // Wait for wakeup from dap_edpt_xfer_cb
        xTaskNotifyWait(0, 0xFFFFFFFFu, &cmd, 1);

#if OPT_CMSIS_DAPV1
        // packet parameters must be restored in this case
        dap_packet_count = _DAP_PACKET_COUNT_NEW;
        dap_packet_size  = _DAP_PACKET_SIZE_NEW;
#endif

        while (RD_SLOT_LEN(requestQueue) != 0)
        {
            //
            // Check next DAP request
            //
            if (RD_SLOT_LEN(requestQueue) != DAP_GetCommandLength(RD_SLOT_PTR(requestQueue), RD_SLOT_LEN(requestQueue)))
            {
                picoprobe_error("dap_thread(): malformed request (probe may crash)\n");
            }
#if 0  &&  DAP_DEBUG
            picoprobe_info("%u %u DAP cmd %s len %d %d\n",
                           (unsigned)requestQueue.wr_idx, (unsigned)requestQueue.rd_idx,
                           dap_cmd_string[RD_SLOT_PTR(requestQueue)[0]],
                           (int)RD_SLOT_LEN(requestQueue),
                           (int)DAP_GetCommandLength(RD_SLOT_PTR(requestQueue), RD_SLOT_LEN(requestQueue)));
#endif

            //
            // execute DAP request
            //
            HandleDapConnectDisconnect(RD_SLOT_PTR(requestQueue), RD_SLOT_LEN(requestQueue));
            resp_len = DAP_ExecuteCommand(RD_SLOT_PTR(requestQueue), WR_SLOT_PTR(responseQueue)) & 0xffff;

            xSemaphoreTake(edpt_spoon, portMAX_DELAY);

            // mark the request buffer as empty & advance to next buffer
            RD_SLOT_LEN(requestQueue) = 0;
            requestQueue.rd_idx = MOD_PACKET_COUNT(requestQueue.rd_idx + 1);

            // If the queue was full in the out callback, we need to queue up another buffer for the endpoint to consume, now that we know there is space in the buffer.
            if (requestQueue.rcvStartDelayed)
            {
#if DAP_DEBUG
                printf("........ %d %d\n", (int)requestQueue.rd_idx, (int)requestQueue.wr_idx);
#endif
                requestQueue.wr_idx = MOD_PACKET_COUNT(requestQueue.wr_idx + 1);
#if TUSB_VERSION_NUMBER <= 2000
                usbd_edpt_xfer(rhport, out_ep_addr, WR_SLOT_PTR(requestQueue), _DAP_PACKET_SIZE_NEW);
#else
                usbd_edpt_xfer(rhport, out_ep_addr, WR_SLOT_PTR(requestQueue), _DAP_PACKET_SIZE_NEW, true);
#endif
                requestQueue.rcvStartDelayed = false;
            }
            xSemaphoreGive(edpt_spoon);

            if (resp_len == 0)
            {
                picoprobe_error("DAP_ExecuteCommand() must return a response (probe may crash)\n");
            }
            else
            {
#if 0  &&  DAP_DEBUG
                picoprobe_info("%u %u DAP resp %s len %d\n",
                               (unsigned)responseQueue.wr_idx, (unsigned)responseQueue.rd_idx,
                               dap_cmd_string[WR_SLOT_PTR(responseQueue)[0]], resp_len);
#endif

                //  Suspend the scheduler to avoid stale values/race conditions between threads
                xSemaphoreTake(edpt_spoon, portMAX_DELAY);

                WR_SLOT_LEN(responseQueue) = resp_len;
                responseQueue.wr_idx = MOD_PACKET_COUNT(responseQueue.wr_idx + 1);

                if ( !responseQueue.xmtRunning)
                {
                    responseQueue.xmtRunning = true;

#if TUSB_VERSION_NUMBER <= 2000
                    usbd_edpt_xfer(rhport, in_ep_addr, RD_SLOT_PTR(responseQueue), RD_SLOT_LEN(responseQueue));
#else
                    usbd_edpt_xfer(rhport, in_ep_addr, RD_SLOT_PTR(responseQueue), RD_SLOT_LEN(responseQueue), true);
#endif
                }

                xSemaphoreGive(edpt_spoon);
            }
        }
    }
}   // dap_thread



usbd_class_driver_t const _dap_edpt_driver =
{
        .init            = dap_edpt_init,
        .deinit          = dap_edpt_deinit,
        .reset           = dap_edpt_reset,
        .open            = dap_edpt_open,
        .control_xfer_cb = dap_edpt_control_xfer_cb,
        .xfer_cb         = dap_edpt_xfer_cb,
        .xfer_isr        = dap_edpt_xfer_isr_cb,
        .sof             = NULL,
#if CFG_TUSB_DEBUG >= 2
        .name            = "DAP ENDPOINT"
#endif
};



usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
/**
 *  Add the custom driver to the TinyUSB stack
 */
{
    *driver_count = 1;
    return &_dap_edpt_driver;
}   // usbd_app_driver_get_cb
#endif



#if OPT_CMSIS_DAPV2  &&  !defined(NEW_DAP)
static void dap_reset_tool_timeout(TimerHandle_t xTimer)
{
    // probe-rs does not like this (because it does reconnects with other fingerprints)
    picoprobe_debug("dap_reset_tool_timeout\n");
    dap_packet_count = _DAP_PACKET_COUNT_UNKNOWN;
    dap_packet_size  = _DAP_PACKET_SIZE_UNKNOWN;
    dap_tool = DAP_FingerprintTool(NULL, 0);
}   // dap_reset_tool_timeout
#endif



void dap_server_init(uint32_t task_prio)
{
    picoprobe_debug("dap_server_init(%u)\n", (unsigned)task_prio);

#ifdef NEW_DAP
    /* Lowest priority thread is debug - need to shuffle buffers before we can toggle swd... */
    xTaskCreate(dap_thread, "DAP", configMINIMAL_STACK_SIZE, NULL, task_prio, &dap_taskhandle);

    #if defined(configUSE_CORE_AFFINITY)  &&  configUSE_CORE_AFFINITY != 0
        vTaskCoreAffinitySet(dap_taskhandle, (1 << 1));
    #endif
#endif

#if OPT_CMSIS_DAPV2  &&  !defined(NEW_DAP)
    dap_stream = xStreamBufferCreate(BUFFER_MAXSIZE, 1);
    if (dap_stream == NULL) {
        picoprobe_error("dap_server_init: cannot create dap_stream\n");
    }
    dap_events = xEventGroupCreate();
    timer_clear_dap_tool = xTimerCreate("DAP reset tools", pdMS_TO_TICKS(50), pdFALSE, NULL, dap_reset_tool_timeout);
    xTaskCreate(dap_task, "CMSIS-DAPv2", configMINIMAL_STACK_SIZE, NULL, task_prio, &dap_taskhandle);
#endif
}   // rtt_console_init
