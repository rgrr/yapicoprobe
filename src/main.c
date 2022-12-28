/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2021 Peter Lawrence
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

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"

#include "picoprobe_config.h"
#include "probe.h"
#include "cdc_debug.h"
#include "cdc_uart.h"
#include "dap_util.h"
#include "get_serial.h"
#include "led.h"
#include "DAP_config.h"
#include "DAP.h"
#include "sw_lock.h"

#if CFG_TUD_MSC
    #include "msc/msc_utils.h"
#endif
#if defined(INCLUDE_RTT_CONSOLE)
    #include "rtt_console.h"
#endif

// UART1 for Picoprobe to target device

static uint8_t TxDataBuffer[DAP_PACKET_COUNT * DAP_PACKET_SIZE];
static uint8_t RxDataBuffer[DAP_PACKET_COUNT * DAP_PACKET_SIZE];


// prios are critical and determine throughput
#define TUD_TASK_PRIO               (tskIDLE_PRIORITY + 20)       // uses one core continuously
#define LED_TASK_PRIO               (tskIDLE_PRIORITY + 12)       // simple task which may interrupt everything else for periodic blinking
#define MSC_WRITER_THREAD_PRIO      (tskIDLE_PRIORITY + 8)        // this is only running on writing UF2 files
#define UART_TASK_PRIO              (tskIDLE_PRIORITY + 5)        // target -> host via UART
#define RTT_CONSOLE_TASK_PRIO       (tskIDLE_PRIORITY + 5)        // target -> host via RTT
#define CDC_DEBUG_TASK_PRIO         (tskIDLE_PRIORITY + 4)        // probe debugging output
#define DAP_TASK_PRIO               (tskIDLE_PRIORITY + 1)        // DAP execution, during connection this takes the other core

static TaskHandle_t tud_taskhandle;
static TaskHandle_t dap_taskhandle;



void dap_task(void *ptr)
{
    bool mounted = false;
    uint32_t used_us;
    uint32_t rx_len = 0;

    for (;;) {
        if (tud_vendor_available()) {
            used_us = time_us_32();
            if ( !mounted) {
                if (sw_lock("DAPv2", true)) {
                    mounted = true;
                    picoprobe_debug("=================================== DAPv2 connect target\n");
                    led_state(LS_DAPV2_CONNECTED);
                }
            }

            if (mounted) {
                rx_len += tud_vendor_read(RxDataBuffer + rx_len, sizeof(RxDataBuffer));

                if (rx_len != 0)
                {
                    uint32_t request_len;

                    request_len = DAP_GetCommandLength(RxDataBuffer, rx_len);
                    if (rx_len >= request_len)
                    {
                        uint32_t resp_len;

                        resp_len = DAP_ExecuteCommand(RxDataBuffer, TxDataBuffer);
                        tud_vendor_write(TxDataBuffer, resp_len & 0xffff);
                        tud_vendor_flush();

                        if (request_len != (resp_len >> 16))
                        {
                            // there is a bug in CMSIS-DAP, see https://github.com/ARM-software/CMSIS_5/pull/1503
                            // but we trust our own length calculation
                            picoprobe_error("   !!!!!!!! request (%lu) and executed length (%lu) differ\n", request_len, resp_len >> 16);
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
            }
        }
        else {
            // disconnect after 1s without data
            if (mounted  &&  time_us_32() - used_us > 1000000) {
                mounted = false;
                picoprobe_debug("=================================== DAPv2 disconnect target\n");
                led_state(LS_DAPV2_DISCONNECTED);
                sw_unlock("DAPv2");
            }
            taskYIELD();
        }
    }
}   // dap_task



void usb_thread(void *ptr)
{
    picoprobe_info("system starting...\n");

    cdc_uart_init(UART_TASK_PRIO);

#if CFG_TUD_MSC
    msc_init(MSC_WRITER_THREAD_PRIO);
#endif

    for (;;) {
        tud_task();
        taskYIELD();    // not sure, if this triggers the scheduler
    }
}   // usb_thread



int main(void)
{
    board_init();
    set_sys_clock_khz(CPU_CLOCK / 1000, true);

    usb_serial_init();
    tusb_init();

    // should be done before anything else (that does cdc_debug_printf())
#if !defined(NDEBUG)
    cdc_debug_init(CDC_DEBUG_TASK_PRIO);
#endif

#if defined(INCLUDE_RTT_CONSOLE)
    rtt_console_init(RTT_CONSOLE_TASK_PRIO);
#endif

    // now we can "print"
    picoprobe_info("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    picoprobe_info("                        Welcome to Yet Another Picoprobe v%02x.%02x\n", PICOPROBE_VERSION >> 8, PICOPROBE_VERSION & 0xff);
    picoprobe_info("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

    sw_lock_init();
    led_init(LED_TASK_PRIO);

    DAP_Setup();

    xTaskCreate(usb_thread, "TUD", configMINIMAL_STACK_SIZE, NULL, TUD_TASK_PRIO, &tud_taskhandle);
    xTaskCreate(dap_task, "DAP", configMINIMAL_STACK_SIZE, NULL, DAP_TASK_PRIO, &dap_taskhandle);
    vTaskStartScheduler();

    return 0;
}



static bool hid_mounted;
static TimerHandle_t     timer_hid_disconnect = NULL;
static void             *timer_hid_disconnect_id;


static void hid_disconnect(TimerHandle_t xTimer)
{
    if (hid_mounted) {
        hid_mounted = false;
        picoprobe_debug("=================================== DAPv1 disconnect target\n");
        led_state(LS_DAPV1_DISCONNECTED);
        sw_unlock("DAPv1");
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
    uint32_t response_size = TU_MIN(CFG_TUD_HID_EP_BUFSIZE, bufsize);

    // TODO do sw_lock() / unlock! (with timer)

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

    if ( !hid_mounted) {
        if (sw_lock("DAPv1", true)) {
            hid_mounted = true;
            picoprobe_debug("=================================== DAPv1 connect target\n");
            led_state(LS_DAPV1_CONNECTED);
        }
    }

    if (hid_mounted) {
        DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);
        tud_hid_report(0, TxDataBuffer, response_size);
    }
}   // tud_hid_set_report_cb



#if (PICOPROBE_DEBUG_PROTOCOL == PROTO_DAP_V2)
extern uint8_t const desc_ms_os_20[];

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    // nothing to with DATA & ACK stage
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
}
#endif



void vApplicationTickHook (void)
{
}



void vApplicationStackOverflowHook(TaskHandle_t Task, char *pcTaskName)
{
  panic("stack overflow (not the helpful kind) for %s\n", *pcTaskName);
}



void vApplicationMallocFailedHook(void)
{
  panic("Malloc Failed\n");
}
