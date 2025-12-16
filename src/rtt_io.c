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

#include "FreeRTOS.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "task.h"
#include "timers.h"

#include "target_board.h"
#include "swd_host.h"

#include "picoprobe_config.h"
#include "rtt_io.h"
#include "sw_lock.h"
#include "RTT/SEGGER_RTT.h"
#if OPT_TARGET_UART
    #include "cdc/cdc_uart.h"
#endif
#if OPT_CDC_SYSVIEW
    #include "cdc/cdc_sysview.h"
#endif
#if OPT_NET_SYSVIEW_SERVER
    #include "net/net_sysview.h"
#endif
#include "led.h"
#include "cmsis-dap/dap_server.h"

#if OPT_CDC_SYSVIEW  ||  OPT_NET_SYSVIEW_SERVER
    #define INCLUDE_SYSVIEW     1
#endif


typedef uint32_t (*rtt_data_to_host)(const uint8_t *buf, uint32_t cnt);

typedef struct {
    uint32_t                addr;                // target address of this aUp[]
    SEGGER_RTT_BUFFER_UP    aUp;                 // local copy of the target aUp[]
} EXT_SEGGER_RTT_BUFFER_UP;

typedef struct {
    uint32_t                addr;                // target address of this aDown[]
    SEGGER_RTT_BUFFER_DOWN  aDown;               // local copy of the target aDown[]
} EXT_SEGGER_RTT_BUFFER_DOWN;

typedef struct {                                                      // see SEGGER_RTT_CB
    char                    acID[16];                                 // Initialized to "SEGGER RTT"
    int                     MaxNumUpBuffers;                          // Initialized to SEGGER_RTT_MAX_NUM_UP_BUFFERS (type. 2)
    int                     MaxNumDownBuffers;                        // Initialized to SEGGER_RTT_MAX_NUM_DOWN_BUFFERS (type. 2)
} EXT_SEGGER_RTT_CB_HEADER;

#define TARGET_RAM_START        (g_board_info.target_cfg->ram_regions[0].start)
#define TARGET_RAM_END          (g_board_info.target_cfg->ram_regions[0].end)

#define STREAM_RTT_SIZE         128
#define STREAM_RTT_TRIGGER      1

#define RTT_CHANNEL_CONSOLE     0
#define RTT_CONSOLE_POLL_INT_MS 10

#define EV_RTT_TO_TARGET        0x01
#define EV_RTT_FROM_TARGET_STRT 0x02
#define EV_RTT_FROM_TARGET_END  0x04

static const uint32_t           segger_alignment = 4;
static const uint8_t            seggerRTT[16] = "SEGGER RTT\0\0\0\0\0\0";
static EXT_SEGGER_RTT_CB_HEADER rtt_cb_header = {0};
static bool                     rtt_console_running = false;
static bool                     ok_console_from_target = false;
static bool                     ok_console_to_target = false;

static TaskHandle_t             task_rtt_console = NULL;
static TaskHandle_t             task_rtt_from_target_thread = NULL;
static StreamBufferHandle_t     stream_rtt_console_to_target;                  // small stream for host->probe->target console communication
static EventGroupHandle_t       events;
static TimerHandle_t            timer_rtt_dap_interleave;                      // minimum time do_rtt_io() should be active if DAP connected

#if INCLUDE_SYSVIEW
    #define RTT_CHANNEL_SYSVIEW 1
    #define RTT_POLL_INT_MS     1                                              // faster polling
    static StreamBufferHandle_t stream_rtt_sysview_to_target;                  // small stream for host->probe->target sysview communication
#else
    #define RTT_POLL_INT_MS     RTT_CONSOLE_POLL_INT_MS
#endif



static void rtt_cb_verify_timeout(TimerHandle_t xTimer)
{
}   // rtt_cb_verify_timeout



static uint32_t check_buffer_for_rtt_cb(uint8_t *buf, uint32_t buf_size, uint32_t base_addr)
{
    uint32_t rtt_cb = 0;

    for (uint32_t ndx = 0;  ndx <= buf_size - sizeof(rtt_cb_header);  ndx += segger_alignment) {
        if (memcmp(buf + ndx, seggerRTT, sizeof(seggerRTT)) == 0) {
            memcpy( &rtt_cb_header, buf + ndx, sizeof(rtt_cb_header));
            rtt_cb = base_addr + ndx;
            break;
        }
    }
    return rtt_cb;
}   // check_buffer_for_rtt_cb



static bool rtt_check_control_block_header(uint32_t rtt_cb)
{
    bool ok = false;
    EXT_SEGGER_RTT_CB_HEADER rtt_cb_header_buf;

    if (rtt_cb < TARGET_RAM_START  ||  rtt_cb > TARGET_RAM_END - sizeof(EXT_SEGGER_RTT_CB_HEADER)) {
        // -> out of range
    }
    else if (memcmp(seggerRTT, rtt_cb_header.acID, sizeof(seggerRTT)) != 0) {
        // -> rtt_cb_header is not yet valid
    }
    else {
        ok = swd_read_memory(rtt_cb, (uint8_t *)&rtt_cb_header_buf, sizeof(rtt_cb_header_buf));
        ok = ok  &&  memcmp( &rtt_cb_header, &rtt_cb_header_buf, sizeof(rtt_cb_header_buf)) == 0;
    }
    return ok;
}   // rtt_check_control_block_header



static bool is_target_ok(uint32_t addr)
/**
 * Check if the target is still ok (after an attach)
 */
{
    uint8_t num[4];
    return swd_read_memory((addr != 0) ? addr : TARGET_RAM_START, num, sizeof(num));
}   // is_target_ok



static bool search_for_rtt_cb(uint32_t *p_rtt_cb)
/**
 * Search for the RTT control block.
 *
 * \param p_rtt_cb  where the search begins or zero for new scan, on exit it holds the last
 *                  RAM block scanned for RTT_CB
 * \return true -> \a p_rtt_cb holds a valid control block
 *
 * \note
 *    Searching all 256KByte RAM of the RP2040 takes 600ms (at 12.5MHz interface clock)
 */
{
    uint8_t buf[1024 + sizeof(EXT_SEGGER_RTT_CB_HEADER)];
    bool res = false;
    uint32_t rtt_cb = *p_rtt_cb;

//    picoprobe_info("-----------search_for_rtt_cb(%08x)\n", (unsigned int)rtt_cb);

    // check parameter
    if (rtt_cb < TARGET_RAM_START  ||  rtt_cb >= TARGET_RAM_END - sizeof(EXT_SEGGER_RTT_CB_HEADER)) {
        rtt_cb = TARGET_RAM_START;
    }
    else if (rtt_check_control_block_header(rtt_cb)) {
        // -> valid control block!
        return true;
    }

    if (dap_is_connected()) {
        xTimerReset(timer_rtt_dap_interleave, 100);
    }

    // note that searches must somehow overlap to find (unaligned) control blocks at the border of read chunks
    for (uint32_t addr = rtt_cb;  addr < TARGET_RAM_END - sizeof(seggerRTT);  addr += sizeof(buf) - sizeof(EXT_SEGGER_RTT_CB_HEADER)) {
        uint32_t size = MIN(sizeof(buf), TARGET_RAM_END - addr);
        if ( !swd_read_memory(addr, buf, size)) {
            *p_rtt_cb = 0;
            break;
        }

        rtt_cb = check_buffer_for_rtt_cb(buf, size, addr);
        if (rtt_cb != 0) {
            *p_rtt_cb = rtt_cb;
            res = true;
            break;
        }

        *p_rtt_cb = addr + (sizeof(buf) - sizeof(EXT_SEGGER_RTT_CB_HEADER));

        if (dap_is_connected()) {
            if (sw_unlock_requested()  &&  !xTimerIsTimerActive(timer_rtt_dap_interleave)) {
                break;
            }
        }
        else if (sw_unlock_requested()) {
            break;
        }
    }

    if (dap_is_connected()) {
        xTimerStop(timer_rtt_dap_interleave, 100);
    }
    return res;
}   // search_for_rtt_cb



static bool rtt_check_channel_from_target(uint32_t rtt_cb, uint16_t channel, EXT_SEGGER_RTT_BUFFER_UP *extRttBuf, bool *found)
/**
 * Check if there is a valid buffer from target for this channel.
 */
{
    bool ok;
    int32_t buff_cnt;

    *found = (rtt_cb >= TARGET_RAM_START  &&  rtt_cb <= TARGET_RAM_END);
    ok = *found  &&  swd_read_word(rtt_cb + offsetof(SEGGER_RTT_CB, MaxNumUpBuffers), (uint32_t *)&(buff_cnt));
    if (ok) {
        extRttBuf->addr = rtt_cb + offsetof(SEGGER_RTT_CB, aUp[channel]);
        *found = *found  &&  (buff_cnt > channel);
        *found = *found  &&  swd_read_memory(extRttBuf->addr, (uint8_t *)&(extRttBuf->aUp), sizeof(extRttBuf->aUp));
        *found = *found  &&  (extRttBuf->aUp.SizeOfBuffer > 0  &&  extRttBuf->aUp.SizeOfBuffer < TARGET_RAM_END - TARGET_RAM_START);
        *found = *found  &&  ((uint32_t)extRttBuf->aUp.pBuffer >= TARGET_RAM_START  &&  (uint32_t)extRttBuf->aUp.pBuffer + extRttBuf->aUp.SizeOfBuffer <= TARGET_RAM_END);
        if (*found) {
            picoprobe_debug("     rtt_check_channel_from_target: %u %p %5u %5u %5u\n", channel, extRttBuf->aUp.pBuffer, extRttBuf->aUp.SizeOfBuffer, extRttBuf->aUp.RdOff, extRttBuf->aUp.WrOff);
        }
    }
    return ok;
}   // rtt_check_channel_from_target



static bool rtt_check_channel_to_target(uint32_t rtt_cb, uint16_t channel, EXT_SEGGER_RTT_BUFFER_DOWN *extRttBuf, bool *found)
/**
 * Check if there is a valid buffer to target for this channel.
 *
 * \note
 *    Order of \a SEGGER_RTT_CB must be up buffer first, then down buffers.
 */
{
    bool ok;
    int32_t buff_cnt;
    int32_t buff_cnt_up;

    *found = (rtt_cb >= TARGET_RAM_START  &&  rtt_cb <= TARGET_RAM_END);
    ok = *found  &&  swd_read_word(rtt_cb + offsetof(SEGGER_RTT_CB, MaxNumDownBuffers), (uint32_t *)&(buff_cnt))
                 &&  swd_read_word(rtt_cb + offsetof(SEGGER_RTT_CB, MaxNumUpBuffers), (uint32_t *)&(buff_cnt_up));
    if (ok) {
        extRttBuf->addr = rtt_cb + offsetof(SEGGER_RTT_CB, aUp[buff_cnt_up]) + channel * sizeof(SEGGER_RTT_BUFFER_DOWN);
        *found = *found  &&  (buff_cnt > channel);
        *found = *found  &&  swd_read_memory(extRttBuf->addr, (uint8_t *)&(extRttBuf->aDown), sizeof(extRttBuf->aDown));
        *found = *found  &&  (extRttBuf->aDown.SizeOfBuffer > 0  &&  extRttBuf->aDown.SizeOfBuffer < TARGET_RAM_END - TARGET_RAM_START);
        *found = *found  &&  ((uint32_t)extRttBuf->aDown.pBuffer >= TARGET_RAM_START  &&  (uint32_t)extRttBuf->aDown.pBuffer + extRttBuf->aDown.SizeOfBuffer <= TARGET_RAM_END);
        if (*found) {
            picoprobe_debug("     rtt_check_channel_to_target  : %u %p %5u %5u %5u\n", channel, extRttBuf->aDown.pBuffer, extRttBuf->aDown.SizeOfBuffer, extRttBuf->aDown.RdOff, extRttBuf->aDown.WrOff);
        }
    }
    return ok;
}   // rtt_check_channel_to_target



static unsigned rtt_get_write_space(SEGGER_RTT_BUFFER_DOWN *pRing)
/**
 * Return the number of space left in the target buffer
 */
{
    unsigned rd_off;
    unsigned wr_off;
    unsigned r;

    rd_off = pRing->RdOff;
    wr_off = pRing->WrOff;
    if (rd_off <= wr_off) {
        r = pRing->SizeOfBuffer - 1u - wr_off + rd_off;
    }
    else {
        r = rd_off - wr_off - 1u;
    }
    return r;
}   // rtt_get_write_space



// ft = from target
static EXT_SEGGER_RTT_BUFFER_UP *ft_extRttBuf;
static uint8_t ft_buf[256];
static uint32_t ft_cnt;
static bool ft_ok;

static void rtt_from_target_thread(void *p)
/**
 * Fetch RTT data from target.
 * Data transfer is CPU intensive, because SWD access is blocking the CPU.
 * So the idea is to put this task in an extra thread with affinity to the second core.
 * Core affinity is set in \a main.c
 */
{
    for (;;) {
        EventBits_t ev;

        ev = xEventGroupWaitBits(events, EV_RTT_FROM_TARGET_STRT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (ev == 0) {
            continue;
        }

        ft_ok = swd_read_word(ft_extRttBuf->addr + offsetof(SEGGER_RTT_BUFFER_UP, WrOff), (uint32_t *)&(ft_extRttBuf->aUp.WrOff));

        if (ft_ok  &&  ft_extRttBuf->aUp.WrOff != ft_extRttBuf->aUp.RdOff) {
            //
            // fetch data from target
            //
            if (ft_extRttBuf->aUp.WrOff > ft_extRttBuf->aUp.RdOff) {
                ft_cnt = MIN(ft_cnt, ft_extRttBuf->aUp.WrOff - ft_extRttBuf->aUp.RdOff);
            }
            else {
                ft_cnt = MIN(ft_cnt, ft_extRttBuf->aUp.SizeOfBuffer - ft_extRttBuf->aUp.RdOff);
            }
            ft_cnt = MIN(ft_cnt, sizeof(ft_buf));

            memset(ft_buf, 0, sizeof(ft_buf));
            ft_ok = ft_ok  &&  swd_read_memory((uint32_t)ft_extRttBuf->aUp.pBuffer + ft_extRttBuf->aUp.RdOff, ft_buf, ft_cnt);
            ft_extRttBuf->aUp.RdOff = (ft_extRttBuf->aUp.RdOff + ft_cnt) % ft_extRttBuf->aUp.SizeOfBuffer;
            ft_ok = ft_ok  &&  swd_write_word(ft_extRttBuf->addr + offsetof(SEGGER_RTT_BUFFER_UP, RdOff), ft_extRttBuf->aUp.RdOff);
        }
        else {
            ft_cnt = 0;
        }

        xEventGroupSetBits(events, EV_RTT_FROM_TARGET_END);
    }
}   // rtt_from_target_thread



#if INCLUDE_SYSVIEW
static void rtt_from_target_reset(EXT_SEGGER_RTT_BUFFER_UP *extRttBuf)
/**
 * Reset an upstream buffer.
 */
{
    swd_read_word(extRttBuf->addr + offsetof(SEGGER_RTT_BUFFER_UP, WrOff), (uint32_t *)&(extRttBuf->aUp.WrOff));
    extRttBuf->aUp.RdOff = extRttBuf->aUp.WrOff;
    swd_write_word(extRttBuf->addr + offsetof(SEGGER_RTT_BUFFER_UP, RdOff), extRttBuf->aUp.RdOff);
}   // rtt_from_target_reset
#endif



static bool rtt_from_target(EXT_SEGGER_RTT_BUFFER_UP *extRttBuf,
                            rtt_data_to_host data_to_host, bool check_host_buffer, bool *worked)
/**
 * Fetch data via RTT from target.
 *
 * \param rtt_cb             address of RTT control block
 * \param channel            RTT channel number
 * \param aUp                copy of RTT up channel
 * \param data_to_host       function to transfer data to host
 * \param check_host_buffer  check if the host can receive this amount of data
 * \param worked             mark as true if there was no failure
 */
{
    bool send_data_to_host = true;

    if (check_host_buffer) {
        ft_cnt = data_to_host(NULL, 0);
        if (ft_cnt < sizeof(ft_buf) / 4) {
            //printf("no space in stream %d: %d\n", channel, ft_cnt);
            send_data_to_host = false;
            *worked = true;
        }
    }
    else {
        ft_cnt = sizeof(ft_buf);
    }

    if (send_data_to_host) {
        ft_extRttBuf = extRttBuf;

        xEventGroupSetBits(events, EV_RTT_FROM_TARGET_STRT);
        xEventGroupWaitBits(events, EV_RTT_FROM_TARGET_END, pdTRUE, pdFALSE, portMAX_DELAY);

        if (ft_cnt != 0) {
            // redirect received data to host
            data_to_host(ft_buf, ft_cnt);

            led_state(LS_RTT_RX_DATA);
            *worked = true;
        }
    }
    return ft_ok;
}   // rtt_from_target



static bool rtt_to_target(EXT_SEGGER_RTT_BUFFER_DOWN *extRttBuf, StreamBufferHandle_t stream, bool *worked)
{
    bool ok = true;
    uint8_t buf[16];
    unsigned num_bytes;

    if ( !xStreamBufferIsEmpty(stream)) {
        //
        // send data to target
        //
        ok = ok  &&  swd_read_word(extRttBuf->addr + offsetof(SEGGER_RTT_BUFFER_DOWN, RdOff), (uint32_t *)&(extRttBuf->aDown.RdOff));

        num_bytes = rtt_get_write_space( &(extRttBuf->aDown));
        if (num_bytes > 0) {
            //printf("a cnt: %u -> ", num_bytes);

            num_bytes = MIN(num_bytes, sizeof(buf));
            num_bytes = xStreamBufferReceive(stream, &buf, num_bytes, 0);
        }

        if (num_bytes > 0) {
            unsigned wr_off;
            unsigned remaining;

            wr_off = extRttBuf->aDown.WrOff;
            remaining = extRttBuf->aDown.SizeOfBuffer - wr_off;

            //printf("%u %u %u %u", channel, aDown->WrOff, num_bytes, remaining);

            if (remaining > num_bytes) {
                //
                // All data fits before wrap around
                //
                ok = ok  &&  swd_write_memory((uint32_t)extRttBuf->aDown.pBuffer + wr_off, buf, num_bytes);
                extRttBuf->aDown.WrOff = wr_off + num_bytes;
            }
            else {
                //
                // We reach the end of the buffer, so need to wrap around
                //
                unsigned num_bytes_at_once;

                num_bytes_at_once = remaining;
                ok = ok  &&  swd_write_memory((uint32_t)extRttBuf->aDown.pBuffer + wr_off, buf, num_bytes_at_once);
                num_bytes_at_once = num_bytes - remaining;
                ok = ok  &&  swd_write_memory((uint32_t)extRttBuf->aDown.pBuffer, buf + remaining, num_bytes_at_once);
                extRttBuf->aDown.WrOff = num_bytes_at_once;
            }

            ok = ok  &&  swd_write_word(extRttBuf->addr + offsetof(SEGGER_RTT_BUFFER_DOWN, WrOff), extRttBuf->aDown.WrOff);

            //printf(" -> %u\n", aDown->WrOff);
        }

        *worked = true;
    }
    return ok;
}   // rtt_to_target



static void do_rtt_io(uint32_t rtt_cb)
/**
 * Do RTT IO until either the RTT_CB is lost or if there is another SWD request.
 */
{
#if OPT_TARGET_UART
    EXT_SEGGER_RTT_BUFFER_UP   aUpConsole;       // Up buffer, transferring information up from target via debug probe to host
    EXT_SEGGER_RTT_BUFFER_DOWN aDownConsole;     // Down buffer, transferring information from host via debug probe to target
    ok_console_from_target = false;
    ok_console_to_target = false;
    bool working_uart = false;
#endif
#if INCLUDE_SYSVIEW
    EXT_SEGGER_RTT_BUFFER_UP   aUpSysView;       // Up buffer, transferring information up from target via debug probe to host
    EXT_SEGGER_RTT_BUFFER_DOWN aDownSysView;     // Down buffer, transferring information from host via debug probe to target
    bool ok_sysview_from_target = false;
    bool ok_sysview_to_target = false;
    bool net_sysview_was_connected = false;
    bool working_sysview = false;
#endif
    bool ok;
    bool probe_rtt_cb;

    static_assert(sizeof(uint32_t) == sizeof(unsigned int), "uint32_t/unsigned int mix up");    // why doesn't segger use uint32_t?

    if (rtt_cb < TARGET_RAM_START  ||  rtt_cb > TARGET_RAM_END - sizeof(EXT_SEGGER_RTT_CB_HEADER)) {
        return;
    }

    // do operations
    if (dap_is_connected()) {
        xTimerReset(timer_rtt_dap_interleave, 100);
    }
    rtt_console_running = true;
    probe_rtt_cb = true;
    ok = true;
    while (ok) { //  &&  !sw_unlock_requested()) {
        if (ok  &&  probe_rtt_cb) {
            //
            // check RTT control block and also all RTT channels
            //
//            printf("%8x %d %d %d %d %d\n", (unsigned int)rtt_cb, ok, probe_rtt_cb, working_uart, ok_console_from_target, ok_console_to_target);
            // did nothing -> check if RTT channels (dis)appeared
            ok = ok  &&  rtt_check_control_block_header(rtt_cb);
//            printf("xx %d\n", ok);
#if OPT_TARGET_UART
            if ( !ok_console_from_target)
                ok = ok  &&  rtt_check_channel_from_target(rtt_cb, RTT_CHANNEL_CONSOLE, &aUpConsole, &ok_console_from_target);
            if ( !ok_console_to_target)
                ok = ok  &&  rtt_check_channel_to_target(rtt_cb, RTT_CHANNEL_CONSOLE, &aDownConsole, &ok_console_to_target);
#endif
#if INCLUDE_SYSVIEW
            if ( !ok_sysview_from_target)
                ok = ok  &&  rtt_check_channel_from_target(rtt_cb, RTT_CHANNEL_SYSVIEW, &aUpSysView, &ok_sysview_from_target);
            if ( !ok_sysview_to_target)
                ok = ok  &&  rtt_check_channel_to_target(rtt_cb, RTT_CHANNEL_SYSVIEW, &aDownSysView, &ok_sysview_to_target);
#endif

            if ( !dap_is_connected()) {
                // -> delay (TODO what is it good for?)
                xEventGroupWaitBits(events, EV_RTT_TO_TARGET, pdTRUE, pdFALSE, pdMS_TO_TICKS(RTT_POLL_INT_MS));
            }
        }

        probe_rtt_cb = true;

        //
        // RTT console IO
        //
#if OPT_TARGET_UART
        {
#if INCLUDE_SYSVIEW
            static TickType_t lastTimeWorked;

            if ( !working_uart  &&  xTaskGetTickCount() - lastTimeWorked < pdMS_TO_TICKS(RTT_CONSOLE_POLL_INT_MS)) {
                //
                // pause console IO for a longer time to let SysView the interface
                //
            }
            else
#endif
            {
                working_uart = false;

                if (ok_console_from_target)
                    ok = ok  &&  rtt_from_target(&aUpConsole, cdc_uart_write, false, &working_uart);

                if (ok_console_to_target)
                    ok = ok  &&  rtt_to_target(&aDownConsole, stream_rtt_console_to_target, &working_uart);

                probe_rtt_cb = probe_rtt_cb  &&  !working_uart;

#if INCLUDE_SYSVIEW
                lastTimeWorked = xTaskGetTickCount();
#endif
            }
        }
#endif

#if INCLUDE_SYSVIEW
        //
        // RTT SysView IO
        //
        if (net_sysview_is_connected()) {
            if ( !net_sysview_was_connected) {
                net_sysview_was_connected = true;
                rtt_from_target_reset(&aUpSysView);
            }

            working_sysview = false;

            if (ok_sysview_from_target)
                ok = ok  &&  rtt_from_target(&aUpSysView, net_sysview_send, true, &working_sysview);

            if (ok_sysview_to_target)
                ok = ok  &&  rtt_to_target(&aDownSysView, stream_rtt_sysview_to_target, &working_sysview);

            probe_rtt_cb = probe_rtt_cb  &&  !working_sysview;
        }
        else {
            net_sysview_was_connected = false;
        }
#endif

        if (dap_is_connected()) {
            if (sw_unlock_requested()  &&  !xTimerIsTimerActive(timer_rtt_dap_interleave)) {
//                picoprobe_info("xx DAP timeout %d %d\n", sw_unlock_requested(), !xTimerIsTimerActive(timer_rtt_dap_interleave));
                ok = false;
            }
        }
        else {
            if (sw_unlock_requested()) {
//                picoprobe_info("xx unlock requested\n");
                ok = false;
            }
        }
    }
//    printf("ee\n");
    rtt_console_running = false;
    if (dap_is_connected()) {
        xTimerStop(timer_rtt_dap_interleave, 100);
    }
}   // do_rtt_io



static bool target_connect(void)
/**
 * Connect to the target, but let the target run
 * \return true -> connected to target
 */
{
    bool r = false;

//    picoprobe_debug("=================================== RTT connect target\n");
//    target_set_state(RESET_PROGRAM);
    if (target_set_state(ATTACH)) {
        r = true;
    }
    return r;
}   // target_connect



static void target_disconnect(void)
{
//    picoprobe_debug("=================================== RTT disconnect target\n");
//    target_set_state(RESET_RUN);
}   // target_disconnect



static void rtt_print_target_info(void)
/**
 * Print target board information
 */
{
    picoprobe_info("\n");
    picoprobe_info("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    picoprobe_info("Target vendor     : %s\n", g_board_info.target_cfg->target_vendor);
    picoprobe_info("Target part       : %s\n", g_board_info.target_cfg->target_part_number);

    if (g_board_info.target_cfg->flash_regions[0].start == g_board_info.target_cfg->flash_regions[0].end)
    {
        picoprobe_error("Flash             : NO FLASH DETECTED\n");
    }
    else
    {
        picoprobe_info("Flash             : 0x%08x..0x%08x (%uK)\n",
                      (unsigned)g_board_info.target_cfg->flash_regions[0]. start,
                      (unsigned)(g_board_info.target_cfg->flash_regions[0]. end - 1),
                      (unsigned)((g_board_info.target_cfg->flash_regions[0]. end -
                                 g_board_info.target_cfg->flash_regions[0]. start) / 1024));
    }

    picoprobe_info("RAM               : 0x%08x..0x%08x (%uK)\n",
                  (unsigned)g_board_info.target_cfg->ram_regions[0].start,
                  (unsigned)(g_board_info.target_cfg->ram_regions[0].end - 1),
                  (unsigned)((g_board_info.target_cfg->ram_regions[0].end -
                             g_board_info.target_cfg->ram_regions[0]. start) / 1024));
    picoprobe_info("SWD frequency     : %ukHz\n", (unsigned)probe_get_swclk_freq_khz());
    picoprobe_info("SWD max frequency : %ukHz\n", g_board_info.target_cfg->rt_max_swd_khz);
    picoprobe_info("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    picoprobe_info("\n");
}   // rtt_print_target_info



void rtt_io_thread(void *ptr)
/**
 * Do RTT input/output for console and SystemView.
 *
 * Experimental feature
 * --------------------
 * During gaps in the CMSIS-DAP command stream it is tried to continue RTT streaming.  This currently
 * seems to work for PyOCD but not for OpenOCD.
 * PyOCD:
 * - stops DAP communication after single steps
 * - during this pause, the probe jumps in and polls RTT communication
 * - on execution OyOCD poll the target, but there are big enough gaps for RTT
 * OpenOCD:
 * - even after single steps, OpenOCD continues to poll the target via DAP
 * - during (small) gaps in the communication the probe tries to do RTT access to the target.
 *   IT SEEMS THAT READING THE TARGET RESULTS IN NONSENSE, which means that e.g. rtt_check_control_block_header()
 *   returns !ok.  But sometimes (esp pressing pause during run or longer "step over") access seems to work.
 *   But this is not reliable.
 * - debugging seems to be disturbed because after a while a hardfault is thrown on the target
 * - TODO can OpenOCD behavior be changed via commands to omit polling?
 * --> no RTT while DAP
 * probe-rs:
 * - is very slow on DAP gdb debugging and gets slower if RTT while DAP is enabled
 * - anyway debugging with probe-rs (0.29.1) is no fun
 * --> no RTT while DAP
 */
{
    static uint32_t rtt_cb = 0;
    static bool rtt_cb_ok = false;
    bool target_online = false;

    for (;;) {
        if ( !target_online) {
            if (g_board_info.prerun_board_config != NULL) {
                g_board_info.prerun_board_config();
            }
            if (g_board_info.target_cfg->rt_board_id != NULL) {
                rtt_print_target_info();
            }
        }

        sw_lock(E_SWLOCK_RTT);
        // post: we have the interface

#if OPT_RTT_WHILE_DEBUGGING  // EXPERIMENTAL FEATURE
        if (dap_is_connected()) {
            //
            // Do RTT while debugging until a DAP command arrives
            // Note that dap_is_connected() must be caught here, because the code downwards disturbs debugging
            //
            do {
                uint32_t prev_rtt_cb;

                prev_rtt_cb = rtt_cb;
                if ( !search_for_rtt_cb( &rtt_cb)) {
                    if (rtt_cb_ok) {
                        rtt_cb_ok = false;
                        picoprobe_info("---- no RTT_CB found (RTT during DAP)\n");
                    }
                    break;
                }

                if (rtt_cb != prev_rtt_cb) {
                    rtt_cb_ok = true;
                    picoprobe_info("---- RTT_CB found at 0x%x (RTT during DAP)\n", (unsigned)rtt_cb);
                }

                do_rtt_io(rtt_cb);
            } while ( !sw_unlock_requested()  &&  is_target_ok(0));
            sw_unlock(E_SWLOCK_RTT);
            continue;
        }
#else
        if (dap_is_connected()) {
            picoprobe_error("RTT WHILE DEBUGGING SEEMS TO BE DISABLED.  WE SHOULD NEVER ARRIVEE HERE.\n");
            vTaskDelay(pdMS_TO_TICKS(100));
            sw_unlock(E_SWLOCK_RTT);
            continue;
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(100));            // give the target some time for startup
        if ( !target_connect()) {
            led_state(LS_NO_TARGET);

            if (target_online) {
                target_online = false;
                picoprobe_info("=================================== Target lost\n");
            }
            vTaskDelay(pdMS_TO_TICKS(900));
        }
        else {
            //
            // search for RTT_CB
            //
            uint32_t prev_rtt_cb;

            picoprobe_info("searching RTT_CB in 0x%08x..0x%08x, prev: 0x%08x\n",
                           (unsigned)TARGET_RAM_START, (unsigned)(TARGET_RAM_END - 1), (unsigned)rtt_cb);
            led_state(LS_TARGET_FOUND);
            target_online = true;

            prev_rtt_cb = rtt_cb;
            if ( !search_for_rtt_cb( &rtt_cb))
            {
                // -> no RTT_CB in memory
                if (rtt_cb_ok) {
                    rtt_cb_ok = false;
                    picoprobe_info("---- no RTT_CB found\n");
                }
                led_state(LS_TARGET_FOUND);
                vTaskDelay(pdMS_TO_TICKS(900));
            }
            else
            {
                // RTT_CB found
                if (rtt_cb != prev_rtt_cb) {
                    rtt_cb_ok = true;
                    picoprobe_info("---- RTT_CB found at 0x%x\n", (unsigned)rtt_cb);
                }
                led_state(LS_RTT_CB_FOUND);
                do_rtt_io(rtt_cb);
            }

            target_disconnect();
        }
        sw_unlock(E_SWLOCK_RTT);
        vTaskDelay(pdMS_TO_TICKS(100));            // give the other task the opportunity to catch sw_lock();
    }
}   // rtt_io_thread



bool rtt_console_cb_exists(void)
/**
 * Check if RTT console is available
 *
 * @return true if console RTT channel is active
 */
{
    return rtt_console_running  &&  ok_console_to_target;
}   // rtt_console_cb_exists



void rtt_send_byte(StreamBufferHandle_t stream, uint16_t channel, uint8_t ch, bool allow_drop)
/**
 * Write a byte into the RTT stream.
 * If there is no space left in the stream, wait 10ms and then try again.
 * If there is still no space, then drop a byte from the stream.
 */
{
    size_t available = xStreamBufferSpacesAvailable(stream);
    if ( !allow_drop  &&  available < sizeof(ch)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        available = xStreamBufferSpacesAvailable(stream);
        if (available < sizeof(ch)) {
            uint8_t dummy;
            xStreamBufferReceive(stream, &dummy, sizeof(dummy), 0);
            picoprobe_error("rtt_console_send_byte: drop byte on channel %d\n", channel);
        }
    }
    xStreamBufferSend(stream, &ch, sizeof(ch), 0);
    xEventGroupSetBits(events, EV_RTT_TO_TARGET);
}   // rtt_send_byte



void rtt_console_send_byte(uint8_t ch)
{
    rtt_send_byte(stream_rtt_console_to_target, RTT_CHANNEL_CONSOLE, ch, false);
}   // rtt_console_send_byte



#if INCLUDE_SYSVIEW
void rtt_sysview_send_byte(uint8_t ch)
/**
 * Send a byte to the SysView channel of the target
 */
{
    rtt_send_byte(stream_rtt_sysview_to_target, RTT_CHANNEL_SYSVIEW, ch, true);
}   // rtt_sysview_send_byte
#endif



void rtt_console_init(uint32_t task_prio)
{
    picoprobe_debug("rtt_console_init()\n");

    events = xEventGroupCreate();

    stream_rtt_console_to_target = xStreamBufferCreate(STREAM_RTT_SIZE, STREAM_RTT_TRIGGER);
    if (stream_rtt_console_to_target == NULL) {
        picoprobe_error("rtt_console_init: cannot create stream_rtt_console_to_target\n");
    }

#if INCLUDE_SYSVIEW
    stream_rtt_sysview_to_target = xStreamBufferCreate(STREAM_RTT_SIZE, STREAM_RTT_TRIGGER);
    if (stream_rtt_sysview_to_target == NULL) {
        picoprobe_error("rtt_console_init: cannot create stream_rtt_sysview_to_target\n");
    }
#endif

    timer_rtt_dap_interleave = xTimerCreate("RTT/DAP interleave timeout", pdMS_TO_TICKS(8), pdFALSE, NULL, rtt_cb_verify_timeout);

    xTaskCreate(rtt_io_thread, "RTT-IO", configMINIMAL_STACK_SIZE, NULL, task_prio, &task_rtt_console);
    if (task_rtt_console == NULL)
    {
        picoprobe_error("rtt_console_init: cannot create task_rtt_console\n");
    }

    xTaskCreate(rtt_from_target_thread, "RTT-From", configMINIMAL_STACK_SIZE, NULL, task_prio, &task_rtt_from_target_thread);
    if (task_rtt_from_target_thread == NULL)
    {
        picoprobe_error("rtt_console_init: cannot create task_rtt_from_target_thread\n");
    }
}   // rtt_console_init
