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

#include "FreeRTOS.h"
#include "task.h"

#include "picoprobe_config.h"
#include "rtt_console.h"
#include "sw_lock.h"


static TaskHandle_t           task_rtt_console = NULL;



void rtt_console_thread(void *ptr)
{
    static bool have_lock;

    for (;;) {
        if ( !have_lock) {
            have_lock = sw_lock("RTT", false);
        }

        if (have_lock) {
            // do operations
            vTaskDelay(pdMS_TO_TICKS(100));

            if (sw_unlock_requested()) {
                have_lock = false;
                sw_unlock("RTT");
            }
        }
        else {
            // wait until sw_lock is available again
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}   // rtt_console_thread



void rtt_console_init(uint32_t task_prio)
{
    picoprobe_debug("rtt_console_init\n");

    xTaskCreate(rtt_console_thread, "RTT_CONSOLE", configMINIMAL_STACK_SIZE, NULL, task_prio, &task_rtt_console);
}   // rtt_console_init