/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Hardy Griech
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

/**
 * This module holds some additional SWD functions.
 */

#ifndef _TARGET_RP2040_H
#define _TARGET_RP2040_H


#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
    extern "C" {
#endif


// required for linking of \a g_board_info.target_cfg and \a g_raspberry_rp2040_family
#define TARGET_RP2040_FAMILY_ID       CREATE_FAMILY_ID(hRPi_VendorID, 1)
#define TARGET_RP2350_FAMILY_ID       CREATE_FAMILY_ID(hRPi_VendorID, 2)


// pre: flash connected, post: generic XIP active
#define RP2xxx_FLASH_RANGE_ERASE(OFFS, CNT, BLKSIZE, CMD)           \
    do {                                                            \
        _flash_exit_xip();                                          \
        _flash_range_erase((OFFS), (CNT), (BLKSIZE), (CMD));        \
        _flash_flush_cache();                                       \
        _flash_enter_cmd_xip();                                     \
    } while (0)

// pre: flash connected, post: generic XIP active
#define RP2xxx_FLASH_RANGE_PROGRAM(ADDR, DATA, LEN)                 \
    do {                                                            \
        _flash_exit_xip();                                          \
        _flash_range_program((ADDR), (DATA), (LEN));                \
        _flash_flush_cache();                                       \
        _flash_enter_cmd_xip();                                     \
    } while (0)


#define rom_hword_as_ptr(rom_address) (void *)(uintptr_t)(*(uint16_t *)rom_address)
#define ROM_FN(a, b)                  (uint32_t)((b << 8) | a)

typedef void *(*rp2040_rom_table_lookup_fn)(uint16_t *table, uint32_t code);
typedef void *(*rp2350_rom_table_lookup_fn)(uint32_t code, uint32_t mask);

typedef int   (*rp2350_rom_get_sys_info_fn)(uint32_t *out_buffer, uint32_t out_buffer_word_size, uint32_t flags);
typedef void  (*rp2350_rom_connect_internal_flash_fn)(void);

typedef void *(*rp2xxx_rom_void_fn)(void);
typedef void *(*rp2xxx_rom_flash_erase_fn)(uint32_t addr, size_t count, uint32_t block_size, uint8_t block_cmd);
typedef void *(*rp2xxx_rom_flash_prog_fn)(uint32_t addr, const uint8_t *data, size_t count);


bool target_core_is_halted(void);
bool target_core_halt(void);
bool target_core_unhalt(void);
bool target_core_unhalt_with_masked_ints(void);


#ifdef __cplusplus
    }
#endif

#endif
