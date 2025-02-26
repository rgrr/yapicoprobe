/**
 * @file    flash_blob.c
 * @brief   Flash algorithm for the rPi Pico family
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2021, Arm Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// TODO ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION
// TODO ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION
// TODO ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION
// TODO ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION
// TODO ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION
// TODO ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION
//
// This blob does not work and is for future usage only
//

#include "flash_blob.h"

// blob stolen from https://github.com/pyocd/pyOCD/blob/main/pyocd/target/builtin/target_RP2040.py
static const uint32_t rp2040_FLM[] = {
        0xe00abe00,
        0xb085b5f0, 0x447e4e1e, 0x28017830, 0xf000d101, 0x2001f839, 0x70309004, 0x46384f15, 0xf00030f7,
        0x4604f8a3, 0x1c804813, 0xf89ef000, 0x46384605, 0xf89af000, 0x48109003, 0xf896f000, 0x480f9002,
        0xf892f000, 0x480b9001, 0xf88ef000, 0x47a04607, 0x607447a8, 0x980360b5, 0x980260f0, 0x98016130,
        0x61b76170, 0x70309804, 0xb0052000, 0x46c0bdf0, 0x00004552, 0x00005843, 0x00005052, 0x00004346,
        0x0000027a, 0x4c07b510, 0x7820447c, 0xd1062801, 0x47806960, 0x478069a0, 0x70202000, 0x2001bd10,
        0x46c0bd10, 0x000001f8, 0x44784805, 0x28007800, 0x2001d101, 0x48014770, 0x46c04770, 0x000070d0,
        0x000001d6, 0x4601b570, 0x447a4a0e, 0x28017810, 0x2301d10e, 0x2400071d, 0x46261b48, 0x42a94166,
        0x68d5d308, 0x041a0319, 0x47a823d8, 0xbd704620, 0xbd702001, 0x44784804, 0x4a042121, 0xf000447a,
        0x46c0f855, 0x000001b6, 0x00000126, 0x00000164, 0xb081b5f0, 0x4d10460b, 0x7829447d, 0xd10f2901,
        0x070e2101, 0x1b812400, 0x41674627, 0xd30b42b0, 0x4608692d, 0x461a4611, 0x462047a8, 0xbdf0b001,
        0x46202401, 0xbdf0b001, 0x44784804, 0x4a042121, 0xf000447a, 0x46c0f82b, 0x00000168, 0x000000d2,
        0x00000120, 0xd4d4de00, 0x2114b280, 0x1e898809, 0x2a00884a, 0x1d09d004, 0xd1f94282, 0x47708808,
        0x44784803, 0x4a03210e, 0xf000447a, 0x46c0f80f, 0x00000076, 0x000000c8, 0xd4d44770, 0x49024801,
        0x46c04770, 0x4d94efcf, 0x7847d224, 0xaf00b580, 0x2300b088, 0x4c079305, 0x93039404, 0x23019302,
        0xab069301, 0x91079300, 0x46689006, 0xf0004611, 0xdefef803, 0x00000244, 0xaf00b580, 0x9103b084,
        0x48049002, 0x48049001, 0x46689000, 0xffbaf7ff, 0x46c0defe, 0x00000244, 0x00000244, 0x636e7546,
        0x746f6e20, 0x756f6620, 0x7273646e, 0x616d2f63, 0x722e6e69, 0xd4d4d473, 0xd4d4d4d4, 0xd4d4d4d4,
        0x65747461, 0x2074706d, 0x73206f74, 0x72746275, 0x20746361, 0x68746977, 0x65766f20, 0x6f6c6672,
        0xd4d4d477, 0x00000199, 0x00000000, 0x00000001, 0x0000019d, 0x0000020a, 0x0000000b, 0x00000014,
        0x00000011, 0x0000020a, 0x0000000b, 0x00000053, 0x00000028, 0x0000020a, 0x0000000b, 0x00000058,
        0x0000002a, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

// This flash algo should work with all members of the rPi Pico family
// blob stolen from https://github.com/pyocd/pyOCD/blob/develop/pyocd/target/family/target_rp2.py
static const uint32_t rp2350_FLM[] = {
        0xe7fdbe00,
        0xaf03b5f0, 0x4614b087, 0x447d4d5a, 0x28007828, 0x485ed007, 0x68004478, 0x485d4780, 0x68004478,
        0x26014780, 0x1e60702e, 0xd3002803, 0x4848e08d, 0xf90ef000, 0x2800460c, 0x4846d15f, 0xf0001c80,
        0x2800f907, 0x460cd001, 0x9406e057, 0x78102210, 0x284d4c41, 0x2311d151, 0x28757818, 0x2012d14d,
        0x78009002, 0xd00a2802, 0xd1462801, 0x92049303, 0x20149105, 0x21188800, 0x4938880a, 0x9303e008,
        0x91059204, 0xf8d4f000, 0x88022016, 0x21044833, 0x4c334790, 0x46222800, 0x4602d000, 0x99052800,
        0x9b039804, 0x7800d029, 0x284d4c2e, 0x7818d125, 0xd1222875, 0x78009802, 0x91052802, 0xd0079201,
        0xd11a2801, 0x88002014, 0x880a2118, 0xe0054926, 0xf8aef000, 0x88022016, 0x21044823, 0x4c234790,
        0x46222800, 0x4602d000, 0xd0062800, 0x48209204, 0xf8aef000, 0x2800460c, 0x4620d002, 0xbdf0b007,
        0xf0004814, 0x2800f8a5, 0x9103d19d, 0x28009806, 0x9806d019, 0x98054780, 0x48174780, 0x99014478,
        0x48166001, 0x99064478, 0x48156001, 0x99044478, 0x48146001, 0x60044478, 0x44784813, 0x60019903,
        0x2400702e, 0x9c05e7d9, 0xf000e7d7, 0x46c0f8ab, 0x00004649, 0x00005843, 0x10004552, 0x00004552,
        0x20004552, 0x10005052, 0x00005052, 0x20005052, 0x00004346, 0x0000029e, 0x00000194, 0x00000188,
        0x00000188, 0x00000184, 0x00000182, 0x000002a4, 0x000002a0, 0xaf02b5d0, 0x447c4c08, 0x28017820,
        0x4807d10a, 0x68004478, 0x48064780, 0x68004478, 0x20004780, 0xbdd07020, 0xbdd02001, 0x0000010e,
        0x00000114, 0x00000110, 0xaf02b5d0, 0x44794909, 0x29017809, 0x210fd10c, 0x18400709, 0x44794906,
        0x2201680c, 0x04120311, 0x47a023d8, 0xbdd02000, 0xbdd02001, 0x000000da, 0x000000d2, 0xaf02b5d0,
        0x4909460b, 0x78094479, 0xd10a2901, 0x0709210f, 0x49061840, 0x680c4479, 0x461a4611, 0x200047a0,
        0x2001bdd0, 0x46c0bdd0, 0x000000a4, 0x000000a0, 0xf45f4806, 0x60014140, 0xf710ee30, 0xec40d404,
        0xec400780, 0xbf400781, 0x00004770, 0xe000ed88, 0xaf02b5d0, 0x2010b284, 0x284d7800, 0x2011d10f,
        0x28757800, 0x2012d10b, 0x28027800, 0x2801d00b, 0x2014d105, 0x21188800, 0x4621880a, 0x2001e009,
        0x18610701, 0xf7ffbdd0, 0x2016ffd3, 0x21048802, 0x47904620, 0x42404601, 0x29004148, 0x2101d1f2,
        0xe7ee0749, 0xaf00b580, 0xdefede00, 0xd4d4d400, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000
};

/**
* List of start and size for each size of flash sector
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address start + size.
*/
static const sector_info_t sectors_info_rp2040[] = {
    {0, 256},
};

static const sector_info_t sectors_info_rp2350[] = {
    {0, 256},
};


static const program_target_t flash_rp2040 = {
    .init         = 0x20000005,
    .uninit       = 0x20000089,
    .erase_chip   = 0x0,
    .erase_sector = 0x200000c9,
    .program_page = 0x20000115,
    .verify       = 0x0,
    {
        .breakpoint    = 0x20000001,
        .static_base   = 0x20000000 + 0x00000004 + 0x00000254,
        .stack_pointer = 0x20002000
    },
    .program_buffer      = 0x20002000,
    .program_buffer_size = 512, // should be USBD_MSC_BlockSize
    .algo_start          = 0x20000000,
    .algo_size           = sizeof(rp2040_FLM),
    .algo_blob           = rp2040_FLM,
    .algo_flags          = kAlgoVerifyReturnsAddress,
};

static const program_target_t flash_rp2350 = {
    .init         = 0x20000005,
    .uninit       = 0x20000199,
    .erase_chip   = 0x0,
    .erase_sector = 0x200001cd,
    .program_page = 0x20000201,
    .verify       = 0x0,
    {
        .breakpoint    = 0x20000001,
        .static_base   = 0x20000000 + 0x00000004 + 0x000002c4,
        .stack_pointer = 0x200020d0
    },
    .program_buffer      = 0x20002000,
    .program_buffer_size = 512, // should be USBD_MSC_BlockSize
    .algo_start          = 0x20000000,
    .algo_size           = sizeof(rp2350_FLM),
    .algo_blob           = rp2350_FLM,
    .algo_flags          = 0,
};
