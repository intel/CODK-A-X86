/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PANIC_QRK_SE_H
#define PANIC_QRK_SE_H

#include "infra/panic.h"

enum {
    QRK_CORE,    /*!< QRK core */
    ARC_CORE,    /*!< ARC core */
    NORDIC_CORE, /*!< Nordic core */
    QRK_SE_CORE_COUNT
} QRK_SE_CORE;

/**
 * \brief QRK specific panic data
 *
 * Do not edit, it must match QRK panic processing stubs
 */
struct x86_panic_arch_data {
    // Panic type (not a register)
    uint32_t type;
    uint32_t cr2;
    // Dump callee saved registers
    uint32_t ebp;
    uint32_t ebx;
    uint32_t esi;
    uint32_t edi;
    // Dump stack registers
    uint32_t esp;
    uint32_t ss;
    // Dump caller saved registers
    uint32_t edx;
    uint32_t ecx;
    uint32_t eax;
    // Data pushed by cpu during exception
    uint32_t error;
    uint32_t eip;
    uint32_t cs;
    uint32_t flags;
    uint32_t priv_esp;
    uint32_t priv_ss;
};
DECLARE_PANIC_DATA(x86);

/**
 * \brief ARC specific panic data
 *
 * Do not edit, it must match ARC panic processing stubs
 */
struct arcv2_panic_arch_data {
    uint32_t ecr;
    uint32_t efa;
    uint32_t eret;
    uint32_t erstatus;
    uint32_t registers[26];
    uint32_t gp;
    uint32_t fp;
    uint32_t sp;
    uint32_t il;
    uint32_t r30;
    uint32_t bl;
};
DECLARE_PANIC_DATA(arcv2);

#endif /* PANIC_QRK_SE_H */
