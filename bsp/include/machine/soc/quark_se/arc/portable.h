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

#ifndef __PORTABLE_H__
#define __PORTABLE_H__

#include "os/os.h"
#include "machine/soc/quark_se/scss_registers.h"
#ifdef CONFIG_OS_ZEPHYR
#ifdef CONFIG_QEMU_X86
#include <arch/x86/arch.h>
#else
#include <arch/arc/arch.h>
#endif
#endif

#define DECLARE_INTERRUPT_HANDLER
#define SET_INTERRUPT_HANDLER(_vec_, _isr_) \
    do { \
        OS_ERR_TYPE err = E_OS_ERR; \
        interrupt_set_isr((_vec_), (_isr_), NULL , 1, &err); \
        interrupt_enable((_vec_)); \
    } while(0)

#ifdef CONFIG_QEMU_X86
#define __builtin_arc_lr(reg) MMIO_REG_VAL((reg))
#define __builtin_arc_sr(value, reg) ((MMIO_REG_VAL(reg)) = (value))
#endif

#define WRITE_ARC_REG(value, reg) \
    __builtin_arc_sr(value, (volatile uint32_t)(reg))

#define READ_ARC_REG(reg) \
    __builtin_arc_lr((volatile uint32_t)(reg))

#define CLEAR_ARC_BIT(reg, bit) \
 do { \
    uint32_t saved = interrupt_lock(); \
    WRITE_ARC_REG(READ_ARC_REG(reg) & ~(1 << bit), reg); \
    interrupt_unlock(saved); \
 } while(0)

#define SET_ARC_BIT(reg, bit) \
 do { \
    uint32_t saved = interrupt_lock(); \
    WRITE_ARC_REG(READ_ARC_REG(reg) | (1 << bit), reg); \
    interrupt_unlock(saved); \
 } while(0)

#define CLEAR_MMIO_BIT(reg, bit) \
 do { \
    uint32_t saved = interrupt_lock(); \
    MMIO_REG_VAL_FROM_BASE(reg, 0) &= ~(1 << bit); \
    interrupt_unlock(saved); \
 } while(0)

#define SET_MMIO_BIT(reg, bit) \
 do { \
    uint32_t saved = interrupt_lock(); \
    MMIO_REG_VAL_FROM_BASE(reg, 0) |= (1 << bit); \
    interrupt_unlock(saved); \
 } while(0)

#endif
