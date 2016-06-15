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

.global panic
.global _Fault

#define AUX_INTERRUPT_CAUSE             0x40a
#define AUX_EXECUTION_RET               0x400
#define AUX_EXECUTION_STATUS            0x402
#define AUX_EXCEPTION_CAUSE             0x403
#define AUX_EXECUTION_ADDR              0x404

.text

.balign 4
panic:
    st sp, [saved_stack_pointer]
    sub_s  sp,sp,4
    st     r0, [sp, 0]

.balign 4
_Fault:
    ; ZephyrOS already dump caller-saved regs, lp_count and aux regs in stack
    ; we do it again here, instead of "memcpy" the stack
    ; we just use r0, which is at the bottom of stack
    mov r0, arcv2_panic_data
    ld r0,  [r0]
    st      r1, [r0, 16+4*1]

    lr      r1, [AUX_EXCEPTION_CAUSE]
    st      r1, [r0, 0]
    lr      r1, [AUX_EXECUTION_ADDR]
    st      r1, [r0, 4]
    lr      r1, [AUX_EXECUTION_RET]
    st      r1, [r0, 8]
    lr      r1, [AUX_EXECUTION_STATUS]
    st      r1, [r0, 12]

    st      r2, [r0, 16+4*2]
    st      r3, [r0, 16+4*3]
    st      r4, [r0, 16+4*4]
    st      r5, [r0, 16+4*5]
    st      r6, [r0, 16+4*6]
    st      r7, [r0, 16+4*7]
    st      r8, [r0, 16+4*8]
    st      r9, [r0, 16+4*9]
    st      r10, [r0, 16+4*10]
    st      r11, [r0, 16+4*11]
    st      r12, [r0, 16+4*12]
    st      r13, [r0, 16+4*13]
    st      r14, [r0, 16+4*14]
    st      r15, [r0, 16+4*15]
    st      r16, [r0, 16+4*16]
    st      r17, [r0, 16+4*17]
    st      r18, [r0, 16+4*18]
    st      r19, [r0, 16+4*19]
    st      r20, [r0, 16+4*20]
    st      r21, [r0, 16+4*21]
    st      r22, [r0, 16+4*22]
    st      r23, [r0, 16+4*23]
    st      r24, [r0, 16+4*24]
    st      r25, [r0, 16+4*25]
    st      r26, [r0, 16+4*26]
    st      r27, [r0, 16+4*27]
    ld      r1, [saved_stack_pointer]
    st      r1, [r0, 16+4*28]
    st      r29, [r0, 16+4*29]
    st      r30, [r0, 16+4*30]
    st      r31, [r0, 16+4*31]
    ld      r1, [sp, 0]
    st      r1, [r0, 16]
    j panic_handler
