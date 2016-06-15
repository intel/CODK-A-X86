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

.global deep_sleep_resume_hook
.global deep_sleep_dump_context
.global halt_arc

.text

#ifdef CONFIG_DEEPSLEEP
.align 4

deep_sleep_dump_context:
	st      r0, [sp, 0] ; save r0 in the stack
	mov     r0, @_deep_sleep_registers ; load dump location addr in r0
	st      r1, [r0, 4] ; save all registers
	st      r2, [r0, 8]
	st      r3, [r0, 12]
	st      r4, [r0, 16]
	st      r5, [r0, 20]
	st      r6, [r0, 24]
	st      r7, [r0, 28]
	st      r8, [r0, 32]
	st      r9, [r0, 36]
	st      r10, [r0, 40]
	st      r11, [r0, 44]
	st      r12, [r0, 48]
	st      r13, [r0, 52]
	st      r14, [r0, 56]
	st      r15, [r0, 60]
	st      r16, [r0, 64]
	st      r17, [r0, 68]
	st      r18, [r0, 72]
	st      r19, [r0, 76]
	st      r20, [r0, 80]
	st      r21, [r0, 84]
	st      r22, [r0, 88]
	st      r23, [r0, 92]
	st      r24, [r0, 96]
	st      r25, [r0, 100]
	st      r26, [r0, 104]
	st      r27, [r0, 108]
	st      r28, [r0, 112]
	st      r29, [r0, 116]
	st      r30, [r0, 120]
	st      r31, [r0, 124]
	ld      r1, [sp, 0] ; get r0 which was in the stack
	st      r1, [r0, 0] ; save r0
#endif

.align 4
halt_arc:
	; Halt ARC core
	ld         r0,[0xb0800600] ; Read SS_CFG register
	bset_s     r0,r0,25        ; Set ARC_HALT_REQ_A bit
	bset_s     r0,r0,26        ; Set ARC_HALT_INT_REDIR bit (Halt Entry)
	bclr_s     r0,r0,24        ; Clear ARC_RUN_REQ_A bit
	st         r0,[0xb0800600] ; Write SS_CFG register back

.align 4
wait_for_run_req:
	ld         r0,[0xb0800600]   ; Read SS_CFG register
	and        r0, r0, 0x1000000 ; Check ARC_RUN_REQ_A bit (RUN_REQ_A from QRK)
	tst_s      r0, r0
	bz         wait_for_run_req

.align 4

#ifdef CONFIG_DEEPSLEEP
deep_sleep_resume_hook:
	clri ; disable interrupts
	mov     r0, @_deep_sleep_registers ; load dump location addr in r0
	ld      r1, [r0, 4] ; Restore all regiters
	ld      r2, [r0, 8]
	ld      r3, [r0, 12]
	ld      r4, [r0, 16]
	ld      r5, [r0, 20]
	ld      r6, [r0, 24]
	ld      r7, [r0, 28]
	ld      r8, [r0, 32]
	ld      r9, [r0, 36]
	ld      r10, [r0, 40]
	ld      r11, [r0, 44]
	ld      r12, [r0, 48]
	ld      r13, [r0, 52]
	ld      r14, [r0, 56]
	ld      r15, [r0, 60]
	ld      r16, [r0, 64]
	ld      r17, [r0, 68]
	ld      r18, [r0, 72]
	ld      r19, [r0, 76]
	ld      r20, [r0, 80]
	ld      r21, [r0, 84]
	ld      r22, [r0, 88]
	ld      r23, [r0, 92]
	ld      r24, [r0, 96]
	ld      r25, [r0, 100]
	ld      r26, [r0, 104]
	ld      r27, [r0, 108]
	ld      r28, [r0, 112]
	ld      r29, [r0, 116]
	ld      r30, [r0, 120]
	ld      r31, [r0, 124]
	ld      r0, [r0, 0] ; restore r0

	j_s [blink] ; go back to C code
#endif
