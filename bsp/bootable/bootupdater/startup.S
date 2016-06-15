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

	/* externs */
	.extern main

	.global __start

.func __start
__start:

	/* clear interrupt flag */
	cli

	/* processor is executing in 32-bit protected mode */

	.balign 16,0x90

	movl $__StackTop, %esp

	/* No frame yet. */
	movl %esp, %ebp

	/* clear out BSS double words (32-bits at a time) */
	xorl	%eax, %eax		/* fill memory with 0 */
	movl	$__bss_start, %edi	/* load BSS start address */
	movl	$__bss_num_words, %ecx	/* number of quad bytes */
	cld
	rep
	stosl				/* zero memory per 4 bytes */

	/* copy initialised variables */
	movl $__data_size, %ecx
	shr $2, %ecx
	movl $__data_vma, %edi
	movl $__data_lma, %esi
	cld
	rep movsl

	/* Ensure no interrupts are pending */
	mov    $0xff,%eax
.L1 :
	dec    %eax
	movl   $0x0,0xfee000b0
	jne    .L1

	/* jump to main */
	call   main

	/* Should not get here. */
	jmp .
.endfunc

.end
