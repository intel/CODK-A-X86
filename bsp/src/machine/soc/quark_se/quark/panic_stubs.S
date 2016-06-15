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

#define _ASMLANGUAGE

#include <arch/x86/asm.h>

GTEXT(_panic_dump_reg)
GTEXT(panic)
#ifdef CONFIG_PANIC_ON_BUS_ERROR
GTEXT(_pmuIntStub)
#endif

.macro process_panic_trap id
	call    _panic_dump_reg /* dump core context */
	movl    $\id, -40(%edi) /* push exception id */
	call    panic_trap      /* call exception handler */
.endm

.macro declare_panic_trap trap, id
	GTEXT(\trap)
	SECTION_FUNC(TEXT, \trap)
	pushl   $0              /* Push default error code */
	process_panic_trap \id
.endm

.macro declare_panic_trap_no_err trap, id
	GTEXT(\trap)
	SECTION_FUNC(TEXT, \trap)
	process_panic_trap \id
.endm

declare_panic_trap _divTrapStub, 0
declare_panic_trap _nmiTrapStub, 2
declare_panic_trap _overflowTrapStub, 4
declare_panic_trap _boundTrapStub, 5
declare_panic_trap _invOptTrapStub, 6
declare_panic_trap _devNotAvailTrapStub, 7
declare_panic_trap _floatTrapStub, 16

declare_panic_trap_no_err _dblFaultTrapStub, 8
declare_panic_trap_no_err _tssTrapStub, 10
declare_panic_trap_no_err _segTrapStub, 11
declare_panic_trap_no_err _stackTrapStub, 12
declare_panic_trap_no_err _gpfTrapStub, 13
declare_panic_trap_no_err _pageTrapStub, 14
declare_panic_trap_no_err _alignTrapStub, 17

SECTION_FUNC(TEXT, panic)
	/* stack contains error code and return addr */
	/* FIXME: return addr is not removed so it will be in stack dump */
	pushf                    /* push flags */
	pushl   %cs              /* save cs */
	pushl   0x8(%esp)        /* save eip */
	pushl   0x10(%esp)       /* save user panic error code */
	process_panic_trap 0x100 /* use user panic error id */

SECTION_FUNC(TEXT, _panic_dump_reg)
	sti                             /* Disable interrupts */
	pushl  %edi                     /* push edi */
	movl   _x86_dump_location, %edi /* load panic dump location addr */
	subl   $4, %edi                 /* get first dump dword address */
	movl   %eax, -24(%edi)          /* save eax */
	movl   %ecx, -28(%edi)          /* save ecx */
	movl   %edx, -32(%edi)          /* save edx */
	movl   %ss, %ecx
	movl   %ecx, -36(%edi)          /* save ss */
	lea    24(%esp), %ecx
	movl   %ecx, -40(%edi)          /* save esp */
	popl   -44(%edi)                /* save edi */
	movl   %esi, -48(%edi)          /* save esi */
	movl   %ebx, -52(%edi)          /* save ebx */
	movl   %ebp, -56(%edi)          /* save ebp */
	movl   %cr2, %ecx
	movl   %ecx, -60(%edi)          /* save cr2 */

	movl   $6, %ecx                 /* we need to save 6 dwords */
	                                /* error, eip, cs, flag, user ss/esp */
	lea    24(%esp), %esi           /* load stack panic data */
	std                             /* set direction flag */
	rep    movsd                    /* copy hardware exception dump in RAM */
	                                /* edi is now pointing to eax */
	movl   (%esp), %eax             /* save return addr to panic stub in eax */
	jmp    *%eax                    /* dump done, return */

#ifdef CONFIG_PANIC_ON_BUS_ERROR
SECTION_FUNC(TEXT, _pmuIntStub)
	/* stack contains error code and return addr */
	/* FIXME: return addr is not removed so it will be in stack dump */
	pushf                    /* push flags */
	pushl   %cs              /* save cs */
	pushl   0x8(%esp)        /* save eip */
	pushl   0xC(%esp)        /* push eip as error code */
	process_panic_trap 0x14  /* use user panic error id */
#endif /* CONFIG_PANIC_ON_BUS_ERROR */
