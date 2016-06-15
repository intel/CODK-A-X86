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

#include "machine/soc/quark_se/quark_se_mapping.h"
#include "machine/soc/quark_se/cos_interface.h"

ENTRY(main)

MEMORY
{
	/* Consider changing this to a PIC */
	esram (rw) : ORIGIN = COS_STARTUP, LENGTH = COS_SIZE
}

SECTIONS
{
	. = ORIGIN(esram);
	. = ALIGN(4);

	.text :
	{
		*(.text.entry)
		KEEP(*(.text.entry*))
		*(.text)
		*(.text.last)
		*(.text.*)
		*(.rodata)
		*(.rodata.*)
		*(.rdata)
		*(.rdata.*)
		*(.data)
		*(.data.*)
		. = ALIGN(4);
		__bss_start = .;
		*(.bss)
		*(.bss.*)
		*(COMMON)
		__bss_end = .;
	} > esram
		. = ALIGN(4);
		_bss_end = .;
	._cos_stack :
	{
		 *(._cos_stack.*)
	} > esram

	_image_end = .;

 .comment 0 : { *(.comment) }

	ASSERT( !(_image_end % 4), "image not 4 bytes aligned")
}
