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

#if __GNUC__ >= 4
#define OFFSET_OF(TYPE, Field) ((UINTN)__builtin_offsetof(TYPE, Field))
#endif
///
/// Private worker functions for ASM_PFX()
///
#define _CONCATENATE(a, b)  __CONCATENATE(a, b)
#define __CONCATENATE(a, b) a ## b

///
/// The __USER_LABEL_PREFIX__ macro predefined by GNUC represents the prefix
/// on symbols in assembly language.
///
#define ASM_PFX(name) _CONCATENATE(__USER_LABEL_PREFIX__, name)

#pragma pack()

#define ASM_GLOBAL .globl

#define GCC_ASM_EXPORT(func__)	\
	.global _CONCATENATE(__USER_LABEL_PREFIX__, func__); \
	.type ASM_PFX(func__), % function

#define GCC_ASM_IMPORT(func__)	\
	.extern _CONCATENATE(__USER_LABEL_PREFIX__, func__)
