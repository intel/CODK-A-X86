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

#ifndef __COMPILER_H
#error "Please don't include compiler-gcc.h directly, include compiler.h instead."
#endif

/*--------------- GCC specific builtins ---------------*/
#define HAVE_SAME_TYPE(a,b) __builtin_types_compatible_p(__typeof__(a),__typeof__(b))

/*--------------- Function inlining control ---------------*/

/* NOINLINE is to instruct the compiler to never inline the requested function.
  Syntax : NOINLINE int foo(int a) { ... } */
#define NOINLINE         __attribute__((noinline))

/*--------------- Function and variable control ---------------*/

/* MAYBE_UNUSED is for variables, for function arguments and for functions
   which may or may not be used depending on macro preprocessing.
   This macro can be used to safely fix warning of type "parameter unused".
  Syntax :
   - for fct parameter : int foo(int MAYBE_UNUSED a) { ... }
   - for function : int MAYBE_UNUSED foo(int a);
   - for variable : int foo(int a) { int MAYBE_UNUSED b; ....}
   - for pointer  : int foo(int a) { int MAYBE_UNUSED * b; ....} */
#define MAYBE_UNUSED     __attribute__((unused))


/*--------------- Function general control ---------------*/

/* DEPRECATED is to inform that the corresponding function is now deprecated.
   This is the only warning that will not break a build.
  Syntax : int DEPRECATED foo(int a); */
#define DEPRECATED        __attribute__((deprecated))

/* NORETURN is to inform that the function is never returning.
  Syntax  : int NORETURN foo(int a); */
#define NORETURN          __attribute__((noreturn))


/*--------------- Code and variable relocation ---------------*/

/* DATA_SECTION is to place the corresponding variable into a secific
   section that will be placed later on by the linker file.
  Syntax : int DATA_SECTION(section_name) a; */
#define DATA_SECTION(x)    __attribute__((section((#x))))

/* CODE_SECTION is the doing the same as DATA_SECTION but for function.
  Syntax : int CODE_SECTION(section_name) foo(int a); */
#define CODE_SECTION(x)    __attribute__((section((#x))))

/* ALIGN is to specify a particular byte alignement for the specified variable.
  Syntax : int ALIGN(128) a; */
#define ALIGN(x)           __attribute__((aligned((x))))

/* PACKED attached to struct or union type definition, specifies that each member (other than
 * zero-width bit-fields) of the structure or union is placed to minimize the memory required.
 * When attached to an enum definition, it indicates that the smallest integral type should be
 * used.
 */
#define _PACKED __attribute__((__packed__))


/*--------------- Optimization barrier ---------------*/
/* BARRIER defines a memory barrier.
   The "volatile" is due to gcc bugs. */
#define BARRIER() __asm__ __volatile__("": : :"memory")


#ifdef __CPU_ARC__
#define _Usually(x) __builtin_expect(!!((x)), 1)
#define _Rarely(x) __builtin_expect(!!((x)), 0)
#define _sr(_src_, _reg_) __builtin_arc_sr((unsigned int)_src_, _reg_)
#define _lr(_reg_) __builtin_arc_lr(_reg_)
#define _nop() 
#endif
