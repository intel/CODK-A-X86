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

#ifndef __MISC_H__
#define __MISC_H__

/* required for offsetof */
#include <stddef.h>

#include "util/compiler.h"

/** Generate a build time error if a condition is met
 *
 * @param condition The condition to be tested
 *
 * @note This function should be used inside a code block
 */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

/** Generate a build time error if a condition is met or return 0
 *
 * @param condition The condition to be tested
 *
 * @return 0
 */
#define BUILD_BUG_ON_ZERO(e) (sizeof(char[1 - 2 * !!(e)]) - 1)

/** Generate a build time error if the parameter is not an array otherwise return 0
 *
 * @param   A   The array to be checked.
 *
 * @return 0
 */
#define __must_be_array(A) \
	BUILD_BUG_ON_ZERO(HAVE_SAME_TYPE(A, &A[0]))

/**
 * Check if the integer provided is a power of two.
 *
 * @param   A   Number to be tested.
 *
 * @return      true if value is power of two else false.
 */
#define IS_POWER_OF_TWO(A)  ((A) != 0 && (((A) - 1) & (A)) == 0)

#define container_of(ptr, type, member) ({\
    (type *)( (char *)(ptr) - offsetof(type,member) );})

/**
 * Compute the number of elements in an array.
 *
 * @param   A   The array to be checked.
 *
 * @return      The number of elements in the array.
 */
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0])) + __must_be_array(A)

#endif  /* __MISC_H__ */
