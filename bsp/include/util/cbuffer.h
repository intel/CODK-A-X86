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

#ifndef __CBUFFER_H
#define __CBUFFER_H

#include <stdint.h>

typedef struct cbuffer {
    uint32_t r;
    uint32_t w;
    uint8_t saturation_flag;
    uint8_t *buf;
    uint32_t buf_size;
} cbuffer_t;

/**
 * Used to set up the circular buffer, if special values are necessary.
 *
 * @param c  cbuffer to initialize
 *
 * @return -1  if the buffer size is not a power of 2,
 *          0  if no error
 */
int32_t cb_init(cbuffer_t *c);

/**
 * Write to a cbuffer.
 *
 * @param dst     cbuffer in which to write
 * @param src     Pointer to data source
 * @param length  How many bytes to write
 *
 * @return -1  if bad length,
 *          0  if no error,
 *          1  if an override occurs
 */
int32_t cb_push(cbuffer_t *dst, const uint8_t *src, const uint8_t length);

/**
 * Read from a cbuffer, starting from an offset.
 *
 * @param src           cbuffer from which to read
 * @param offset        Position in buffer where to start the write
 * @param dst           Pointer to destination
 * @param length        How many bytes to read
 *
 * @return -1  if bad length
 *          0  if no message found
 *          1  if no error
 */
int32_t cb_pop(cbuffer_t *src, const uint32_t offset, uint8_t *dst, const uint8_t length);

/**
 * Search for a char in the buffer. Used to search for a marker.
 *
 * @param byte   char to be searched
 * @param src    cbuffer in which to search
 * @param start  position in buffer where to start the search
 * @param stop   stop if we reach this index (but check it too)
 * @param cnt    max number of characters to search
 *
 * @return -2 if error, -1 if not found; else index in cbuffer buf where first found
 */
int32_t cb_find(const uint8_t byte, const cbuffer_t *src, const uint32_t start, const uint32_t stop, const uint32_t cnt);
#endif /* __CBUFFER_H */
