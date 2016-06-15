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

#include <string.h>
#include "util/cbuffer.h"
#include "util/misc.h"

static void cb_read(const cbuffer_t *src, const uint32_t offset, uint8_t *dst, const uint8_t length);
static void cb_write(cbuffer_t *dst, const uint32_t offset, const uint8_t *src, const uint8_t length);

int32_t cb_init(cbuffer_t *c)
{
    if (IS_POWER_OF_TWO(c->buf_size)) {
        memset(c->buf, 0, c->buf_size);
        return 0;
    } else {
        return -1;
    }
}


int32_t cb_find(const uint8_t byte, const cbuffer_t *src, const uint32_t start, const uint32_t stop, const uint32_t cnt)
{
    uint32_t l_start = start;
    uint32_t l_stop = stop;
    uint32_t l_cnt = cnt;

    if (start >= CONFIG_LOG_CBUFFER_SIZE || stop >= CONFIG_LOG_CBUFFER_SIZE || cnt == 0)
        return -2;

    /* start = stop => search all cbuffer */
    if (l_start == l_stop) {
        if (l_start == 0)
            l_stop = CONFIG_LOG_CBUFFER_SIZE - 1;
        else
            l_stop = l_start - 1;
    }

    while (l_start != l_stop && l_cnt > 0) {
        if (src->buf[l_start] == byte) {
            return l_start;
        }
        l_start = (l_start + 1) & (CONFIG_LOG_CBUFFER_SIZE - 1);
        --l_cnt;
    }

    return -1;
}


int32_t cb_push(cbuffer_t *dst, const uint8_t *src, const uint8_t length)
{
    uint32_t current_w;

    if ( (length > CONFIG_LOG_CBUFFER_SIZE) || (length <= 0) ) {
        return -1;
    }

    current_w = dst->w;
    if ( ((dst->r - dst->w - 1) & (CONFIG_LOG_CBUFFER_SIZE - 1)) < length ) {
        dst->r += length - ((dst->r - dst->w - 1) & (CONFIG_LOG_CBUFFER_SIZE - 1));
        dst->r &= (CONFIG_LOG_CBUFFER_SIZE - 1);
        dst->saturation_flag = 1;
    }
    dst->w = (dst->w + length) & (CONFIG_LOG_CBUFFER_SIZE - 1);

    cb_write(dst, current_w, src, length);

    return 0;
}


int32_t cb_pop(cbuffer_t *src, const uint32_t offset, uint8_t *dst, const uint8_t length)
{
    uint32_t next_r;

    if ( (length > CONFIG_LOG_CBUFFER_SIZE) || (length <= 0) ) {
        return -1;
    }

    next_r = (offset + length) & (CONFIG_LOG_CBUFFER_SIZE -1);

    if ( ((offset < src->w) && (next_r > src->w) && (next_r > offset)) ||
         ((offset > src->w) && (next_r > src->w) && (next_r < offset))) {
        src->r = src->w;
    } else {
        src->r = next_r;
    }

    src->saturation_flag = 0;

    cb_read(src, offset, dst, length);

    return 1;
}


/**
 * Read from a cbuffer, starting from an offset. Pointers are not changed.
 *
 * src     cbuffer from which to read
 * offset  Position in buffer where to start the read
 * dst     Pointer to destination
 * length  How many bytes to read
 */
static void cb_read(const cbuffer_t *src, const uint32_t offset, uint8_t *dst, const uint8_t length)
{
    if (length > (CONFIG_LOG_CBUFFER_SIZE - offset)) {
        memcpy(&dst[0], &src->buf[offset], (CONFIG_LOG_CBUFFER_SIZE - offset));
        memcpy(&dst[(CONFIG_LOG_CBUFFER_SIZE - offset)], &src->buf[0],  (length - (CONFIG_LOG_CBUFFER_SIZE - offset)));
    } else {
        memcpy(dst, &src->buf[offset], length);
    }
}


/**
 * Write to a cbuffer, starting from an offset. Pointers are not changed.
 *
 * dst         cbuffer in which to write
 * offset      Position in buffer where to start the write
 * src         Pointer to data source
 * length      How many bytes to write
 */
static void cb_write(cbuffer_t *dst, const uint32_t offset, const uint8_t *src, const uint8_t length)
{
    if (length > (CONFIG_LOG_CBUFFER_SIZE - offset)) {
        memcpy(&dst->buf[offset], &src[0], (CONFIG_LOG_CBUFFER_SIZE - offset));
        memcpy(&dst->buf[0], &src[(CONFIG_LOG_CBUFFER_SIZE - offset)], (length - (CONFIG_LOG_CBUFFER_SIZE - offset)));
    } else {
        memcpy(&dst->buf[offset], &src[0], length);
    }
}
