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

/**
 * @defgroup infra_panic Panic
 * Reset the device in case of non-recoverable errors
 * @ingroup infra
 * @{
 */

#ifndef __PANIC_H__
#define __PANIC_H__

#include <stdint.h>

/** Panic magic key. */
#define PANIC_DATA_MAGIC 0x21636e50

/* Flags for panic */
/** Panic dump is valid and needs to be processed. */
#define PANIC_DATA_FLAG_FRAME_VALID  (1 << 0)
/** Panic dump has been processed and can be removed. */
#define PANIC_DATA_FLAG_FRAME_PULLED (1 << 1)

/** Declare RAM panic data. */
#define DECLARE_PANIC_DATA(arch) \
    struct arch ## _panic_data { \
        struct arch ## _panic_arch_data arch_data; \
        struct panic_data_footer footer; \
    }

/** Declare Flash panic data. */
#define DECLARE_PANIC_DATA_FLASH(arch) \
    struct arch ## _panic_data_flash { \
        struct panic_data_flash_header header; \
        struct arch ## _panic_arch_data arch_data; \
    }

/**
 * Structure for data panic in RAM.
 */
struct panic_data_footer {
    uint8_t arch;               /*!<Architecture (PANIC_ARCH_*) */
    uint8_t struct_version;     /*!<Structure version */
    uint8_t flags;              /*!<Flags (PANIC_DATA_FLAG_*) */
    uint8_t reserved;           /*!<Reserved; set 0 */
    uint32_t time;              /*!<Time stamp */
    uint32_t build_cksum;       /*!<Build checksum (micro-sha1) */
    /*
     * These fields go at the END of the struct so we can find it at the
     * end of memory.
     */
    uint32_t struct_size;       /*!<Size of the full dump structure */
    uint32_t magic;             /*!<PANIC_SAVE_MAGIC if valid */
};

/**
 * Structure for data panic in flash.
 */
struct panic_data_flash_header {
    uint32_t magic;             /*!<PANIC_SAVE_MAGIC if valid */
    uint32_t struct_size;       /*!<Size of this struct */

    uint32_t build_cksum;       /*!<Build checksum (micro-sha1) */
    uint32_t time;              /*!<Time stamp */
    uint8_t arch;               /*!<Architecture (PANIC_ARCH_*) */
    uint8_t struct_version;     /*!<Structure version */
    uint8_t flags;              /*!<Flags (PANIC_DATA_FLAG_*) */
    uint8_t reserved;           /*!<Reserved; set 0 */
};

/**
 * Dump error code, cores registers and stacks into RAM.
 *
 * @param err error see @ref infra_panic.
 */
void panic(int err);

/**
 * @cond PANIC_INTERNAL
 */

/**
 * TODO remove from panic API.
 *
 * Notify the other cores that a panic happened.
 *
 * @param timeout timeout to resume panic handling if slave handshake is not received (in ms).
 */
void panic_notify(int timeout);

/**
 * Notify the other cores that panic notification has been processed.
 *
 */
void panic_done();

/**
 * TODO remove from panic API.
 *
 * Handle panic notification from another core.
 *
 * @param core_id id of core that notified the panic.
 */
void handle_panic_notification(int core_id);

/** @endcond */
/** @} */

#endif /* __PANIC_H__ */
