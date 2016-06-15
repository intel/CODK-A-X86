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

#ifndef LOG_CBUFFER_H
#define LOG_CBUFFER_H

/**
 * @defgroup infra_log_cbuffer Circular buffer log implementation
 * @ingroup infra_log
 * @{
 * 
 * Use circular buffer to store logs before flushing them.
 *
 * In this implementation, log messages are not output on the backend 
 * immediately but stored in a circular buffer first. A log task with low
 * priority is then awakened and flushes the log messages from the circular
 * buffer to the log_backend.
 * 
 * The log_task() task needs to be started just after calling the log_init()
 * function.
 * 
 * This architecture has the following advantages:
 *  - allows a fast implementation of the pr_debug() and co. family of
 *    functions because they don't do I/O on the backend.
 *  - in multi-core architecture, it allows the "master" to decide when logs can
 *    be output, i.e. "slaves" can discard logs if the master is overwhelmed.
 *  - allows to record logs history for each slaves and dump them only when
 *    necessary, e.g. in case of panic.
 *
 * @image html logger.png "Circular buffer log"
 */

/**
 * Logger task. Should be lower priority than any other task that
 * sends logging messages.
 */
void log_task(void);

/** @} */

#endif /* LOG_CBUFFER_H */
