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
#include <stdio.h>
#include "log_impl.h"
#include "infra/panic.h"

#define LOG_HEADER_LEN   (31) /* According to "9+3+8+5 +5 + 1" length */
#define LOG_EOL_LEN      (2)  /* According to ("/r/n" - '\0') length  */
#define LOG_LINE_SIZE    (LOG_HEADER_LEN + LOG_MAX_MSG_LEN + LOG_EOL_LEN)

/* Compilation options sanity check */
#ifdef CONFIG_LOG_MULTI_CPU_SUPPORT
#if !defined(CONFIG_LOG_SLAVE) && !defined(CONFIG_LOG_MASTER)
#error You need to be slave or master in MULTI_CPU_SUPPORT mode
#endif
#endif

#ifdef CONFIG_LOG_MASTER
#include "machine.h"
#include "infra/ipc.h"
#define DEFINE_LOGGER_CORE(_i_,_k_,_l_,_n_) \
		[_i_] = { \
			.cpu_id = _n_, \
			.name = _k_, \
			.send_buffer = _l_ \
		},
const struct log_core log_cores[] = {
#include "log_cores"
};
#undef DEFINE_LOGGER_CORE

/**
 * Get the human-friendly name of a log core.
 *
 * This name is the one set in the log_cores file, passed as 2nd argument of the
 * DEFINE_LOGGER_CORE X_MACRO.
 *
 * @param core the log core id
 * @return the log core name or NULL if not found
 */
static const char *log_get_core_name(uint8_t logcore_id)
{
	if (logcore_id >= LOG_CORE_NUM)
		panic(E_OS_ERR);
	return log_cores[logcore_id].name;
}

/**
 * Helper function to convert a CPU ID as used in the IPC system into
 * a log core ID.
 */
uint8_t cpu_id_to_logcore_id(uint8_t cpu_id)
{
	int i;
	for (i=0;i<LOG_CORE_NUM;++i) {
		if (log_cores[i].cpu_id == cpu_id)
			return i;
	}
	panic(0);
	return 0xFF;
}
#endif

#if defined(CONFIG_LOG_MASTER) || !defined(CONFIG_LOG_MULTI_CPU_SUPPORT)

/* The backend used to actually ouput text */
static struct log_backend out_backend;

void log_set_backend(struct log_backend backend) {
	out_backend = backend;
}

/* Output one message on the backend */
void output_one_message(const log_message_t* msg)
{
	static const char headerfmt[] = "%9u|%3.3s|%8.8s|%5.5s| ";
	static char line[LOG_LINE_SIZE];
	int msg_len;

	if (msg->buf_size <= 0) {
		return;
	}

	if (msg->has_saturated || msg->lost_messages_count)
	{
		snprintf(line, sizeof(line), headerfmt,
				(unsigned int)msg->timestamp,
#ifdef CONFIG_LOG_MASTER
				log_get_core_name(cpu_id_to_logcore_id(msg->cpu_id)),
#else
"",
#endif
				log_get_module_name(LOG_MODULE_LOG),
				log_get_level_name(LOG_LEVEL_WARNING));

		if (msg->has_saturated)
		{
			static const char log_saturation[] = "-- log saturation --\r\n";
			msg_len = sprintf(line + LOG_HEADER_LEN -1, log_saturation);
			if (msg_len > 0)
				out_backend.put_one_msg(line, LOG_HEADER_LEN + msg_len -1);
		}

		if (msg->lost_messages_count)
		{
			static const char log_lost[] = "%03u log messages lost\r\n";
			msg_len = sprintf(line + LOG_HEADER_LEN -1, log_lost, msg->lost_messages_count);
			if (msg_len > 0)
				out_backend.put_one_msg(line, LOG_HEADER_LEN + msg_len -1);
		}
	}
	snprintf(line, LOG_HEADER_LEN, headerfmt,
		(unsigned int)msg->timestamp,
#ifdef CONFIG_LOG_MASTER
		log_get_core_name(cpu_id_to_logcore_id(msg->cpu_id)),
#else
"",
#endif
		log_get_module_name(msg->module),
		log_get_level_name(msg->level));

	/* A too long log is truncated */
	uint16_t buf_size = msg->buf_size;
	if(msg->buf_size > LOG_MAX_MSG_LEN)
		buf_size = LOG_MAX_MSG_LEN;

	strncpy(line + LOG_HEADER_LEN -1, msg->buf, buf_size);
	strncpy(line + LOG_HEADER_LEN -1 + buf_size, "\r\n", 2);
	out_backend.put_one_msg(line, LOG_HEADER_LEN -1 + buf_size + LOG_EOL_LEN);

}

#endif

