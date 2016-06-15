/*
 * Copyright (c) 2016, Intel Corporation. See the bottom of the file for full license.
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include "os/os.h"

#include "util/cbuffer.h"
#include "infra/log.h"
#include "log_impl.h"
#include "infra/log_impl_cbuffer.h"

#ifdef CONFIG_LOG_MULTI_CPU_SUPPORT
#include "infra/port.h"
#include "infra/ipc.h"
#include "machine.h"
#endif
#include "infra/time.h"

#define LOG_BUFFER_SIZE             CONFIG_LOG_CBUFFER_SIZE

/* First message magic number; */
#define LOG_MESSAGE_MAGIC_A     0xFE

/* Second message magic number; */
#define LOG_MESSAGE_MAGIC_B     0xEE

/* The main circular buffer where messages are transiently stored */
static uint8_t logbuf[CONFIG_LOG_CBUFFER_SIZE];
static cbuffer_t log_buffer = {.buf=logbuf, .buf_size=CONFIG_LOG_CBUFFER_SIZE};

/* Used when a new message comes either from local core, or other cores */
static T_SEMAPHORE new_msg_notif = NULL;

/* Fill the passed message by taking one from the cbuffer */
static int32_t log_read_msg(log_message_t * p_msg);


#ifdef CONFIG_LOG_MASTER

#define LOG_SLAVES_NUM (LOG_CORE_NUM-1)
struct log_slave_data {
	log_message_t msg;
	bool state;
	uint8_t cpu_id;
};
static struct log_slave_data slavesdata[LOG_SLAVES_NUM];

static uint8_t cpu_id_to_slave_index(uint8_t cpu_id)
{
	int i;
	for (i=0;i<LOG_SLAVES_NUM;++i) {
		if (slavesdata[i].cpu_id == cpu_id)
			return i;
	}
	/* assert(0); */
	return 0;
}

void log_incoming_msg_from_slave(int cpu_id, const log_message_t* msg)
{
	slavesdata[cpu_id_to_slave_index(cpu_id)].state = 1;
	semaphore_give(new_msg_notif, NULL);
}

/* Flush all log messages currently in the cbuffer, this does not include
 * messages from external log cores */
void log_flush()
{
	log_message_t msg;
	while (log_read_msg(&msg) > 0)
		output_one_message(&msg);
}

/* Logger task. Should be lower prio than any other tasks that send messages. */
void log_task()
{
	OS_ERR_TYPE ret;
	uint8_t i;
	static log_message_t tmp_msg;

	/* Flush requests to the slave will be done based on the reception
	 * of the first IPC_REQUEST_LOGGER from the slave
	 */

	while (1) {
		ret = semaphore_take(new_msg_notif, OS_WAIT_FOREVER);
		if (ret == E_OS_OK) {
			while (log_read_msg(&tmp_msg) > 0)
				output_one_message(&tmp_msg);
			for (i = 0; i < LOG_SLAVES_NUM; i++) {
				if (slavesdata[i].state == 1) {
					/* Call backend */
					output_one_message(&slavesdata[i].msg);
					slavesdata[i].state = 0;
					/* Send a flush request to the slaves */
					log_cores[cpu_id_to_logcore_id(slavesdata[i].cpu_id)].send_buffer(IPC_REQUEST_LOGGER, 0, 0, (void*)(&slavesdata[i].msg));
				}
			}
		}
	}
}
#endif

#ifdef CONFIG_LOG_SLAVE
static volatile log_message_t* out_msg = NULL;
static T_SEMAPHORE ipc_notif = NULL;

void log_master_ready_for_new_msg(int cpu_id, log_message_t* msg)
{
	out_msg = msg;
	semaphore_give(ipc_notif, NULL);
}

void log_flush()
{
	return;
}

/* Logger task. Should be lower prio than any other tasks that send messages. */
void log_task()
{
	uint8_t lost_messages = 0;
	/* Send an initial IPC request to tell the master that slave task
	 * is ready. */
	ipc_request_sync_int(IPC_REQUEST_LOGGER, 0, 0, NULL);

	while(1) {
		if (out_msg==NULL) {
			/* Wait for a new valid buffer to be received from master */
			if (semaphore_take(ipc_notif, OS_WAIT_FOREVER) != E_OS_OK) {
				panic(E_OS_ERR);
			}
		}

		/* At this point we are guaranteed to have a master buffer available */
		assert(out_msg);
		/* Wait for a message to be queued in our circular buffer */
		if (semaphore_take(new_msg_notif, OS_WAIT_FOREVER) != E_OS_OK) {
			panic(E_OS_ERR);
		}

		/* Process next message */
		log_message_t* p_msg = (log_message_t*)out_msg;
		if (log_read_msg(p_msg)<=0) {
			/* We were too late and the message has been overwritten (saturation)
			 * Skip this message.. */
			lost_messages++;
		} else {
			/* Use the message as a container to tell the master how many
			 * messages we lost */
			p_msg->lost_messages_count = lost_messages;
			lost_messages = 0;
			out_msg = NULL;
			ipc_request_sync_int(IPC_REQUEST_LOGGER, 0, 0, NULL);
		}
	}
}
#endif

#ifndef CONFIG_LOG_MULTI_CPU_SUPPORT

/* Extract and format all messages in the circular buffer. */
static void log_extract_messages()
{
	static log_message_t rx_msg;
	while (log_read_msg(&rx_msg) > 0) {
		output_one_message(&rx_msg);
	}
}
/**
 * Logger task; should be lower prio than any other task that
 * sends logging messages.
 */
void log_task(void)
{
	OS_ERR_TYPE ret;

	while (1) {
		ret = semaphore_take(new_msg_notif, OS_WAIT_FOREVER);
		if (ret == E_OS_OK) {
			log_extract_messages();
		}
	}
}

void log_flush()
{
	log_extract_messages();
}

#endif


void log_impl_init(void)
{
	if(cb_init(&log_buffer) == -1)
            panic(E_OS_ERR_UNKNOWN);
	new_msg_notif = semaphore_create(0, NULL);

#ifdef CONFIG_LOG_SLAVE
	ipc_notif = semaphore_create(0, NULL);
#endif

#ifdef CONFIG_LOG_MASTER
	/* Initialize the list of slaves */
	int i, j=0;
	for (i=0;i<LOG_SLAVES_NUM;++i)
	{
		if (log_cores[j].cpu_id == get_cpu_id())
		{
			/* This the master CPU, skip from our slaves list */
			j++;
		}
		slavesdata[i].state = 0;
		slavesdata[i].cpu_id = log_cores[j].cpu_id;
		j++;
	}
#endif
}


/**
 * @brief Creates and pushes a user's log message into the logging queue.
 *
 * @retval Message's length if inserted, -1 if an error occurs, 0 if
 * message was discarded.
 */
uint32_t log_write_msg(uint8_t level, uint8_t module, const char *format,
				va_list args)
{
	log_message_t msg;

	/* Contains the full text size not including the terminating \0 */
	msg.buf_size = vsnprintf(msg.buf, sizeof(msg.buf), format, args);
	if (msg.buf_size>=sizeof(msg.buf))
		msg.buf_size=sizeof(msg.buf)-1;
	if (msg.buf_size<=0)
		return 0;

	/* Fill up the message contents */
	/* Note that we abuse the has_saturated and lost_messages_count members
	 * to temporarily store the magic numbers required to check message
	 * consistency when extracting them from the cbuffer.
	 *
	 * Both variables will be properly reset at extraction time.
	 */
	msg.has_saturated = LOG_MESSAGE_MAGIC_A;
	msg.lost_messages_count = LOG_MESSAGE_MAGIC_B;
	msg.level = level;
	msg.module = module;
	msg.timestamp = get_uptime_ms();
#ifdef CONFIG_LOG_MULTI_CPU_SUPPORT
	msg.cpu_id = get_cpu_id();
#else
	msg.cpu_id = 0;
#endif
	uint32_t msg_len = sizeof(msg) - sizeof(msg.buf) + msg.buf_size;

	uint32_t saved = interrupt_lock();
	cb_push(&log_buffer, (const uint8_t*)&msg, msg_len);
	interrupt_unlock(saved);

	semaphore_give(new_msg_notif, NULL);

	return msg_len;
}

/**
 * @brief Read a message in a circular buffer.
 *
 * @param p_msg  pointer on the message filled by the function:
 *   - p_msg->has_saturated is set to 1 if saturation occured, 0 otherwise
 *   - p_msg->lost_messages_count is set to 0
 *
 * @return  1  If no error,
 * @return  0  If no message has been found,
 * @return -1  If an error occurs
 */
static int32_t log_read_msg(log_message_t * p_msg)
{
	uint32_t it_flags;
	int32_t ret;
	int msg_len = 0;
	int buf_len = 0;
	int start_r = 0;	/* Start of the next message index */

	if (log_buffer.r == log_buffer.w) {
		/* No messages in cbuffer */
		return 0;
	}

	/* Searching for the first magic number outside of critical section should be ok.
	 * we check inside the critical section if the second magic number matches; if
	 * the contents were changed during the search, the second magic number will not
	 * match.
	 *
	 * if the first magic number isn't already under log_buffer.r, then it means that log_buffer.r
	 * was pushed ahead because of a saturation; we should be able to find a message
	 * 'to the right' within sizeof(log_message_t) characters. If none is found, then
	 * something is not right => we set log_buffer.r = log_buffer.w = no messages.
	 *
	 * for a safer option: instead of sizeof(log_message_t) use LOG_BUFFER_SIZE
	 */
	start_r = cb_find(LOG_MESSAGE_MAGIC_A, &log_buffer,
			log_buffer.r, log_buffer.w, sizeof(log_message_t));

	it_flags = interrupt_lock();

	if (start_r < 0) {
		log_buffer.r = log_buffer.w;	/* No message found; push r forward to speed up next search */
		interrupt_unlock(it_flags);
		return 0;
	}

	if (log_buffer.buf[(start_r + offsetof(log_message_t, lost_messages_count)) % LOG_BUFFER_SIZE] !=
		LOG_MESSAGE_MAGIC_B) {
		/* Something is not right, we can't match the second magic number; we'll advance r to speed up next search */
		log_buffer.r = start_r;
		interrupt_unlock(it_flags);
		return -1;
	}

	buf_len = log_buffer.buf[((start_r + offsetof(log_message_t, buf_size))
							% LOG_BUFFER_SIZE)];
	msg_len = sizeof(*p_msg) - sizeof(p_msg->buf) + buf_len;

	if (buf_len < 0 || msg_len < 0) {
		/* Start_r doesn't look good.. we'll wait. */
		interrupt_unlock(it_flags);
		return -1;
	}

	uint8_t saturation = log_buffer.saturation_flag;
	ret = cb_pop(&log_buffer, start_r, (unsigned char *)p_msg, msg_len);

	interrupt_unlock(it_flags);

	p_msg->has_saturated = saturation;
	p_msg->lost_messages_count = 0;
	return ret;
}

/*
 * The full license is here at the bottom of the file because
 * changing the top portion of the file modifies the final binaries.
 *
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