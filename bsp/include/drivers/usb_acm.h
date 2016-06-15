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

#ifndef __ACM_H__
#define __ACM_H__

#include "drivers/data_type.h"
#include "usb.h"

/**
 * @defgroup usb_driver_acm USB CDC-ACM driver API
 * USB CDC-ACM driver API.
 * @ingroup usb_driver
 * @{
 */

#define CDC_CLASS 2
#define CDC_DATA_INTERFACE_CLASS 0xA

#define CDC_SUBCLASS_ACM  2

#define CDC_PROTOCOL_V250 1

#define CDC_DESCRIPTOR_TYPE_CS_INTERFACE 0x24
#define CDC_DESCRIPTOR_TYPE_CS_ENDPOINT  0x25

#define CDC_DESCRIPTOR_SUBTYPE_HEADER                       0
#define CDC_DESCRIPTOR_SUBTYPE_CALL_MANAGEMENT              1
#define CDC_DESCRIPTOR_SUBTYPE_ACM                          2
#define CDC_DESCRIPTOR_SUBTYPE_UNION                        6

#define ACM_GET_ENCAPSULATED_RESPONSE       0
#define ACM_SEND_ENCAPSULATED_COMMAND       1

#define ACM_REQUEST_SET_LINE_CODING         0x20
#define ACM_REQUEST_GET_LINE_CODING         0x21
#define ACM_REQUEST_SET_CONTROL_LINE_STATE  0x22

#define ACM_NOTIFICATION_RESPONSE_AVAILABLE 0x01
#define ACM_NOTIFICATION_SERIAL_STATE       0x20

/* bits for acm_interrupt_request */
#define ACM_SERIAL_STATE_RX_CARRIER  (1<<0)
#define ACM_SERIAL_STATE_TX_CARRIER  (1<<1)
#define ACM_SERIAL_STATE_BREAK       (1<<2)
#define ACM_SERIAL_STATE_RING_SIGNAL (1<<3)
#define ACM_SERIAL_STATE_FRAMING     (1<<4)
#define ACM_SERIAL_STATE_PARITY      (1<<5)
#define ACM_SERIAL_STATE_OVERRUN     (1<<6)

/** disconnection event */
#define ACM_EVENT_DISCONNECTED      0
/** connection event */
#define ACM_EVENT_CONNECTED         1
/** baud rate changed event
 *
 * callback parameter is the selected baudrate.
 */
#define ACM_EVENT_SET_BAUD          2
/**
 * Set control line state event
 * Generated when the host's cdc-acm wants to change the value of one or both
 * of the follwoing control bits: DTR and RTS
 */
#define ACM_SET_CONTROL_LINE_STATE  3

/*
 * Input control lines state
 */
#define ACM_CTRL_DISC           0x00
#define ACM_CTRL_DCD            0x01
#define ACM_CTRL_CTS            0x02

/**
 * read data from an acm channel.
 * callback is called when the transfer completes.
 * The first parameter of the callback is the actual number of bytes
 * transfered.
 *
 * @param  idx the index of the ACM interface to use
 * @param  buffer the buffer to read from acm interface
 * @param  len the length of the data to read (in bytes)
 * @param  xfer_done the callback function called when transfer is complete
 *                  this function will be called in the interrupt context
 * @param  data the data passed to the transfer complete callback
 * @return  0 if success DRV_RC_BUSY if a request is already pending
 */
int acm_read(int idx, uint8_t *buffer, int len,
	     void (*xfer_done)(int, void*), void * data);
/**
 * write data to an acm channel
 * callback is called when the transfer completes.
 * The first parameter of the callback is the actual number of bytes
 * transfered.
 *
 * @param  idx the index of the ACM interface to use
 * @param  buffer the data buffer to write
 * @param  len the length of the data buffer
 * @param  xfer_done the callback function called when transfer is complete
 *                  this function will be called in the interrupt context
 * @param  data the data passed to the transfer complete callback
 * @return  0 if success.
 */
int acm_write(int idx, uint8_t *buffer, int len,
	      void (*xfer_done)(int, void*), void * data);

/**
 * Set the com state.
 *
 * @param  idx the ACM interface index.
 * @param  state bitmap of ACM_SERIAL_STATE*
 */
void acm_set_comm_state(int idx, uint8_t state);

/**
 * Initialize an acm interface.
 *
 * @param  idx the index of the ACM interface to use
 * @param  event_callback the callback that will be called on various ACM
 *                       events like:
 *                       \ref ACM_EVENT_CONNECTED
 *                       \ref ACM_EVENT_DISCONNECTED
 *                       \ref ACM_EVENT_SET_BAUD
 * @return  0 if success
 */
int acm_init(int idx, void (*event_callback)(int event, int param));

/**
 * ACM driver initialization function.
 * This function should be called by the platform initialization.
 *
 * @param priv pointer to user defined data
 */
void usb_acm_class_init(void *priv, usb_string_descriptor_t * alt_strings);

/** @} */
#endif /* __ACM_H__ */
