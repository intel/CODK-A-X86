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

#ifndef __GPIO_SERVICE_H__
#define __GPIO_SERVICE_H__

#include <stdint.h>

#include "cfw/cfw.h"
#include "cfw/cfw_client.h"
#include "services/services_ids.h"

#include "drivers/data_type.h"

/**
 * @defgroup gpio_service GPIO Service
 * General Purpose Input/Output (GPIO)
 * @ingroup services
 */

 /**
 * @defgroup gpio_service_api GPIO Service API
 * Defines an API common for the GPIO services of each core.
 * @ingroup gpio_service
 * @{
 */

/** service internal message ID for @ref gpio_configure */
#define MSG_ID_GPIO_CONFIGURE_REQ   MSG_ID_GPIO_BASE
/** service internal message ID for @ref gpio_set_state */
#define MSG_ID_GPIO_SET_REQ         (MSG_ID_GPIO_BASE + 1)
/** service internal message ID for @ref gpio_get_state */
#define MSG_ID_GPIO_GET_REQ         (MSG_ID_GPIO_BASE + 2)
/** service internal message ID for @ref gpio_listen */
#define MSG_ID_GPIO_LISTEN_REQ      (MSG_ID_GPIO_BASE + 3)
/** service internal message ID for @ref gpio_unlisten */
#define MSG_ID_GPIO_UNLISTEN_REQ    (MSG_ID_GPIO_BASE + 4)

/** message ID of service response for @ref gpio_configure */
#define MSG_ID_GPIO_CONFIGURE_RSP   (MSG_ID_GPIO_CONFIGURE_REQ | 0x40)
/** message ID of service response for @ref gpio_set_state */
#define MSG_ID_GPIO_SET_RSP         (MSG_ID_GPIO_SET_REQ | 0x40)
/** message ID of service response for @ref gpio_get_state */
#define MSG_ID_GPIO_GET_RSP         (MSG_ID_GPIO_GET_REQ | 0x40)
/** message ID of service response for @ref gpio_listen */
#define MSG_ID_GPIO_LISTEN_RSP      (MSG_ID_GPIO_LISTEN_REQ | 0x40)
/** message ID of service response for @ref gpio_unlisten */
#define MSG_ID_GPIO_UNLISTEN_RSP    (MSG_ID_GPIO_UNLISTEN_REQ | 0x40)
/** message ID of service response for GPIO events */
#define MSG_ID_GPIO_EVT             (MSG_ID_GPIO_BASE | 0x80)

/** GPIO interrupt types */
typedef enum {
    RISING_EDGE,
    FALLING_EDGE,
    BOTH_EDGE     // Used on soc GPIO only
} gpio_service_isr_mode_t;

/** Debounce configuration */
typedef enum {
    DEB_OFF = 0,
    DEB_ON
} gpio_service_debounce_mode_t;

/** Request message structure for @ref gpio_configure */
typedef struct gpio_configure_req_msg {
    struct cfw_message header;
    gpio_service_isr_mode_t mode; /*!< requested mode for each gpio: 0 - gpio is an output; 1 - gpio is an input */
    uint8_t index;                /*!< index of the gpio to configure in the port */
} gpio_configure_req_msg_t;

/** Response message structure for @ref gpio_configure */
typedef struct gpio_configure_rsp_msg {
    struct cfw_rsp_message rsp_header;
} gpio_configure_rsp_msg_t;

/** Request message structure for @ref gpio_set_state */
typedef struct gpio_set_req_msg {
    struct cfw_message header;
    uint8_t index;  /*!< index of the gpio in the port */
    uint8_t state;  /*!< state of the gpio to set */
} gpio_set_req_msg_t;

/** Response message structure for @ref gpio_set_state */
typedef struct gpio_set_rsp_msg {
    struct cfw_rsp_message rsp_header;
} gpio_set_rsp_msg_t;

/** Request message structure for @ref gpio_listen */
typedef struct gpio_listen_req_msg {
    struct cfw_message header;
    uint8_t index;                         /*!< index of GPIO pin to monitor */
    gpio_service_isr_mode_t mode;          /*!< interrupt mode */
    gpio_service_debounce_mode_t debounce; /*!< debounce mode */
} gpio_listen_req_msg_t;

/** Response message structure @ref gpio_listen */
typedef struct gpio_listen_rsp_msg {
    struct cfw_rsp_message rsp_header;
    uint8_t index; /*!< index of GPIO pin to monitor */
} gpio_listen_rsp_msg_t;

/** Event message structure for @ref gpio_listen */
typedef struct gpio_listen_evt_msg {
    struct cfw_message header;
    uint8_t index;  /*!< index of GPIO which triggered the interrupt */
    bool pin_state; /*!< current gpio state */
} gpio_listen_evt_msg_t;

/** Request message structure for @ref gpio_unlisten */
typedef struct gpio_unlisten_req_msg {
    struct cfw_message header;
    uint8_t index; /*!< index of GPIO to monitor */
} gpio_unlisten_req_msg_t;

/** Response message structure @ref gpio_unlisten */
typedef struct gpio_unlisten_rsp_msg {
    struct cfw_rsp_message rsp_header;
    uint8_t index; /*!< index of GPIO to stop monitoring */
} gpio_unlisten_rsp_msg_t;

/** Request message structure for @ref gpio_get_state */
typedef struct gpio_get_req_msg {
    struct cfw_message header;
} gpio_get_req_msg_t;

/** Response message structure @ref gpio_get_state */
typedef struct gpio_get_rsp_msg {
    struct cfw_rsp_message rsp_header;
    uint32_t state; /*!< state of the gpio port */
} gpio_get_rsp_msg_t;

/**
 * Intialize/register the GPIO service.
 *
 * @param queue the queue this service will use for processing its messages
 * @param service_id the id this service is assigned
 */
void gpio_service_init(void * queue, int service_id);

/**
 * Configure a GPIO line.
 *
 * @param svc_handle the service connection handle
 * @param index the GPIO index in the port to configure
 * @param mode the mode of the GPIO line (0 - input, 1 - output)
 * @param priv the private data passed back in the
 *             response message.
 */
int gpio_configure(svc_client_handle_t * svc_handle, uint8_t index,
        uint8_t mode, void * priv);

/**
 * Set the state of a GPIO line.
 *
 * @param svc_handle the service connection handle
 * @param index the GPIO index in the port to configure
 * @param value the state to set the GPIO line (0 - low level, 1 - high level)
 * @param priv the private data passed back in the
 *             response message.
 */
int gpio_set_state(svc_client_handle_t * svc_handle, uint8_t index,
        uint8_t value, void * priv);

/**
 * Get state of a GPIO line
 *
 * @param svc_handle the service connection handle
 * @param priv the private data passed back in the
 *             response message.
 *
 * GPIO state will be available in the @ref gpio_get_rsp_msg
 */
int gpio_get_state(svc_client_handle_t * svc_handle, void * priv);

/**
 * Register to gpio state change.
 *
 * @param h the service connection handle
 * @param pin the GPIO index in the port to configure
 * @param mode interrupt mode (RISING_EDGE, FALLING_EDGE)
 * @param debounce debounce config (DEB_OFF, DEB_ON)
 * @param priv the private data passed back in the response message.
 *
 * @msc
 *  Client,"GPIO Service","GPIO driver";
 *
 *  Client->"GPIO Service" [label="register state change", URL="\ref gpio_listen_req_msg"];
 *  "GPIO Service"=>"GPIO driver" [label="call driver configuration"];
 *  "GPIO Service"<<"GPIO driver" [label="configuration status"];
 *  Client<-"GPIO Service" [label="status", URL="\ref gpio_listen_rsp_msg"];
 *  --- [label="GPIO state change"];
 *  Client<-"GPIO Service" [label="state change event", URL="\ref gpio_listen_evt_msg"];
 * @endmsc
 *
 * GPIO state and changed mask are available in the @ref gpio_listen_evt_msg message.
 */
int gpio_listen(svc_client_handle_t * h, uint8_t pin, gpio_service_isr_mode_t mode, uint8_t debounce, void *priv);

/**
 * Unregister to gpio state change.
 *
 * @param h the service connection handle
 * @param pin the GPIO index in the port to configure
 * @param priv the private data passed back in the response message.
 *
 *
 * GPIO state and changed mask are available in the @ref gpio_unlisten_rsp_msg message.
 */
int gpio_unlisten(svc_client_handle_t * h, uint8_t pin, void *priv);

/** @} */

#endif
