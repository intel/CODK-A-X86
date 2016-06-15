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

#include <stdint.h>

#include "cfw/cfw.h"
#include "cfw/cfw_service.h"

#ifdef CONFIG_SS_GPIO
#include "drivers/ss_gpio_iface.h"
#endif
#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)
#include "drivers/soc_gpio.h"
#endif

#include "services/services_ids.h"
#include "services/gpio_service.h"
#include "infra/log.h"
#include "infra/device.h"

#include "machine.h"

/**
 * \brief gpio service management structure
 */
struct gpio_service_data {
    service_t svc;
    struct device **gpio_devs;
};

/**
 * \brief gpio listen management structure
 */
struct gpio_service_listen_priv {
    conn_handle_t *conn;
    void          *priv;
    uint8_t        pin;
};


static void gpio_client_connected(conn_handle_t * instance);
static void gpio_client_disconnected(conn_handle_t * instance);
static void gpio_handle_message(struct cfw_message * msg, void * param);

/****************************************************************************************
 ************************** SERVICE INITIALIZATION **************************************
 ****************************************************************************************/

#ifdef CONFIG_SS_GPIO
static struct gpio_service_data ss_gpio_service = {
    .svc = {
        .client_connected = gpio_client_connected,
        .client_disconnected = gpio_client_disconnected,
    },
    // ss_gpio service handles ss_gpio_8b0 and ss_gpio_8b1 devices
    .gpio_devs = (struct device*[2]) {NULL}
};
#endif
#ifdef CONFIG_SOC_GPIO_32
static struct gpio_service_data soc_gpio_service = {
    .svc = {
        .client_connected = gpio_client_connected,
        .client_disconnected = gpio_client_disconnected,
    },
    .gpio_devs = (struct device*[1]) {NULL}
};
#endif
#ifdef CONFIG_SOC_GPIO_AON
static struct gpio_service_data aon_gpio_service = {
    .svc = {
        .client_connected = gpio_client_connected,
        .client_disconnected = gpio_client_disconnected,
    },
    .gpio_devs = (struct device*[1]) {NULL}
};
#endif

/****************************************************************************************
 ************************** SERVICE IMPLEMENTATION **************************************
 ****************************************************************************************/

void gpio_service_init(void * queue, int service_id) {
    switch (service_id) {
#ifdef CONFIG_SS_GPIO
        case SS_GPIO_SERVICE_ID:
            // Get ss_gpio device handlers
            ss_gpio_service.gpio_devs[0] = get_device(SS_GPIO_8B0_ID);
            ss_gpio_service.gpio_devs[1] = get_device(SS_GPIO_8B1_ID);
            if (!(ss_gpio_service.gpio_devs[0] || ss_gpio_service.gpio_devs[1])) {
                pr_error(LOG_MODULE_GPIO_SVC, "ss_gpio service register failed");
                break;
            }
            // Set service_id field
            ss_gpio_service.svc.service_id = service_id;
            // Register service
            cfw_register_service(queue, &ss_gpio_service.svc, gpio_handle_message, &ss_gpio_service);
            pr_debug(LOG_MODULE_GPIO_SVC, "register ss_gpio service %d", service_id);
            break;
#endif
#ifdef CONFIG_SOC_GPIO_32
        case SOC_GPIO_SERVICE_ID:
            // Get ss_gpio device handlers
            soc_gpio_service.gpio_devs[0] = get_device(SOC_GPIO_32_ID);
            if (!soc_gpio_service.gpio_devs[0]) {
                pr_error(LOG_MODULE_GPIO_SVC, "soc_gpio_32 service register failed");
                break;
            }
            // Set service_id field
            soc_gpio_service.svc.service_id = service_id;
            // Register service
            cfw_register_service(queue, &soc_gpio_service.svc, gpio_handle_message, &soc_gpio_service);
            pr_debug(LOG_MODULE_GPIO_SVC, "register soc_gpio_32 service %d", service_id);
            break;
#endif
#ifdef CONFIG_SOC_GPIO_AON
        case AON_GPIO_SERVICE_ID:
            // Get ss_gpio device handlers
            aon_gpio_service.gpio_devs[0] = get_device(SOC_GPIO_AON_ID);
            if (!aon_gpio_service.gpio_devs[0]) {
                pr_error(LOG_MODULE_GPIO_SVC, "soc_gpio_aon service register failed");
                break;
            }
            // Set service_id field
            aon_gpio_service.svc.service_id = service_id;
            // Register service
            cfw_register_service(queue, &aon_gpio_service.svc, gpio_handle_message, &aon_gpio_service);
            pr_debug(LOG_MODULE_GPIO_SVC, "register soc_gpio_aon service %d", service_id);
            break;
#endif
        default:
            pr_error(LOG_MODULE_GPIO_SVC, "service ID %d unknown", service_id);
            break;
    }
}

static void gpio_client_connected(conn_handle_t * instance) {
    pr_debug(LOG_MODULE_GPIO_SVC, "%s:", __func__);
}

static void gpio_client_disconnected(conn_handle_t * instance) {
    pr_debug(LOG_MODULE_GPIO_SVC, "%s:", __func__);
}

static void gpio_set_default_config(gpio_cfg_data_t *config)
{
    // Default configuration for interrupts
    config->gpio_type = GPIO_INPUT;
    config->int_type = EDGE;
    config->int_polarity = ACTIVE_LOW;
    config->int_debounce = DEBOUNCE_OFF;
    config->int_ls_sync = LS_SYNC_OFF;
    config->gpio_cb = NULL;
    config->gpio_cb_arg = NULL;
}

static void gpio_service_callback(bool state, void *priv)
{
    struct gpio_service_listen_priv *service_priv = (struct gpio_service_listen_priv*)priv;

    OS_ERR_TYPE err;
    gpio_listen_evt_msg_t *msg;
    if((msg = (gpio_listen_evt_msg_t*)
                cfw_alloc_message(sizeof(*msg), &err)) != NULL) {
        CFW_MESSAGE_LEN((struct cfw_message *)msg) = sizeof(*msg);
        CFW_MESSAGE_ID((struct cfw_message *)msg) = MSG_ID_GPIO_EVT;
        CFW_MESSAGE_TYPE((struct cfw_message *)msg) = TYPE_EVT;
        CFW_MESSAGE_SRC((struct cfw_message *)msg) = service_priv->conn->svc->port_id;
        CFW_MESSAGE_DST((struct cfw_message *)msg) = service_priv->conn->client_port;
        CFW_MESSAGE_PRIV((struct cfw_message *)msg) = service_priv->priv;
        CFW_MESSAGE_CONN((struct cfw_message *)msg) = service_priv->conn;
        msg->pin_state = state;
        msg->index = service_priv->pin;

        cfw_send_message(msg);
    } else {
        pr_error(LOG_MODULE_GPIO_SVC, "%s: Error: cfw allocation failed for message", __func__);
        panic(E_OS_ERR_UNKNOWN); // Panic because we should never reach this point.
    }
}

/**
 * Find which gpio device to use, according to pin number.
 *
 * @param svc gpio service to use (port identifier)
 * @param pin gpio pin to use with gpio service svc
 *            This parameter is translated to device pin index.
 *
 * @return gpio device if found, and update pin variable to driver format
 *         NULL if pin value is not valid or gpio device not found
 */
static struct device* get_gpio_dev(struct gpio_service_data *svc, uint8_t *pin)
{
    switch (svc->svc.service_id) {
#if defined(CONFIG_SS_GPIO)
        case SS_GPIO_SERVICE_ID:
            // There are two ports of 8 bits on SS GPIO (SS_GPIO_8B0 and SS_GPIO_8B1)
            // The user specify a pin between 0 and 15
            // We need to convert it to a port number and a pin between 0 and 7
            if (*pin < SS_GPIO_8B0_BITS) {
                return svc->gpio_devs[0];
            }
            *pin -= SS_GPIO_8B0_BITS;
            if (*pin < SS_GPIO_8B1_BITS) {
                return svc->gpio_devs[1];
            }
            break;
#endif
#if defined(CONFIG_SOC_GPIO_32)
        case SOC_GPIO_SERVICE_ID:
            if (*pin < SOC_GPIO_32_BITS) {
                return svc->gpio_devs[0];
            }
            break;
#endif
#if defined(CONFIG_SOC_GPIO_AON)
        case AON_GPIO_SERVICE_ID:
            if (*pin < SOC_GPIO_AON_BITS) {
                return svc->gpio_devs[0];
            }
            break;
#endif
        default:
            break;
    }
    return NULL;
}

void handle_configure(struct cfw_message *msg, struct gpio_service_data *svc)
{
    DRIVER_API_RC ret;
    gpio_cfg_data_t config;

    gpio_configure_req_msg_t * req = (gpio_configure_req_msg_t*) msg;
    gpio_configure_rsp_msg_t *resp = (gpio_configure_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
         MSG_ID_GPIO_CONFIGURE_RSP, sizeof(*resp));

    uint8_t index = req->index;
    struct device *gpio_dev = get_gpio_dev(svc, &index);

    if (!gpio_dev) {
        // gpio device not found or pin not available
        resp->rsp_header.status = DRV_RC_INVALID_CONFIG;
        goto send_rsp;
    }

    // Default configuration for interrupts
    gpio_set_default_config(&config);

    config.gpio_type = req->mode == 0 ? GPIO_INPUT : GPIO_OUTPUT;

    switch (svc->svc.service_id) {
#if defined(CONFIG_SS_GPIO)
        case SS_GPIO_SERVICE_ID:
            ret = ss_gpio_set_config(gpio_dev, index, &config);
            break;
#endif
#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)
        case SOC_GPIO_SERVICE_ID:
        case AON_GPIO_SERVICE_ID:
            ret = soc_gpio_set_config(gpio_dev, index, &config);
            break;
#endif
        default:
            ret = DRV_RC_INVALID_CONFIG;
            break;
    }
    resp->rsp_header.status = ret;

send_rsp:
    cfw_send_message(resp);
}

void handle_set(struct cfw_message *msg, struct gpio_service_data *svc)
{
    DRIVER_API_RC ret;

    gpio_set_req_msg_t * req = (gpio_set_req_msg_t *) msg;
    gpio_set_rsp_msg_t * resp = (gpio_set_rsp_msg_t * ) cfw_alloc_rsp_msg(msg,
            MSG_ID_GPIO_SET_RSP, sizeof(*resp));

    uint8_t index = req->index;
    struct device *gpio_dev = get_gpio_dev(svc, &index);

    if (!gpio_dev) {
        // gpio device not found or pin not available
        resp->rsp_header.status = DRV_RC_INVALID_CONFIG;
        goto send_rsp;
    }

    switch (svc->svc.service_id) {
#if defined(CONFIG_SS_GPIO)
        case SS_GPIO_SERVICE_ID:
            ret = ss_gpio_write(gpio_dev, index, req->state);
            break;
#endif
#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)
        case SOC_GPIO_SERVICE_ID:
        case AON_GPIO_SERVICE_ID:
            ret = soc_gpio_write(gpio_dev, index, req->state);
            break;
#endif
        default:
            ret = DRV_RC_INVALID_CONFIG;
            break;
    }
    resp->rsp_header.status = ret;

send_rsp:
    cfw_send_message(resp);
}

void handle_get(struct cfw_message *msg, struct gpio_service_data *svc)
{
    DRIVER_API_RC ret = DRV_RC_OK;
    uint32_t port_state;
    // get_gpio is currently for a gpio port only, no need to use an index
    uint8_t index = 0;

    gpio_get_rsp_msg_t * resp = (gpio_get_rsp_msg_t * ) cfw_alloc_rsp_msg(msg,
            MSG_ID_GPIO_GET_RSP, sizeof(*resp));

    struct device *gpio_dev = get_gpio_dev(svc, &index);

    if (!gpio_dev) {
        // gpio device not found or pin not available
        resp->rsp_header.status = DRV_RC_INVALID_CONFIG;
        goto send_rsp;
    }

    switch (svc->svc.service_id) {
#if defined(CONFIG_SS_GPIO)
        case SS_GPIO_SERVICE_ID:
            {
            uint32_t val = 0;
            if (svc->gpio_devs[0]) {
                ret = ss_gpio_read_port(svc->gpio_devs[0], &val);
            }
            port_state = val & ((1 << SS_GPIO_8B0_BITS) - 1);
            if ((ret == DRV_RC_OK) && svc->gpio_devs[1]) {
                ret = ss_gpio_read_port(svc->gpio_devs[1], &val);
                ((uint8_t*)(&port_state))[1] = val;
            }
            }
            break;
#endif
#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)
        case SOC_GPIO_SERVICE_ID:
        case AON_GPIO_SERVICE_ID:
            ret = soc_gpio_read_port(gpio_dev, &port_state);
            break;
#endif
        default:
            ret = DRV_RC_INVALID_CONFIG;
            break;
    }

    resp->rsp_header.status = ret;
    resp->state = ((ret == DRV_RC_OK) ? port_state:0);

send_rsp:
    cfw_send_message(resp);
}

void handle_unlisten(struct cfw_message *msg, struct gpio_service_data *svc)
{
    struct gpio_service_listen_priv *priv = NULL;
    DRIVER_API_RC ret;
    gpio_cfg_data_t config;
    gpio_unlisten_req_msg_t * req = (gpio_unlisten_req_msg_t *) msg;

    gpio_unlisten_rsp_msg_t * resp = (gpio_unlisten_rsp_msg_t * ) cfw_alloc_rsp_msg(msg,
            MSG_ID_GPIO_UNLISTEN_RSP, sizeof(*resp));

    resp->index = req->index;

    uint8_t index = req->index;
    struct device *gpio_dev = get_gpio_dev(svc, &index);

    if (!gpio_dev) {
        // gpio device not found or pin not available
        resp->rsp_header.status = DRV_RC_INVALID_CONFIG;
        goto send_rsp;
    }

    // Get priv pointer
    switch (svc->svc.service_id) {
#if defined(CONFIG_SS_GPIO)
        case SS_GPIO_SERVICE_ID:
            priv = ss_gpio_get_callback_arg(gpio_dev, index);
            break;
#endif
#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)
        case SOC_GPIO_SERVICE_ID:
        case AON_GPIO_SERVICE_ID:
            priv = soc_gpio_get_callback_arg(gpio_dev, index);
            break;
#endif
        default:
            ret = DRV_RC_INVALID_CONFIG;
            goto send_rsp;
            break;
    }

    // Default configuration for interrupts
    gpio_set_default_config(&config);

    switch (svc->svc.service_id) {
#if defined(CONFIG_SS_GPIO)
        case SS_GPIO_SERVICE_ID:
            ret = ss_gpio_set_config(gpio_dev, index, &config);
            break;
#endif
#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)
        case SOC_GPIO_SERVICE_ID:
        case AON_GPIO_SERVICE_ID:
            ret = soc_gpio_set_config(gpio_dev, index, &config);
            break;
#endif
        default:
            ret = DRV_RC_INVALID_CONFIG;
            break;
    }

    if (ret != DRV_RC_OK) {
        // Configuration failed
        resp->rsp_header.status = ret;
        goto send_rsp;
    }

    // Listen is now disabled, free callback argument
    if (priv != NULL) {
        bfree(priv);
    }
    resp->rsp_header.status = DRV_RC_OK;

send_rsp:
    cfw_send_message(resp);
}

void handle_listen(struct cfw_message *msg, struct gpio_service_data *svc)
{
    struct gpio_service_listen_priv *priv;
    DRIVER_API_RC ret = DRV_RC_OK;
    OS_ERR_TYPE err;
    gpio_cfg_data_t config;
    gpio_listen_req_msg_t * req = (gpio_listen_req_msg_t *) msg;

    gpio_listen_rsp_msg_t * resp = (gpio_listen_rsp_msg_t * ) cfw_alloc_rsp_msg(msg,
            MSG_ID_GPIO_LISTEN_RSP, sizeof(*resp));

    resp->index = req->index;

    uint8_t index = req->index;
    struct device *gpio_dev = get_gpio_dev(svc, &index);

    if (!gpio_dev) {
        // gpio device not found or pin not available
        resp->rsp_header.status = DRV_RC_INVALID_CONFIG;
        goto send_rsp;
    }

    // Check that callback argument is NULL for the gpio to listen
    switch (svc->svc.service_id) {
#if defined(CONFIG_SS_GPIO)
        case SS_GPIO_SERVICE_ID:
            priv = ss_gpio_get_callback_arg(gpio_dev, index);
            break;
#endif
#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)
        case SOC_GPIO_SERVICE_ID:
        case AON_GPIO_SERVICE_ID:
            priv = soc_gpio_get_callback_arg(gpio_dev, index);
            break;
#endif
        default:
            resp->rsp_header.status = DRV_RC_INVALID_CONFIG;
            goto send_rsp;
    }

    if (priv != NULL) {
        // GPIO pin is already listening, abort
        resp->rsp_header.status = DRV_RC_CONTROLLER_IN_USE;
        goto send_rsp;
    }

    // Alloc priv data for service
    if((priv = balloc(sizeof(struct gpio_service_listen_priv), &err)) == NULL) {
        resp->rsp_header.status = err;
        goto send_rsp;
    }

    priv->priv = ((gpio_listen_req_msg_t*)(msg))->header.priv;
    priv->conn = ((gpio_listen_req_msg_t*)(msg))->header.conn;
    priv->pin = req->index;

    // Default configuration for interrupts
    gpio_set_default_config(&config);

    config.gpio_type = GPIO_INTERRUPT;
    config.int_debounce = ((gpio_listen_req_msg_t*)(msg))->debounce;
    config.gpio_cb = gpio_service_callback;
    config.gpio_cb_arg = priv;

    switch (((gpio_listen_req_msg_t*)(msg))->mode) {
        case RISING_EDGE:
            config.int_type = EDGE;
            config.int_polarity = ACTIVE_HIGH;
            break;
        case FALLING_EDGE:
            config.int_type = EDGE;
            config.int_polarity = ACTIVE_LOW;
            break;
        case DOUBLE_EDGE:
            config.int_type = DOUBLE_EDGE;
            config.int_polarity = ACTIVE_HIGH;
            break;
        default:
            pr_error(LOG_MODULE_GPIO_SVC, "%s: unknown mode %d", __FILE__, ((gpio_listen_req_msg_t*)(msg))->mode);
            ret = DRV_RC_INVALID_CONFIG;
            goto exit_free_priv;
    }

    switch (svc->svc.service_id) {
#ifdef CONFIG_SS_GPIO
        case SS_GPIO_SERVICE_ID:
            ret = ss_gpio_set_config(gpio_dev, index, &config);
            break;
#endif
#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)
        case SOC_GPIO_SERVICE_ID:
        case AON_GPIO_SERVICE_ID:
            ret = soc_gpio_set_config(gpio_dev, index, &config);
            break;
#endif
        default:
            ret = DRV_RC_INVALID_CONFIG;
            break;
    }

    if (ret != DRV_RC_OK) {
        pr_error(LOG_MODULE_GPIO_SVC, "gpio_svc: Failed to configure gpio %d (err=%d)\n", svc->svc.service_id, ret);
    }

    resp->rsp_header.status = ret;
    goto send_rsp;

exit_free_priv:
    bfree(priv);
send_rsp:
    cfw_send_message(resp);
}

static void gpio_handle_message(struct cfw_message * msg, void * param)
{
    struct gpio_service_data *gpio_svc = (struct gpio_service_data*)param;
    // Process gpio request
    switch (CFW_MESSAGE_ID(msg)) {
    case MSG_ID_GPIO_CONFIGURE_REQ:
        handle_configure(msg, gpio_svc);
        break;
    case MSG_ID_GPIO_SET_REQ:
        handle_set(msg, gpio_svc);
        break;
    case MSG_ID_GPIO_GET_REQ:
        handle_get(msg, gpio_svc);
        break;
    case MSG_ID_GPIO_LISTEN_REQ:
        handle_listen(msg, gpio_svc);
        break;
    case MSG_ID_GPIO_UNLISTEN_REQ:
        handle_unlisten(msg, gpio_svc);
        break;
    default:
        pr_error(LOG_MODULE_GPIO_SVC, "gpio_srv %d: unexpected message id: %x",
                ((conn_handle_t*)(msg->conn))->svc->service_id, CFW_MESSAGE_ID(msg));
        break;
    }
    cfw_msg_free(msg);
}
