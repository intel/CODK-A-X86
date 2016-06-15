/*******************************************************************************
 *
 * Synopsys DesignWare Sensor and Control IP Subsystem IO Software Driver and
 * documentation (hereinafter, "Software") is an Unsupported proprietary work
 * of Synopsys, Inc. unless otherwise expressly agreed to in writing between
 * Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ******************************************************************************/

/*******************************************************************************
 *
 * Modifications Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 ******************************************************************************/

 /*
 * Intel SOC SPI driver
 *
 */
#ifndef SOC_SPI_PRIV_H_
#define SOC_SPI_PRIV_H_

#include "drivers/data_type.h"

/* SPI software configs */
#define     SPI_TX_FIFO_THRESHOLD       (7)
#define     SPI_RX_FIFO_THRESHOLD       (0)

/* SPI FIFO Size */
#define     IO_SPI_MST0_FS              (8)
#define     IO_SPI_MST1_FS              (8)
#define     IO_SPI_SLV_FS               (8)

/* Chip-Select and GPIO lookup */
#define     SPIM0_CS_1_GPIO24           (24)
#define     SPIM0_CS_2_GPIO25           (25)
#define     SPIM0_CS_3_GPIO26           (26)
#define     SPIM0_CS_4_GPIO27           (27)

/* SoC SPI device register offsets  */
#define     CTRL0                       (0x00)              /* SoC SPI Control Register 1 */
#define     CTRL1                       (0x04)              /* SoC SPI Control Register 2 */
#define     SPIEN                       (0x08)              /* SoC SPI Enable Register */
#define     SER                         (0x10)              /* SoC SPI Slave Enable Register */
#define     BAUDR                       (0x14)              /* SoC SPI Baud Rate Select */
#define     TXFTL                       (0x18)              /* SoC SPI Transmit FIFO Threshold Level */
#define     RXFTL                       (0x1C)              /* SoC SPI Receive FIFO Threshold Level */
#define     TXFL                        (0x20)              /* SoC SPI Transmit FIFO Level Register */
#define     RXFL                        (0x24)              /* SoC SPI Receive FIFO Level Register */
#define     SR                          (0x28)              /* SoC SPI Status Register */
#define     IMR                         (0x2C)              /* SoC SPI Interrupt Mask Register */
#define     ISR                         (0x30)              /* SoC SPI Interrupt Status Register */
#define     RISR                        (0x34)              /* SoC SPI Raw Interrupt Status Register */
#define     TXFOIC                      (0x38)              /* SoC SPI TX FIFO Overflow Interrupt Clear */
#define     RXFOIC                      (0x3C)              /* SoC SPI RX FIFO Overflow Interrupt Clear */
#define     RXFUIC                      (0x40)              /* SoC SPI TX FIFO Underflow Interrupt Clear */
#define     MMIC                        (0x44)              /* SoC SPI Multi Master Interrupt Clear */
#define     ICR                         (0x48)              /* SoC SPI TX Interrupt Clear Register */
#define     IDR                         (0x58)              /* SoC SPI Identification Register */
#define     DR                          (0x60)              /* SoC SPI Data Register */

/* SPI specific macros */
#define     SPI_ENABLE_INT              (0x1f)              /* Enable SoC SPI Interrupts */
#define     SPI_ENABLE_RX_INT           (0x10)              /* Enable SoC SPI RX Interrupts */
#define     SPI_ENABLE_TX_INT           (0x01)              /* Enable SoC SPI TX Interrupts */
#define     SPI_DISABLE_TX_INT          (~0x00000001)       /* Disable TX FIFO Empty interrupts */
#define     SPI_DISABLE_INT             (0x0)               /* Disable SoC SPI Interrupts */
#define     SPI_STATUS_RFF              (0x10)              /* RX FIFO Full */
#define     SPI_STATUS_TFE              (0x4)               /* TX FIFO Empty */
#define     SPI_STATUS_TFNF             (0x2)               /* TX FIFO not full */
#define     SPI_STATUS_RFNE             (0x1 << 3)          /* RX FIFO not empty */
#define     SPI_STATUS_BUSY             (0x1)               /* Busy status */
#define     SPI_POP_DATA                (0x80000000)        /* Dummy data */
#define     SPI_PUSH_DATA               (0xc0000000)        /* Dummy data */
#define     SPI_ENABLE                  (0x1)               /* Enable SoC SPI Device */
#define     SPI_DISABLE                 (0x0)               /* Disable SoC SPI Device */
#define     SPI_TX_INT                  (0x1)               /* SoC SPI TX Interrupt */
#define     SPI_RX_INT                  (0x10)              /* SoC SPI RX Interrupt */
#define     SPI_TXE                     (0x1 << 5)          /* Transmission Error */
#define     SPI_SLAVE_OD                (0x1 << 10)         /* Slave output disable */
#define     SPI_SLAVE_OE                ~(SPI_SLAVE_OD)     /* Slave output enable */

#define     ENABLE_SOC_SPI_INTERRUPTS   (~0x1 << 8)
#define     ENABLE_SPI_MASTER_0         (0x1 << 14)
#define     ENABLE_SPI_MASTER_1         (0x1 << 15)
#define     ENABLE_SPI_SLAVE            (0x1 << 16)
#define     DISABLE_SPI_MASTER_0        (~ENABLE_SPI_MASTER_0)
#define     DISABLE_SPI_MASTER_1        (~ENABLE_SPI_MASTER_1)
#define     DISABLE_SPI_SLAVE           (~ENABLE_SPI_SLAVE)

/* SPI device states */
#define     SPI_STATE_CLOSED            (0)
#define     SPI_STATE_DISABLED          (1)
#define     SPI_STATE_IDLE              (2)
#define     SPI_STATE_TRANSMIT          (3)
#define     SPI_STATE_RECEIVE           (4)
#define     SPI_STATE_SLAVE_FD          (5)     /* Full duplex slave mode */

#define     BAUD_DIVISOR                1000

typedef enum
{
    NORMAL_MODE = 0,
    TEST_MODE           /* Test mode used for loopback testing */
}SHIFT_REG_LOOP;

/*! Enables tracking of which setup function is called first - necessary
 *  when enabling interrupts */
typedef enum
{
    UNASSIGNED = 0,
    RX_FIRST,
    TX_FIRST,
}SLAVE_FULL_DUPLEX_ORDER;


typedef void (*SPI_ISR) ();
typedef void (* IO_CB_FUNC)( uint32_t );

/**
 * Callback struct
 */
typedef struct {
  IO_CB_FUNC        cb;
} io_cb_t;

/**
 *  Private data structure maintained by the driver.
 */
typedef struct soc_spi_info_struct
{
    uint32_t        reg_base; /* base address of device register set */
    /* TX & RX Buffer and lengths */
    uint32_t        tx_len;
    uint32_t        rx_len;
    uint32_t        tx_count;
    uint32_t        rx_count;
    uint32_t        discarded;
    uint32_t        dummy_tx;
    uint8_t *       tx_buf;
    uint8_t *       rx_buf;
    uint16_t *      rx_buf_16;
    uint8_t         state;
    uint8_t         mode;
    uint8_t         cs_gpio;
    uint8_t         instID;
    uint8_t         full_duplex;
    SLAVE_FULL_DUPLEX_ORDER fd_order;
    /* Data frame Size */
    uint8_t         dfs;
    /* Callbacks */
    IO_CB_FUNC      xfer_cb;
    uint32_t        cb_xfer_data;
    IO_CB_FUNC      err_cb;
    uint32_t        cb_err_data;
    IO_CB_FUNC      slave_rx_cb;
    /* Interrupt numbers and handlers */
    uint8_t         isr_vector; /* ISR vectors */
    SPI_ISR         isr;        /* SPI device ISR */
    uint16_t        fifo_depth;
    /* Interrupt Routing Mask Registers */
    uint32_t        spi_int_mask;
} soc_spi_info_t, *soc_spi_info_pt;

void soc_spi_ISR_proc( soc_spi_info_pt dev );

#endif /*   SOC_SPI_PRIV_H_ */
