/*
 * xen/include/asm-arm/meson-uart.h
 *
 * Brian Kim <brian.kim@hardkernel.com>
 * Copyright (c) 2016 Hardkernel Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __ASM_ARM_MESON_H
#define __ASM_ARM_MESON_H

#define FIFO_MAX_SIZE               64

/* Register offsets */
#define AML_UART_WFIFO              0x00
#define AML_UART_RFIFO              0x04
#define AML_UART_CONTROL            0x08
#define AML_UART_STATUS             0x0c
#define AML_UART_MISC               0x10
#define AML_UART_REG5               0x14

/* AML_UART_CONTROL bits */
#define AML_UART_TX_EN              (1 << 12)
#define AML_UART_RX_EN              (1 << 13)
#define AML_UART_TX_RST             (1 << 22)
#define AML_UART_RX_RST             (1 << 23)
#define AML_UART_CLR_ERR            (1 << 24)
#define AML_UART_RX_INT_EN          (1 << 27)
#define AML_UART_TX_INT_EN          (1 << 28)
#define AML_UART_DATA_LEN_MASK      (0x03 << 20)
#define AML_UART_DATA_LEN_8BIT      (0x00 << 20)
#define AML_UART_DATA_LEN_7BIT      (0x01 << 20)
#define AML_UART_DATA_LEN_6BIT      (0x02 << 20)
#define AML_UART_DATA_LEN_5BIT      (0x03 << 20)

/* AML_UART_STATUS bits */
#define AML_UART_PARITY_ERR         (1 << 16)
#define AML_UART_FRAME_ERR          (1 << 17)
#define AML_UART_TX_FIFO_WERR       (1 << 18)
#define AML_UART_RX_FULL            (1 << 19)
#define AML_UART_RX_EMPTY           (1 << 20)
#define AML_UART_TX_FULL            (1 << 21)
#define AML_UART_TX_EMPTY           (1 << 22)
#define AML_UART_RX_FIFO_OVERFLOW   (1 << 24)
#define AML_UART_XMIT_BUSY          (1 << 25)
#define AML_UART_ERR                (AML_UART_PARITY_ERR | \
                                     AML_UART_FRAME_ERR  | \
                                     AML_UART_RX_FIFO_OVERFLOW)

/* AML_UART_STATUS FIFO counts */
#define AML_UART_COUNT_MASK         (0x7f)
#define AML_UART_TX_COUNT_SHIFT     (8)
#define AML_UART_RX_COUNT_SHIFT     (0)
#define AML_UART_TX_COUNT_MASK      (AML_UART_COUNT_MASK << AML_UART_TX_COUNT_SHIFT)
#define AML_UART_RX_COUNT_MASK      (AML_UART_COUNT_MASK << AML_UART_RX_COUNT_SHIFT)

/* AML_UART_CONTROL bits */
#define AML_UART_TWO_WIRE_EN        (1 << 15)
#define AML_UART_PARITY_TYPE        (1 << 18)
#define AML_UART_PARITY_EN          (1 << 19)
#define AML_UART_CLEAR_ERR          (1 << 24)
#define AML_UART_STOP_BIN_LEN_MASK  (0x03 << 16)
#define AML_UART_STOP_BIN_1SB       (0x00 << 16)
#define AML_UART_STOP_BIN_2SB       (0x01 << 16)

/* AML_UART_MISC bits */
#define AML_UART_XMIT_IRQ(c)        (((c) & 0xff) << 8)
#define AML_UART_RECV_IRQ(c)        ((c) & 0xff)

/* AML_UART_REG5 bits */
#define AML_UART_BAUD_MASK          (0x7fffff)
#define AML_UART_BAUD_USE           (1 << 23)
#define AML_UART_BAUD_XTAL          (1 << 24)

#define AML_UART_RX_DATA_MASK       (0xff)

#endif /* __ASM_ARM_MESON_H */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
