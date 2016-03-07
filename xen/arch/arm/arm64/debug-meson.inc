/*
 * xen/arch/arm/arm64/debug-meson.S
 *
 * Meson specific debug code
 *
 * Brian Kim <brian.kim@hardkernel.com>
 * Copyright (c) 2016 Hardkernel Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/meson-uart.h>

/* MESON UART wait UART to be ready to transmit
 * xb: register which contains the UART base address
 * c: scratch register number */
.macro early_uart_ready xb, c
1:
        ldr  w\c, [\xb, #AML_UART_STATUS] /* <- UARTx_STATUS (Flag Register) */
        tst  w\c, #AML_UART_TX_FULL       /* Check Tx FIFO Full bit */
        b.ne 1b                           /* Wait for the UART to be ready */
.endm

/* MESON UART transmit character
 * xb: register which contains the UART base address
 * wt: register which contains the character to transmit */
.macro early_uart_transmit xb, wt
        strb  \wt, [\xb]                  /* -> UARTx_WFIFO (Data Register) */
.endm

/*
 * Local variables:
 * mode: ASM
 * indent-tabs-mode: nil
 * End:
 */
