/*
 * xen/drivers/char/meson-uart.c
 *
 * Driver for Meson UART.
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

#include <xen/config.h>
#include <xen/console.h>
#include <xen/errno.h>
#include <xen/serial.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <xen/mm.h>
#include <asm/device.h>
#include <asm/io.h>
#include <asm/meson-uart.h>

static struct meson_uart {
    unsigned int baud, clock_hz, data_bits, parity, stop_bits;
    unsigned int irq;
    void *regs;
    struct irqaction irqaction;
    struct vuart_info vuart;
} meson_com = {0};

/* These parity settings can be ORed directly into the ULCON. */
#define PARITY_NONE             (0x0)
#define PARITY_ODD              (0x4)
#define PARITY_EVEN             (0x5)
#define FORCED_CHECKED_AS_ONE   (0x6)
#define FORCED_CHECKED_AS_ZERO  (0x7)

#define meson_read(uart, off)       readl((uart)->regs + off)
#define meson_write(uart, off, val) writel(val, (uart->regs) + off)

static void meson_uart_interrupt(int irq, void *data, struct cpu_user_regs *regs)
{
    struct serial_port *port = data;
    struct meson_uart *uart = port->uart;
    uint32_t status, mode;

    status = meson_read(uart, AML_UART_STATUS);

    if ( status & AML_UART_ERR )
    {
        if ( status & AML_UART_TX_FIFO_WERR )
            dprintk(XENLOG_ERR, "uart: overrun error\n");
        else if ( status & AML_UART_FRAME_ERR )
            dprintk(XENLOG_ERR, "uart: frame error\n");
        else if ( status & AML_UART_PARITY_ERR )
            dprintk(XENLOG_ERR, "uart: parity error\n");

        mode = meson_read(uart, AML_UART_CONTROL);
        mode |= AML_UART_CLEAR_ERR;
        meson_write(uart, AML_UART_CONTROL, mode);

        /* It doesn't clear to 0 automatically */
        mode &= ~AML_UART_CLEAR_ERR;
        meson_write(uart, AML_UART_CONTROL, mode);
    }

    if ( !(meson_read(uart, AML_UART_STATUS) & AML_UART_RX_EMPTY) )
        serial_rx_interrupt(port, regs);

    if ( !(meson_read(uart, AML_UART_STATUS) & AML_UART_TX_FULL) )
    {
        if ( meson_read(uart, AML_UART_CONTROL) & AML_UART_TX_INT_EN )
            serial_tx_interrupt(port, regs);
    }
}

static void __init meson_uart_init_preirq(struct serial_port *port)
{
    struct meson_uart *uart = port->uart;
    uint32_t val;

    val = meson_read(uart, AML_UART_CONTROL);
    val |= (AML_UART_RX_RST | AML_UART_TX_RST | AML_UART_CLR_ERR);
    meson_write(uart, AML_UART_CONTROL, val);

    val &= ~(AML_UART_RX_RST | AML_UART_TX_RST | AML_UART_CLR_ERR);
    meson_write(uart, AML_UART_CONTROL, val);
}

static void __init meson_uart_init_postirq(struct serial_port *port)
{
    struct meson_uart *uart = port->uart;
    uint32_t val;

    uart->irqaction.handler = meson_uart_interrupt;
    uart->irqaction.name    = "meson_uart";
    uart->irqaction.dev_id  = port;

    if ( setup_irq(uart->irq, 0, &uart->irqaction) != 0 )
    {
        dprintk(XENLOG_ERR, "Failed to allocated meson_uart IRQ %d\n",
                uart->irq);
        return;
    }

    val = meson_read(uart, AML_UART_CONTROL);

    val |= (AML_UART_RX_EN | AML_UART_TX_EN);
    meson_write(uart, AML_UART_CONTROL, val);

    val |= (AML_UART_RX_INT_EN | AML_UART_TX_INT_EN);
    meson_write(uart, AML_UART_CONTROL, val);

    val = (AML_UART_RECV_IRQ(1) | AML_UART_XMIT_IRQ(FIFO_MAX_SIZE / 2));
    meson_write(uart, AML_UART_MISC, val);
}

static void meson_uart_suspend(struct serial_port *port)
{
    BUG();
}

static void meson_uart_resume(struct serial_port *port)
{
    BUG();
}

static int meson_uart_tx_ready(struct serial_port *port)
{
    struct meson_uart *uart = port->uart;
    uint32_t cnt;

    /* Check for empty space in TX FIFO */
    if ( meson_read(uart, AML_UART_STATUS) & AML_UART_TX_FULL )
        return 0;

    /* Check number of data bytes stored in TX FIFO */
    cnt = meson_read(uart, AML_UART_STATUS);
    cnt = (cnt & AML_UART_TX_COUNT_MASK) >> AML_UART_TX_COUNT_SHIFT;
    ASSERT(cnt >= 0 && cnt <= FIFO_MAX_SIZE);

    return (FIFO_MAX_SIZE - cnt);
}

static void meson_uart_putc(struct serial_port *port, char c)
{
    struct meson_uart *uart = port->uart;

    meson_write(uart, AML_UART_WFIFO, (uint32_t)(unsigned char)c);
}

static int meson_uart_getc(struct serial_port *port, char *pc)
{
    struct meson_uart *uart = port->uart;
    uint32_t ufstat = meson_read(uart, AML_UART_STATUS);
    uint32_t count;

    count = (ufstat & AML_UART_RX_COUNT_MASK) >> AML_UART_RX_COUNT_SHIFT;

    /* Check for available data bytes in RX FIFO */
    if ( ufstat & AML_UART_RX_FULL || count )
    {
        *pc = meson_read(uart, AML_UART_RFIFO) & AML_UART_RX_DATA_MASK;
        return 1;
    }

    return 0;
}

static int __init meson_uart_irq(struct serial_port *port)
{
    struct meson_uart *uart = port->uart;

    return uart->irq;
}

static const struct vuart_info *meson_vuart_info(struct serial_port *port)
{
    struct meson_uart *uart = port->uart;

    return &uart->vuart;
}

static struct uart_driver __read_mostly meson_uart_driver = {
    .init_preirq  = meson_uart_init_preirq,
    .init_postirq = meson_uart_init_postirq,
    .endboot      = NULL,
    .suspend      = meson_uart_suspend,
    .resume       = meson_uart_resume,
    .tx_ready     = meson_uart_tx_ready,
    .putc         = meson_uart_putc,
    .getc         = meson_uart_getc,
    .irq          = meson_uart_irq,
    .vuart_info   = meson_vuart_info,
};

static int __init meson_uart_init(struct dt_device_node *dev, const void *data)
{
    const char *config = data;
    struct meson_uart *uart;
    int res;
    u64 addr, size;

    if ( strcmp(config, "") )
        printk("WARNING: UART configuration is not supported\n");

    uart = &meson_com;

    uart->clock_hz  = 24000000;
    uart->baud      = BAUD_AUTO;
    uart->data_bits = 8;
    uart->parity    = PARITY_NONE;
    uart->stop_bits = 1;

    res = dt_device_get_address(dev, 0, &addr, &size);
    if ( res )
    {
        printk("meson-uart: Unable to retrieve the base"
               " address of the UART\n");
        return res;
    }

    res = platform_get_irq(dev, 0);
    if ( res < 0 )
    {
        printk("meson-uart: Unable to retrieve the IRQ\n");
        return -EINVAL;
    }
    uart->irq = res;

    uart->regs = ioremap_nocache(addr, size);
    if ( !uart->regs )
    {
        printk("meson-uart: Unable to map the UART memory\n");
        return -ENOMEM;
    }

    uart->vuart.base_addr = addr;
    uart->vuart.size = size;
    uart->vuart.data_off = AML_UART_WFIFO;
    uart->vuart.status_off = AML_UART_STATUS;
    uart->vuart.status = AML_UART_TX_EMPTY;

    /* Register with generic serial driver. */
    serial_register_uart(SERHND_DTUART, &meson_uart_driver, uart);

    dt_device_set_used_by(dev, DOMID_XEN);

    return 0;
}

static const struct dt_device_match meson_dt_match[] __initconst =
{
    DT_MATCH_COMPATIBLE("amlogic, meson-uart"),
    { /* sentinel */ },
};

DT_DEVICE_START(meson, "Meson UART", DEVICE_SERIAL)
        .dt_match = meson_dt_match,
        .init = meson_uart_init,
DT_DEVICE_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
