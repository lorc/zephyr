/*
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_scif

#include <kernel.h>
#include <arch/cpu.h>
#include <init.h>
#include <device.h>
#include <soc.h>
#include <drivers/uart.h>

/*
 * UART SCIF register map structure
 */
struct scif_regs {
	uint16_t scsmr;	 /* 0x00 */
	uint16_t __pad1;

	uint8_t scbrr;	/* 0x04 */
	uint8_t __pad2[3];

	uint16_t scscr;	/* 0x08 */
	uint16_t __pad3;

	uint8_t scftdr;	 /* 0x0C */
	uint8_t __pad4[3];

	uint16_t scfsr;	/* 0x10 */
	uint16_t __pad5;

	uint8_t scfrdr;	 /* 0x14 */
	uint8_t __pad6[3];

	uint16_t scfcr;	/* 0x18 */
	uint16_t __pad7;

	uint16_t scfdr;	/* 0x1C */
	uint16_t __pad8;

	uint16_t scsptr; /* 0x20 */
	uint16_t __pad9;

	uint16_t sclsr; /* 0x24 */
	uint16_t __pad10;

	uint16_t dl; /* 0x30 */
	uint16_t __pad11;

	uint16_t cks; /* 0x34 */
	uint16_t __pad12;
};

#define SCIF_SCSCR_TOIE		BIT(2)
#define SCIF_SCSCR_REIE		BIT(3)
#define SCIF_SCSCR_RE		BIT(4)
#define SCIF_SCSCR_TE		BIT(5)
#define SCIF_SCSCR_RIE		BIT(6)
#define SCIF_SCSCR_TIE		BIT(7)
#define SCIF_SCSCR_TEIE		BIT(11)

#define SCIF_SCFSR_TEND		BIT(6)

#define SCIF_TX_FIFO_CNT(dev)  ((SCIF_REGS(dev)->scfdr >> 8) & 0x1F)
#define SCIF_RX_FIFO_CNT(dev)  ((SCIF_REGS(dev)->scfdr >> 0) & 0x1F)

/* Device data structure */
struct scif_data {
	uint32_t baud_rate;	/* Baud rate */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t irq_cb;
	void *irq_cb_data;
#endif
};

#define DEV_CFG(dev)							\
	((const struct uart_device_config * const)(dev)->config)
#define DEV_DATA(dev) \
	((struct scif_data *)(dev)->data)
#define SCIF_REGS(dev) \
	((volatile struct scif_regs  *)(DEV_CFG(dev))->base)

static void scif_enable(const struct device *dev)
{
	SCIF_REGS(dev)->scscr |= SCIF_SCSCR_TE | SCIF_SCSCR_RE;
}

static void scif_disable(const struct device *dev)
{
	SCIF_REGS(dev)->scscr &= ~(SCIF_SCSCR_TE | SCIF_SCSCR_RE);
}

static int scif_poll_in(const struct device *dev, unsigned char *c)
{
	if (SCIF_RX_FIFO_CNT(dev) == 0)
		return -1;

	/* got a character */
	*c = (unsigned char)SCIF_REGS(dev)->scfrdr;

	return 0;
}

static void scif_poll_out(const struct device *dev,
			  unsigned char c)
{
	/* Wait for space in FIFO */
	while (SCIF_TX_FIFO_CNT(dev) >= 0x10 ) {
		; /* Wait */
	}

	/* Send a character */
	SCIF_REGS(dev)->scftdr = (uint16_t)c;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int scif_fifo_fill(const struct device *dev,
				    const uint8_t *tx_data, int len)
{
	uint8_t num_tx = 0U;

	while ((SCIF_TX_FIFO_CNT(dev) < 0x10) &&
	       (len - num_tx > 0)) {
		SCIF_REGS(dev)->scftdr = tx_data[num_tx++];
	}
	return num_tx;
}

static int scif_fifo_read(const struct device *dev,
				    uint8_t *rx_data, const int len)
{
	uint8_t num_rx = 0U;

	while ((len - num_rx > 0) && SCIF_RX_FIFO_CNT(dev)) {
		rx_data[num_rx++] = SCIF_REGS(dev)->scfrdr;
	}

	return num_rx;
}

static void scif_irq_tx_enable(const struct device *dev)
{
	SCIF_REGS(dev)->scscr |= SCIF_SCSCR_TIE;
}

static void scif_irq_tx_disable(const struct device *dev)
{
	SCIF_REGS(dev)->scscr &= ~SCIF_SCSCR_TIE;
}

static int scif_irq_tx_complete(const struct device *dev)
{
	/* check for TX FIFO empty or TX is done */
	return (SCIF_REGS(dev)->scfsr & SCIF_SCFSR_TEND) ||
			(SCIF_TX_FIFO_CNT(dev) == 0);
}

static int scif_irq_tx_ready(const struct device *dev)
{
	return SCIF_TX_FIFO_CNT(dev) < 0x10;
}

static void scif_irq_rx_enable(const struct device *dev)
{
	SCIF_REGS(dev)->scscr |= SCIF_SCSCR_RIE;
}

static void scif_irq_rx_disable(const struct device *dev)
{
	SCIF_REGS(dev)->scscr &= SCIF_SCSCR_RIE;
}

static int scif_irq_rx_ready(const struct device *dev)
{
	return SCIF_RX_FIFO_CNT(dev) > 0;
}

static void scif_irq_err_enable(const struct device *dev)
{
	SCIF_REGS(dev)->scscr |= SCIF_SCSCR_TOIE | SCIF_SCSCR_REIE;
}

static void scif_irq_err_disable(const struct device *dev)
{
	SCIF_REGS(dev)->scscr &= ~(SCIF_SCSCR_TOIE | SCIF_SCSCR_REIE);
}

static int scif_irq_is_pending(const struct device *dev)
{
	return scif_irq_rx_ready(dev) || scif_irq_tx_ready(dev);
}

static int scif_irq_update(const struct device *dev)
{
	return 1;
}

static void scif_irq_callback_set(const struct device *dev,
					    uart_irq_callback_user_data_t cb,
					    void *cb_data)
{
	DEV_DATA(dev)->irq_cb = cb;
	DEV_DATA(dev)->irq_cb_data = cb_data;
}

void scif_isr(const struct device *dev)
{
	struct scif_data *data = DEV_DATA(dev);

	/* Verify if the callback has been registered */
	if (data->irq_cb) {
		data->irq_cb(dev, data->irq_cb_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api scif_driver_api = {
	.poll_in = scif_poll_in,
	.poll_out = scif_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = scif_fifo_fill,
	.fifo_read = scif_fifo_read,
	.irq_tx_enable = scif_irq_tx_enable,
	.irq_tx_disable = scif_irq_tx_disable,
	.irq_tx_ready = scif_irq_tx_ready,
	.irq_rx_enable = scif_irq_rx_enable,
	.irq_rx_disable = scif_irq_rx_disable,
	.irq_tx_complete = scif_irq_tx_complete,
	.irq_rx_ready = scif_irq_rx_ready,
	.irq_err_enable = scif_irq_err_enable,
	.irq_err_disable = scif_irq_err_disable,
	.irq_is_pending = scif_irq_is_pending,
	.irq_update = scif_irq_update,
	.irq_callback_set = scif_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static void scif_irq_config_func(const struct device *dev);

static int scif_init(const struct device *dev)
{
	/* disable the uart */
	scif_disable(dev);

	/* TODO: Set BR. At this we use BR configured by bootloader */

	/* /\* Set baud rate *\/ */
	/* ret = scif_set_baudrate(dev, DEV_CFG(dev)->sys_clk_freq, */
	/* 			      DEV_DATA(dev)->baud_rate); */
	/* if (ret != 0) { */
	/* 	return ret; */
	/* } */

	/* initialize all IRQs as masked */
	SCIF_REGS(dev)->scscr &= ~(SCIF_SCSCR_RIE | SCIF_SCSCR_TIE |
				   SCIF_SCSCR_REIE | SCIF_SCSCR_TEIE |
				   SCIF_SCSCR_TOIE );
	__ISB();

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* Set FIFO watermarks to 8 for tx and 1 for rx */
	SCIF_REGS(dev)->scfcr = 0;
	/* Connect IRQs */
	scif_irq_config_func(dev);
#endif
	scif_enable(dev);

	return 0;
}

static struct uart_device_config scif_cfg = {
	.base = (uint8_t *)DT_INST_REG_ADDR(0),
};

static struct scif_data scif_data = {
	.baud_rate = 0,
};

DEVICE_AND_API_INIT(scif,
		    DT_INST_LABEL(0),
		    &scif_init,
		    &scif_data,
		    &scif_cfg, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &scif_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void scif_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    scif_isr,
		    DEVICE_GET(scif),
		    0);
	irq_enable(DT_INST_IRQN(0));
}
#endif
