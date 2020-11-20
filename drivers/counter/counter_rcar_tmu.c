/*
 * Copyright (c) 2020 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_tmu

#include <soc.h>
#include <sys/sys_io.h>
#include <drivers/counter.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/rcar_cpg_clock_control.h>

#define REG_TSTRT	(0x04)
#define REG_TCOR	(0x08)
#define REG_TCNT	(0x0C)
#define REG_TCR		(0x10)

#define TSTRT_STR0	BIT(0)

#define TCR_TPSC_DIV4	0
#define TCR_CKEG	0
#define TCR_UNIE	BIT(5)
#define TCR_UNF		BIT(8)

/* S3D2 (133Mhz) dividied by 4  */
#define TIMER_CLK_HZ	(133120000UL / 4)
#define TICK_PERIOD_NS (NSEC_PER_SEC / TIMER_CLK_HZ)

struct rcar_tmu_config {
	struct counter_config_info info;
	mem_addr_t base;
};

struct rcar_tmu_data {
	volatile counter_top_callback_t callback;
	volatile void *user_data;
};

#define DEV_CFG(dev) ((const struct rcar_tmu_config* const)(dev)->config)
#define DEV_REG(dev, offset) ((DEV_CFG(dev))->base + offset)

static void update_ctrl(const struct device *dev, uint16_t set, uint16_t reset)
{
	uint16_t val;

	val = sys_read16(DEV_REG(dev, REG_TCR));

	val &= ~reset;
	val |= set;

	sys_write16(val, DEV_REG(dev, REG_TCR));
}

static void rcar_tmu_isr(const struct device *dev)
{
	struct rcar_tmu_data *data = dev->data;

	if (! (sys_read16(DEV_REG(dev, REG_TCR)) & TCR_UNF)) {
		printk("rcar_tmu: spurious interrupt!\n");
		return;
	}

	/* Clear IRQ flag */
	update_ctrl(dev, 0, TCR_UNF);

	if (data->callback != NULL) {
		data->callback(dev, (void *)data->user_data);
	}
}

static int rcar_tmu_start(const struct device *dev)
{
	sys_write8(TSTRT_STR0, DEV_REG(dev, REG_TSTRT));

	return 0;
}

static int rcar_tmu_stop(const struct device *dev)
{
	sys_write8(0, DEV_REG(dev, REG_TSTRT));

	return 0;
}

static int rcar_tmu_get_value(const struct device *dev, uint32_t *ticks)
{

	*ticks = sys_read32(DEV_REG(dev, REG_TCNT));

	return 0;
}

static int rcar_tmu_set_top_value(const struct device *dev,
				  const struct counter_top_cfg *cfg)
{
	struct rcar_tmu_data *data = dev->data;

	data->callback = cfg->callback;
	data->user_data = cfg->user_data;

	sys_write32(cfg->ticks, DEV_REG(dev, REG_TCOR));
	sys_write32(cfg->ticks, DEV_REG(dev, REG_TCNT));

	if (cfg->callback != NULL) {
		/* (Re)enable interrupt */
		update_ctrl(dev, TCR_UNIE, 0);
	}

	return 0;
}

static uint32_t rcar_tmu_get_pending_int(const struct device *dev)
{
	return sys_read16(DEV_REG(dev, REG_TCR)) & TCR_UNF;
}

static uint32_t rcar_tmu_get_top_value(const struct device *dev)
{
	return sys_read32(DEV_REG(dev, REG_TCNT));
}

static uint32_t rcar_tmu_get_max_relative_alarm(const struct device *dev)
{
	return 0xFFFFFFFF;
}

static const struct counter_driver_api rcar_tmu_driver_api = {
	.start = rcar_tmu_start,
	.stop = rcar_tmu_stop,
	.get_value = rcar_tmu_get_value,
	.set_top_value = rcar_tmu_set_top_value,
	.get_pending_int = rcar_tmu_get_pending_int,
	.get_top_value = rcar_tmu_get_top_value,
	.get_max_relative_alarm = rcar_tmu_get_max_relative_alarm,
};

static int init(const struct device *dev);

static const struct rcar_tmu_config rcar_tmu_config = {
	.info = {
			.max_top_value = 0xFFFFFFFF,
			.freq = TIMER_CLK_HZ,
			.flags = 0,
			.channels = 1U,	 /* actually 3, but only one supported now */
		},
	.base = DT_INST_REG_ADDR(0),
};

static struct rcar_tmu_data data;

DEVICE_AND_API_INIT(rcar_tmu,
		    DT_INST_LABEL(0),
		    &init,
		    &data,
		    &rcar_tmu_config,
		    PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &rcar_tmu_driver_api);

static int init(const struct device *dev)
{
#ifdef CONFIG_CLOCK_CONTROL
	const struct device *clk_dev =
			device_get_binding(DT_LABEL(DT_INST_PHANDLE(0, clocks)));

	struct rcar_cpg_clk clk = {
		.mode = DT_INST_PHA_BY_IDX(0, clocks, 0, mode),
		.id = DT_INST_PHA_BY_IDX(0, clocks, 0, id),
	};

	clock_control_on(clk_dev, &clk);
#endif

	/* Configure timer: div 4 */
	sys_write16(TCR_TPSC_DIV4 | TCR_CKEG, DEV_REG(dev, REG_TCR));

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    rcar_tmu_isr, DEVICE_GET(rcar_tmu), 0);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}
