/*
 * Copyright (c) 2020 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT renesas_r8a7795_cpg_mssr

#include <soc.h>
#include <sys/sys_io.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/rcar_cpg_clock_control.h>

struct cpg_device_config {
	mem_addr_t base;
};

#define CPG_REG_FROM_CLK_ID(clk_id) (clk_id / 100)
#define CPG_BIT_FROM_CLK_ID(clk_id) (clk_id % 100)

#define SMSTPCR_OFFSET 0x130
#define DEV_CFG(dev) ((const struct cpg_device_config* const)(dev)->config)
#define SMSTPCR_REG(dev, n) ((DEV_CFG(dev))->base + SMSTPCR_OFFSET + 4 * n)

static int clk_off(const struct device *dev, clock_control_subsys_t subsys)
{
	struct rcar_cpg_clk *clk = subsys;
	uint32_t val;

	val = sys_read32(SMSTPCR_REG(dev, CPG_REG_FROM_CLK_ID(clk->id)));
	val |= BIT(CPG_BIT_FROM_CLK_ID(clk->id));
	sys_write32(val, SMSTPCR_REG(dev, CPG_REG_FROM_CLK_ID(clk->id)));

	return 0;
}

static int clk_on(const struct device *dev, clock_control_subsys_t subsys)
{
	struct rcar_cpg_clk *clk = subsys;
	uint32_t val;

	val = sys_read32(SMSTPCR_REG(dev, CPG_REG_FROM_CLK_ID(clk->id)));
	val &= ~BIT(CPG_BIT_FROM_CLK_ID(clk->id));
	sys_write32(val, SMSTPCR_REG(dev, CPG_REG_FROM_CLK_ID(clk->id)));

	return 0;
}

static int clk_async_on(const struct device *dev, clock_control_subsys_t subsys,
			clock_control_cb_t cb, void *user_data)
{
	clk_on(dev, subsys);

	if (cb)
		cb(dev, subsys, user_data);

	return 0;
}

static int clk_init(const struct device *dev)
{
	return 0;
}

static const struct clock_control_driver_api clock_control_api = {
	.on = clk_on,
	.off = clk_off,
	.async_on = clk_async_on,
};

static struct cpg_device_config config = {
	.base = DT_INST_REG_ADDR(0),
};

DEVICE_AND_API_INIT(clock_rcar_cph, DT_INST_LABEL(0),
		    clk_init, NULL, &config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_OBJECTS,
		    &clock_control_api);
