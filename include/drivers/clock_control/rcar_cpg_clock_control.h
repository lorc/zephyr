/*
 * Copyright (c) 2020 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CPG_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CPG_CLOCK_CONTROL_H_

#include <drivers/clock_control.h>

struct rcar_cpg_clk {
	uint32_t mode;
	uint32_t id;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CPG_CLOCK_CONTROL_H_ */
