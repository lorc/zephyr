/*
 * Copyright (c) 2020 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <device.h>
#include <drivers/counter.h>
#include <sys/printk.h>

#define DELAY 10000
#define BREAK_DELAY 1000
#define THRESHOLD_US 30

struct counter_top_cfg top_cfg;

#define TIMER DT_LABEL(DT_NODELABEL(tmu))

#define HISTORY_SIZE 1024

static uint32_t history[HISTORY_SIZE];
static int hpos = 0;
static uint32_t freq;
static uint64_t period_ns;
static uint32_t gmax = 0;
static uint32_t gmin = ~0;

static unsigned long int_sqrt(unsigned long x)
{
	unsigned long b, m, y = 0;

	if (x <= 1)
		return x;

	m = 1UL << (BITS_PER_LONG - 2);
	while (m != 0) {
		b = y + m;
		y >>= 1;

		if (x >= b) {
			x -= b;
			y += m;
		}
		m >>= 2;
	}

	return y;
}

static void handle_history(void)
{
	int64_t total = 0;
	int64_t dev_sum = 0;
	int64_t mean;
        int64_t stddev;
        uint32_t lmin = ~0, lmax = 0;
	int i;
	int above_threshold = 0;
	const int threshold_ticks = THRESHOLD_US * 1000 / period_ns;

	hpos = 0;

	for (i = 0; i < HISTORY_SIZE; i++) {
		if (history[i] > lmax)
			lmax = history[i];
		if (history[i] < lmin)
			lmin = history[i];
		if (history[i] > threshold_ticks)
			above_threshold++;
		total += history[i];
	}
	mean = total / HISTORY_SIZE;

	for (i = 0; i < HISTORY_SIZE; i++)
		dev_sum += (mean - history[i]) * (mean - history[i]);
	stddev = int_sqrt(dev_sum / (HISTORY_SIZE - 1));

	printk("Mean: %lld (%llu ns) stddev: %lld (%llu ns) above thr: %d%% [%u (%llu ns) - %u (%llu ns)] global [%u (%llu ns) %u (%llu ns)] \n",
	       mean, mean * period_ns,
	       stddev, stddev * period_ns,
	       above_threshold * 100 / HISTORY_SIZE,
	       lmin, lmin * period_ns,
	       lmax, lmax * period_ns,
	       gmin, gmin * period_ns,
	       gmax, gmax * period_ns);
}

volatile int brk_count = 1;
void zphr_break_me(void)
{
	/* Break every Nth time, so we can recover after debugger break */

	if (++brk_count == 2)
	{
		//		__asm__ volatile("hlt #0");
		brk_count = 0;
	}
}

static void test_counter_interrupt_fn(const struct device *counter_dev,
                                      void *user_data)
{
  	/* struct counter_alarm_cfg *config = user_data; */
	uint32_t now_ticks, skew;
	int err;

	err = counter_get_value(counter_dev, &now_ticks);
	if (err) {
		printk("Failed to read counter value (err %d)", err);
		return;
	}

	skew = top_cfg.ticks - now_ticks;
	if (skew > gmax)
	  gmax = skew;
	if (skew < gmin)
	  gmin = skew;

	if (skew > counter_us_to_ticks(counter_dev, BREAK_DELAY))
		zphr_break_me();

	history[hpos++] = skew;
	if (hpos == HISTORY_SIZE)
		handle_history();
}

void main(void)
{
	const struct device *counter_dev;
	int err;

	printk("RT Eval app\n\n");
	counter_dev = device_get_binding(TIMER);
	if (counter_dev == NULL) {
		printk("TMU dev not found\n");
		return;
	}

	freq = counter_get_frequency(counter_dev);
	period_ns = NSEC_PER_SEC / freq;

	printk("Counter freq is %d Hz. Period is %lld ns\n", freq, period_ns);

	top_cfg.ticks = counter_us_to_ticks(counter_dev, DELAY);
	top_cfg.callback = test_counter_interrupt_fn;
	top_cfg.user_data = &top_cfg;

	err = counter_set_top_value(counter_dev, &top_cfg);
	printk("Set alarm in %u sec (%u ticks)\n",
	       (uint32_t)(counter_ticks_to_us(counter_dev,
					   top_cfg.ticks) / USEC_PER_SEC),
	       top_cfg.ticks);

	counter_start(counter_dev);

	if (-EINVAL == err) {
		printk("Alarm settings invalid\n");
	} else if (-ENOTSUP == err) {
		printk("Alarm setting request not supported\n");
	} else if (err != 0) {
		printk("Error\n");
	}

	while (1) {
		k_sleep(K_FOREVER);
	}
}
