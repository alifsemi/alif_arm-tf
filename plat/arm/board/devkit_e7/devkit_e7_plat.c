/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include <platform_def.h>
#include <lib/mmio.h>
#include <common/debug.h>
#include <arch_helpers.h>

#define MICRO_UNIT                      1000000
#define MAX_REFCLK_COUNT                0xFFFFFFFFFFFFFFFFULL

/* @brief - configure pinmux and pad configuration for the chosen uart. */
void configure_devkit_uart_pinmux_padconf()
{
	uint32_t value;
	/* Enable peripheral and APB clocks */
	value = mmio_read_32(EXPMST0_CTRL_REG);
	mmio_write_32(EXPMST0_CTRL_REG, (value | 0xC0000000));

	/* Enable UART clock and select UART clock source */
	value = mmio_read_32(UART_CTRL_REG);
	mmio_write_32(UART_CTRL_REG, (value | 0x0000FFFF));

	/* Initialize pinmux, and pad config for UART4_B */
	value = mmio_read_32(PINMUX_UART_RX_ADDR);
	mmio_write_32(PINMUX_UART_RX_ADDR, (value | PINMUX_UART_RX_VAL));
	value = mmio_read_32(PINMUX_UART_TX_ADDR);
	mmio_write_32(PINMUX_UART_TX_ADDR, (value | PINMUX_UART_TX_VAL));
}

/* @brief Gives delay in microseconds.
 * parameters,
 * delay - delay value in microseconds.
 */
void delay_in_us(uint32_t delay)
{
	uint32_t cnt_clk_freq;
	uint64_t total_count, pre_timestamp, cur_timestamp, cur_count;
	/* Get system counter timer frequency */
	cnt_clk_freq = mmio_read_32(ARM_SYS_TIMCTL_BASE + CNTCTLBASE_CNTFRQ);
	/* Get total count value for the given microseconds */
	total_count = (delay * (cnt_clk_freq/MICRO_UNIT));

	cur_count = 0;

	/* Read the start counter value */
	cur_timestamp = mmio_read_64(ARM_SYS_CNTREAD_BASE);
	pre_timestamp = cur_timestamp;

	/* Loop until the counter value exceeds the required count */
	while (cur_count < total_count) {
		cur_timestamp = mmio_read_64(ARM_SYS_CNTREAD_BASE);
		dsb();
		if (cur_timestamp > pre_timestamp) {
			/* Increament count */
			cur_count += (cur_timestamp - pre_timestamp);
		} else {
			/* The 64bit counter overflowed so adjust */
			/* count by subtracting with the max value */
			cur_count += ((MAX_REFCLK_COUNT - pre_timestamp)
					+ cur_timestamp);
		}
		/* Update latest counter value */
		pre_timestamp = cur_timestamp;
	}
}
