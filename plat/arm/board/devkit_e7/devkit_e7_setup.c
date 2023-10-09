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
