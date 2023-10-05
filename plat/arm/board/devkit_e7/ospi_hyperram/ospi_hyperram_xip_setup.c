/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include <arch_helpers.h>
#include <stddef.h>
#include <lib/mmio.h>
#include <common/debug.h>
#include <lib/xlat_tables/xlat_tables.h>
#include <ospi_hyperram_xip.h>
#include <ospi_drv.h>
#include <ospi_hram_reg_access.h>

#define OSPI_RESET_PIN			6
#define DDR_DRIVE_EDGE      		1
#define RXDS_DELAY			16
#define OSPI_BUS_SPEED			50000000 /* 50 MHz */
#define ISSI_WAIT_CYCLES		6

#define LPGPIO_BASE			0x42002000UL
#define LPGPIO_CTRL_BASE		0x42007000UL
#define GPIO_SWPORTA_DR_OFFSET		0x0
#define GPIO_SWPORTA_DDR_OFFSET		0x4
#define GPIO_INTMASK_OFFSET		0x34
#define GPIO_PIN_DIRECTION_INPUT	0
#define GPIO_PIN_DIRECTION_OUTPUT	1
#define PAD_CTRL_REN			0x01

#define PINMUX_BASE			0x1A603000
#define PADCTRL_REG(p,b,pad_value,pinmux_value)	(volatile uint32_t *) \
	(PINMUX_BASE + (p *32 + b *4)) = pad_value<<16 | pinmux_value
#define READ_PINCTRL_VALUE(po, pi)	*(volatile uint32_t *) \
	((uint8_t *)PINMUX_BASE + (po *32) + (pi*4))

static const ospi_hyperram_xip_config issi_config = {
	.instance       = OSPI_INSTANCE_0,
	.bus_speed      = OSPI_BUS_SPEED,
	/* No special initialization needed by the hyperram device */
	.hyperram_init  = NULL,
	.ddr_drive_edge = DDR_DRIVE_EDGE,
	.rxds_delay     = RXDS_DELAY,
	.wait_cycles    = ISSI_WAIT_CYCLES,
	.slave_select   = 0,
};

static ospi_cfg_t ospi_cfg;
static void setup_pinmux()
{
	uint32_t value;
	/* PADCTRL_REG(port, pin, pad_value, mux value) */
	*PADCTRL_REG(2, 0, PAD_CTRL_REN, 1);
	*PADCTRL_REG(2, 1, PAD_CTRL_REN, 1);
	*PADCTRL_REG(2, 2, PAD_CTRL_REN, 1);
	*PADCTRL_REG(2, 3, PAD_CTRL_REN, 1);
	*PADCTRL_REG(2, 4, PAD_CTRL_REN, 1);
	*PADCTRL_REG(2, 5, PAD_CTRL_REN, 1);
	*PADCTRL_REG(2, 6, PAD_CTRL_REN, 1);
	*PADCTRL_REG(2, 7, PAD_CTRL_REN, 1);
	*PADCTRL_REG(3, 0, PAD_CTRL_REN, 1);
	*PADCTRL_REG(3, 1, PAD_CTRL_REN, 1);
	*PADCTRL_REG(3, 2, PAD_CTRL_REN, 1);
	*PADCTRL_REG(1, 6, PAD_CTRL_REN, 1);
	mmio_write_32((LPGPIO_CTRL_BASE + (6 * 4)), 0);

	/* initialize */
	value =	mmio_read_32((LPGPIO_BASE + GPIO_INTMASK_OFFSET));
	mmio_write_32((LPGPIO_BASE + GPIO_INTMASK_OFFSET),
		      (value | (1 << OSPI_RESET_PIN)));

	/* set direction */
	value = mmio_read_32((LPGPIO_BASE + GPIO_SWPORTA_DDR_OFFSET));
	mmio_write_32((LPGPIO_BASE + GPIO_SWPORTA_DDR_OFFSET),
		      (value | (1 << OSPI_RESET_PIN)));

	/* set low state */
	value = mmio_read_32((LPGPIO_BASE + GPIO_SWPORTA_DR_OFFSET));
	mmio_write_32((LPGPIO_BASE + GPIO_SWPORTA_DR_OFFSET),
		      (value & ~(1 << OSPI_RESET_PIN)));

	/* set high state */
	value = mmio_read_32((LPGPIO_BASE + GPIO_SWPORTA_DR_OFFSET));
	mmio_write_32((LPGPIO_BASE + GPIO_SWPORTA_DR_OFFSET),
		      (value | (1 << OSPI_RESET_PIN)));
}

int ospi_hyperram_hw_init()
{
	ospi_cfg_t *ospi = &ospi_cfg;

	ospi->regs = (ospi_regs_t *) OSPI0_BASE;
	ospi->aes_regs = (aes_regs_t *) AES0_BASE;
	ospi->rx_buff[0] = 0;

	INFO("OSPI Version = %x\n", ospi->regs->spi_ver_id);

	/* Setup Clock */
	ospi_disable(ospi);
	writel(ospi, baudr, 0x8); /* OSPI_CLK/OSPI_BUS_SPEED [400MHz/50MHz] */
	writel(ospi, txd_drive_edge, 0x1);
	writel(ospi, rx_sample_dly, 0x0);
	ospi->aes_regs->aes_rxds_delay = 0x10;
	ospi_enable(ospi);

	/* Set 64byte wrap length (b01) along with default settings*/
	hyper_ram_command_write_conf_reg0(ospi, 0x8f1d);

	return 0;
}

int init_ospi_hyperram()
{
	uint32_t rt, rt2;

	/* Check CPUACLTR 28, 27, 26, and 25 bits are set to
	* make sure write-allocate lines allocate in the L1 or L2 cache */
	__asm__ __volatile__("mrrc p15, 0, %0, %1, c15" : "=r"(rt),"=r"(rt2));
	if ((rt & 0x1E000000UL) != 0x1E000000UL)
	{
		ERROR("Failed to allocate write-allocate in the L1/L2 cache\n");
		return -1;
	}

	setup_pinmux();
	ospi_hyperram_hw_init();
	if(ospi_hyperram_xip_init(&issi_config) < 0)
	{
		ERROR("HyperRAM OSPI XIP init failed\n");
		return -1;
	}
	INFO("HyperRAM configured successfully\n");
	return 0;
}
