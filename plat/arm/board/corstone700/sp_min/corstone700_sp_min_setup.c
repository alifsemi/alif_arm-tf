/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <plat/arm/common/plat_arm.h>
#include <lib/mmio.h>

extern int init_nor_flash(void);
extern int init_ospi_hyperram(void);

void plat_arm_sp_min_early_platform_setup(u_register_t arg0, u_register_t arg1,
			u_register_t arg2, u_register_t arg3)
{
	arm_sp_min_early_platform_setup((void *)arg0, arg1, arg2, (void *)arg3);
#if FLASH_EN
	/* Logic to reset OSPI1 flash on push button press (SW2) */
	/* Set 1.8V for I/O pin for GPIO in GPIO_CTRL register */
	mmio_write_32(0x1A609000, 0x1);
	/* Set bit 7 for output pin direction */
	mmio_write_32(0x42002004, 0x80);
	/* Delay for the pulse */
	volatile int count = 0;
	for (count = 0; count < 0xFFF ; count++);
	/* Set bit 7 to send high signal to GPIO pin */
	mmio_write_32(0x42002000, 0x80);
#endif
#if HYPRAM_EN
	init_ospi_hyperram();
#endif
}
