/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <plat/arm/common/arm_def.h>
#include <common/bl_common.h>
#include <plat/arm/common/plat_arm.h>
#include <plat/common/platform.h>
#include <platform_def.h>
#include <mhu.h>

/*
 * Table of regions to map using the MMU.
 * Replace or extend the below regions as required
 */

const mmap_region_t plat_arm_mmap[] = {
	ARM_MAP_SHARED_RAM,
	ARM_MAP_NS_DRAM1,
	CORSTONE700_MAP_DEVICE,
	{0}
};

/* Nothing to do*/
void __init plat_arm_pwrc_setup(void)
{
	mhu_secure_init();
}

unsigned int plat_get_syscnt_freq2(void)
{
	return FPGA_TIMER_BASE_FREQUENCY;
}

#ifdef PLAT_ARM_BOLT_FPGA
unsigned int plat_get_syscnt_freq2(void)
{
	/* Returns 10Mhz as timer base frequency */
	return BOLT_FPGA_TIMER_BASE_FREQUENCY;
}
#endif
