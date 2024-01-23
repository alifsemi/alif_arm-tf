/*
 * Copyright (c) 2023, Alif Semiconductor. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/arm/gicv3.h>
#include <lib/extensions/spe.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <plat/arm/common/arm_config.h>
#include <plat/arm/common/plat_arm.h>
#include <plat/common/platform.h>
#include <platform_def.h>

#include "../drivers/arm/gic/v3/gicv3_private.h"

#if ARM_RECOM_STATE_ID_ENC
/*
 *  The table storing the valid idle power states. Ensure that the
 *  array entries are populated in ascending order of state-id to
 *  enable us to use binary search during power state validation.
 *  The table must be terminated by a NULL entry.
 */
const unsigned int arm_pm_idle_states[] = {
	/* State-id - 0x01 */
	arm_make_pwrstate_lvl1(ARM_LOCAL_STATE_RUN, ARM_LOCAL_STATE_RET,
			ARM_PWR_LVL0, PSTATE_TYPE_STANDBY),
	/* State-id - 0x02 */
	arm_make_pwrstate_lvl1(ARM_LOCAL_STATE_RUN, ARM_LOCAL_STATE_OFF,
			ARM_PWR_LVL0, PSTATE_TYPE_POWERDOWN),
	/* State-id - 0x22 */
	arm_make_pwrstate_lvl1(ARM_LOCAL_STATE_OFF, ARM_LOCAL_STATE_OFF,
			ARM_PWR_LVL1, PSTATE_TYPE_POWERDOWN),
	/* State-id - 0x222 */
	arm_make_pwrstate_lvl2(ARM_LOCAL_STATE_OFF, ARM_LOCAL_STATE_OFF,
		ARM_LOCAL_STATE_OFF, ARM_PWR_LVL2, PSTATE_TYPE_POWERDOWN),
	0,
};
#endif
static void ensemble_power_domain_on_finish_common(const psci_power_state_t *target_state)
{

	assert(target_state->pwr_domain_state[ARM_PWR_LVL0] ==
					ARM_LOCAL_STATE_OFF);

	/* Perform the common system specific operations */
	if (target_state->pwr_domain_state[ARM_PWR_LVL2] ==
						ARM_LOCAL_STATE_OFF)
		arm_system_pwr_domain_resume();

}

extern unsigned int secondary_cpu_flags[3];
/*******************************************************************************
 * FVP handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 ******************************************************************************/
static int ensemble_pwr_domain_on(u_register_t mpidr)
{
	int rc = PSCI_E_SUCCESS;
	unsigned int cpu = mpidr, val;

#define HOST_BASE_SYS_CTRL              0x1A010000
#define PE_CONFIG(cpu)                  ((cpu) * 0x10 + 0x0)
#define PE_RVBARADDR_LW(cpu)            ((cpu) * 0x10 + 0x4)
#define HOST_CPU_BOOT_MSK               0x300
#define HOST_CPU_WAKEUP                 0x308

	/* Set the flag so that core jumps to sp_min_warm_boot */
	secondary_cpu_flags[cpu - 1] = 0x000ADD;

	flush_dcache_range((uint32_t)&secondary_cpu_flags,
		sizeof(secondary_cpu_flags));

	val = mmio_read_32(HOST_BASE_SYS_CTRL + HOST_CPU_BOOT_MSK);
	val |= (1 << cpu);
	mmio_write_32(HOST_BASE_SYS_CTRL + HOST_CPU_BOOT_MSK, val);

	val = mmio_read_32(HOST_BASE_SYS_CTRL + HOST_CPU_WAKEUP);
	val |= (1 << cpu);
	mmio_write_32(HOST_BASE_SYS_CTRL + HOST_CPU_WAKEUP, val);

	dsb();
	return rc;
}

/*******************************************************************************
 * FVP handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
static void ensemble_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	ensemble_power_domain_on_finish_common(target_state);

	/* Enable the gic cpu interface */
	plat_arm_gic_pcpu_init();

	/* Program the gic per-cpu distributor or re-distributor interface */
	plat_arm_gic_cpuif_enable();
}

/*******************************************************************************
 * Export the platform handlers via plat_arm_psci_pm_ops. The ARM Standard
 * platform layer will take care of registering the handlers with PSCI.
 ******************************************************************************/
plat_psci_ops_t plat_arm_psci_pm_ops = {
	.pwr_domain_on = ensemble_pwr_domain_on,
	.pwr_domain_on_finish = ensemble_pwr_domain_on_finish,
};

const plat_psci_ops_t *plat_arm_psci_override_pm_ops(plat_psci_ops_t *ops)
{
	return ops;
}
