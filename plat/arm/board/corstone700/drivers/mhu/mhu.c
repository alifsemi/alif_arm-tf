/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <lib/bakery_lock.h>
#include <lib/mmio.h>
#include <plat_arm.h>
#include <platform_def.h>
#include <common/debug.h>
#include "mhu.h"

/* CPU MHU secure channel registers */
#define CPU_INTR_S_STAT		0x00
#define CPU_INTR_S_SET		0x0C

ARM_INSTANTIATE_LOCK;


#pragma weak plat_arm_pwrc_setup

/*
 * Slot 31 is reserved because the MHU hardware uses this register bit to
 * indicate a non-secure access attempt. The total number of available slots is
 * therefore 31 [30:0].
 */
#define MHU_MAX_SLOT_ID		30

void mhu_secure_message_start(unsigned int address, unsigned int slot_id)
{
	assert(slot_id <= MHU_MAX_SLOT_ID);
	arm_lock_get();

	/* Make sure any previous command has finished */
	while (mmio_read_32(address + CPU_INTR_S_STAT) &
						(1 << slot_id));
}

void mhu_secure_message_send(unsigned int address, unsigned int slot_id,
		uint32_t message)
{
	assert(slot_id <= MHU_MAX_SLOT_ID);
	assert(!(mmio_read_32(address + CPU_INTR_S_STAT) &
						(1 << slot_id)));

	MHU_V2_ACCESS_REQUEST(address);
	while (MHU_V2_IS_ACCESS_READY(address) == 0)
		;

	mmio_write_32(address + CPU_INTR_S_SET, message);

}

void mhu_secure_message_end(unsigned int address, unsigned int slot_id)
{
	assert(slot_id <= MHU_MAX_SLOT_ID);
	/*
	 * Clear any response we got by writing one in the relevant slot bit to
	 * the CLEAR register
	 */
	MHU_V2_CLEAR_REQUEST(address);

	arm_lock_release();
}

void __init mhu_secure_init(void)
{
	arm_lock_init();

	/*
	 * The STAT register resets to zero. Ensure it is in the expected state,
	 * as a stale or garbage value would make us think it's a message we've
	 * already sent.
	 */


	assert(mmio_read_32(PLAT_SDK700_MHU0_SEND + CPU_INTR_S_STAT) == 0);
}
