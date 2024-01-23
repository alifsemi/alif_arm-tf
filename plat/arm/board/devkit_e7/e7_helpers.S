/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <asm_macros.S>
#include <platform_def.h>

	.globl	plat_secondary_cold_boot_setup
	.globl	plat_get_my_entrypoint
	.globl	plat_is_my_cpu_primary
	.globl	plat_arm_calc_core_pos
	.globl	wfi_loop
	.globl	secondary_cpu_flags

	/* --------------------------------------------------------------------
	 * void plat_secondary_cold_boot_setup (void);
	 *
	 * For AArch32, cold-booting secondary CPUs is not yet
	 * implemented and they panic.
	 * --------------------------------------------------------------------
	 */
func plat_secondary_cold_boot_setup
	mrc p15, 0, r0, c0, c0, 5	/* Read MPIDR to get current cpu core */
	and	r0, r0, #0x3		/* cores could be 0 to 3 */
	sub	r0, r0, #1
	lsl	r0, r0, #2
	ldr	r1, =secondary_cpu_flags
	add	r1, r1, r0
	ldr	r1, [r1 , #0]		/* till psci comes in and boots you up */
	mov	r0, #0xADD		/* Magic word '0x00000ADD' */
	cmp	r1, r0			/* Test if the flag has been set */
	bne	wfi_loop		/* If not go to wfi and powerdown */
	ldr	r9, =sp_min_warm_entrypoint
	mov	pc, r9
endfunc plat_secondary_cold_boot_setup

func wfi_loop
		mrrc    15, 1, r0, r1, cr15
		bic     r0, r0, #0x40 		/* Disable SMP_EN bit in CPUECTLR reg */
		mcrr    15, 1, r0, r1, cr15
wfi_lp:		mov r0,r0
		wfi
		b	wfi_lp
endfunc wfi_loop

	.pushsection .data
	.align 2
secondary_cpu_flags:
	.rept	10		/* Max 3 secondary a32 cores for ensemble */
	.word	0
	.endr
	.popsection
	/* ---------------------------------------------------------------------
	 * unsigned long plat_get_my_entrypoint (void);
	 *
	 * Main job of this routine is to distinguish between a cold and warm
	 * boot.
	 * ---------------------------------------------------------------------
	 */
func plat_get_my_entrypoint

	mov	r0, #0
	bx	lr
endfunc plat_get_my_entrypoint

	/* -----------------------------------------------------
	 * unsigned int plat_is_my_cpu_primary (void);
	 *
	 * Find out whether the current cpu is the primary
	 * cpu.
	 * -----------------------------------------------------
	 */
func plat_is_my_cpu_primary
	ldcopr	r0, MPIDR
	ldr	r1, =MPIDR_AFFINITY_MASK
	and	r0, r1
	cmp	r0, #0
	moveq	r0, #1
	movne	r0, #0
	bx	lr
endfunc plat_is_my_cpu_primary

