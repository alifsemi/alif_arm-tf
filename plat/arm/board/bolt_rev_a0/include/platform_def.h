/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <plat/arm/common/arm_def.h>
#include <plat/arm/common/arm_spm_def.h>
#include <plat/common/common_def.h>
#include <lib/utils_def.h>
#include <plat/arm/board/common/v2m_def.h>

#define PLAT_ARM_CORSTONE700
#define PLAT_ARM_BOLT_REV_A0

#define CORSTONE700_MAX_CPUS_PER_CLUSTER	4
#define PLAT_ARM_CLUSTER_COUNT		1
#define CORSTONE700_MAX_PE_PER_CPU	1
#define PLATFORM_CORE_COUNT		(PLAT_ARM_CLUSTER_COUNT *       \
					CORSTONE700_MAX_CPUS_PER_CLUSTER *   \
					CORSTONE700_MAX_PE_PER_CPU)

/* Bolt REV_A0 Generic Timer Frequency */
#define BOLT_REV_A0_TIMER_BASE_FREQUENCY      100000000 /* 100Mhz */

#define PLAT_MAX_PWR_LVL		2

#define PLAT_ARM_TRUSTED_MAILBOX_BASE	ARM_TRUSTED_SRAM_BASE
#define PLAT_ARM_NSTIMER_FRAME_ID	U(1)

#define PLAT_ARM_BOOT_UART_BASE		0x1a510000
#define PLAT_ARM_BOOT_UART_CLK_IN_HZ	V2M_IOFPGA_UART0_CLK_IN_HZ
#define PLAT_ARM_RUN_UART_BASE		0x1a520000
#define PLAT_ARM_RUN_UART_CLK_IN_HZ	V2M_IOFPGA_UART1_CLK_IN_HZ

#define PLAT_ARM_NS_IMAGE_OFFSET	(ARM_DRAM1_BASE + UL(0x8000000))

/* CORSTONE700 Power controller base address*/
#define PWRC_BASE		UL(0x1A020000)
#define PSYSR_WK_SHIFT		24
#define PSYSR_WK_WIDTH		0x2
#define PSYSR_WK_MASK		((1U << PSYSR_WK_WIDTH) - 1U)
#define PSYSR_WK(x)		((x) >> PSYSR_WK_SHIFT) & PSYSR_WK_MASK
#define WKUP_COLD		U(0x0)
#define WKUP_RESET		U(0x1)
#define WKUP_PPONR		U(0x2)
#define WKUP_GICREQ		U(0x3)
#define PPOFFR_OFF		U(0x0)
#define PPONR_OFF		U(0x4)
#define PCOFFR_OFF		U(0x8)
#define PWKUPR_OFF		U(0xc)
#define PSYSR_OFF		U(0x10)
#define PWKUPR_WEN		BIT_32(31)
#define PSYSR_AFF_L2		BIT_32(31)
#define PSYSR_AFF_L1		BIT_32(30)
#define PSYSR_AFF_L0		BIT_32(29)
#define PSYSR_WEN		BIT_32(28)
#define PSYSR_PC		BIT_32(27)
#define PSYSR_PP		BIT_32(26)

# define PLAT_ARM_MMAP_ENTRIES          8
# define MAX_XLAT_TABLES                5

#define PLAT_ARM_TRUSTED_SRAM_SIZE	UL(0x00040000)  /* 256 KB */

/* The remaining Trusted SRAM is used to load the BL images */
#define ARM_BL_RAM_BASE			(ARM_SHARED_RAM_BASE +  \
					ARM_SHARED_RAM_SIZE)
#define ARM_BL_RAM_SIZE			(PLAT_ARM_TRUSTED_SRAM_SIZE -   \
					ARM_SHARED_RAM_SIZE)
#define PLATFORM_STACK_SIZE		UL(0x440)

#define PLAT_ARM_CRASH_UART_BASE	PLAT_ARM_RUN_UART_BASE

#define PLAT_ARM_CRASH_UART_CLK_IN_HZ	PLAT_ARM_RUN_UART_CLK_IN_HZ

#define CORSTONE700_DEVICE_BASE		(0x1A000000)
#define CORSTONE700_DEVICE_SIZE		(0x26000000)
#define CORSTONE700_MAP_DEVICE                       MAP_REGION_FLAT(        \
						CORSTONE700_DEVICE_BASE,      \
						CORSTONE700_DEVICE_SIZE,      \
						MT_DEVICE | MT_RW | MT_SECURE)

/* GIC related constants */
#define PLAT_ARM_GICD_BASE              0x1C010000
#define PLAT_ARM_GICC_BASE              0x1C02F000


/* MHUv2 Secure Channel receiver and sender */
#define PLAT_SDK700_MHU0_SEND            0x1B800000
#define PLAT_SDK700_MHU0_RECV            0x1B810000

#define CORSTONE700_IRQ_TZ_WDOG		64
#define CORSTONE700_IRQ_SEC_SYS_TIMER	65
/*
 * Define a list of Group 1 Secure and Group 0 interrupts as per GICv3
 * terminology. On a GICv2 system or mode, the lists will be merged and treated
 * as Group 0 interrupts.
 */
#define PLAT_ARM_G1S_IRQ_PROPS(grp) \
	ARM_G1S_IRQ_PROPS(grp), \
	INTR_PROP_DESC(CORSTONE700_IRQ_TZ_WDOG, GIC_HIGHEST_SEC_PRIORITY, \
			(grp), GIC_INTR_CFG_LEVEL), \
	INTR_PROP_DESC(CORSTONE700_IRQ_SEC_SYS_TIMER, \
			GIC_HIGHEST_SEC_PRIORITY, (grp), GIC_INTR_CFG_LEVEL)

#define PLAT_ARM_G0_IRQ_PROPS(grp)      ARM_G0_IRQ_PROPS(grp)
