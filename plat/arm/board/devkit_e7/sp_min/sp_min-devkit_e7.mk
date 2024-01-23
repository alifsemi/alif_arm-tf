#
# Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

# SP_MIN source files specific to FVP platform
BL32_SOURCES		+=	drivers/cfi/v2m/v2m_flash.c				\
				lib/utils/mem_region.c					\
				plat/arm/board/devkit_e7/e7_helpers.S			\
				plat/arm/board/corstone700/corstone700_topology.c			\
				plat/arm/board/corstone700/corstone700_security.c			\
				plat/arm/board/corstone700/corstone700_plat.c			\
				plat/arm/board/corstone700/corstone700_stack_protector.c	\
				plat/arm/board/devkit_e7/e7_pm.c			\
				plat/arm/board/corstone700/sp_min/corstone700_sp_min_setup.c	\
				${DEVKIT_E7_CPU_SOURCES}					\
				${DEVKIT_E7_GIC_SOURCES}

include plat/arm/common/sp_min/arm_sp_min.mk
