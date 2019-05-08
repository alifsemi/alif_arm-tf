/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MHU_H
#define MHU_H

#include <stdint.h>

/* MHUv2 Control Registers Offsets */
#define MHU_V2_MSG_CFG_OFFSET                   0xF80
#define MHU_V2_ACCESS_REQ_OFFSET                0xF88
#define MHU_V2_ACCESS_READY_OFFSET              0xF8C

#define MHU_V2_ACCESS_REQUEST(addr)     \
        mmio_write_32((addr) + MHU_V2_ACCESS_REQ_OFFSET, 0x1)

#define MHU_V2_CLEAR_REQUEST(addr)      \
        mmio_write_32((addr) + MHU_V2_ACCESS_REQ_OFFSET, 0x0)

#define MHU_V2_IS_ACCESS_READY(addr)    \
        (mmio_read_32((addr) + MHU_V2_ACCESS_READY_OFFSET) & 0x1)

void mhu_secure_message_start(unsigned int address,unsigned int slot_id);
void mhu_secure_message_send(unsigned int address,unsigned int slot_id, uint32_t message);
void mhu_secure_message_end(unsigned int address,unsigned int slot_id);
void mhu_secure_init(void);

#endif /* MHU_H */
