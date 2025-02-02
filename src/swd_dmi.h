// Copyright (c) Luke Wren 2023
// SPDX-License-Identifier: Apache-2.0 

// An SWD implementation of a Debug Module Interface (the RISC-V name for the
// bus downstream of a Debug Transport Module)
//
// This code provides a bridge between the emulated JTAG-DTM, and a Debug
// Module behind a SWD DAP (SW-DP + APB-AP) inside the target SoC.

#ifndef _SWD_DMI_H
#define _SWD_DMI_H

#include <stdint.h>
#include <stdbool.h>

typedef unsigned int uint;

struct swd_dmi;
typedef struct swd_dmi swd_dmi_t;

// Dynamically allocate a DMI instance, and initialise its members (but do not
// attempt to connect the SWD link).
swd_dmi_t *swd_dmi_create(uint32_t targetid, uint apsel);

void swd_dmi_destroy(swd_dmi_t *dmi);

// Call repeatedly until a connection is established, at which point it will
// return 0. (Main reason for failure will be no target being connected!)
int swd_dmi_connect(swd_dmi_t *dmi);

// TODO errors

// Note these functions scale their addresses by four (Mem-AP uses byte
// addresses, and DM registers are nominally word-addressed)
void swd_dmi_write(swd_dmi_t *dmi, uint32_t addr, uint32_t data);

void swd_dmi_read(swd_dmi_t *dmi, uint32_t addr, uint32_t *data);

#endif
