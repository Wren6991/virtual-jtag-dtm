// Copyright (c) Luke Wren 2023
// SPDX-License-Identifier: Apache-2.0 

#ifndef _JTAG_VDTM_H
#define _JTAG_VDTM_H

#include <stdint.h>
#include <stdbool.h>
typedef unsigned int uint;

// Virtual RISC-V JTAG Debug Transport Module implementation. Push a raw JTAG
// bitstream in, and you will get callbacks for DMI read/write accesses
// performed by the virtual DTM.

struct jtag_vdtm;
typedef struct jtag_vdtm jtag_vdtm_t;

// Using u8 to reduce JTAG traffic overhead -- at least 7 bits of word-address
// are required for a standard Debug Module
typedef uint8_t dmi_addr_t;

// A function for the DTM to call when it wants to perform a DMI write
typedef void (*jtag_vdtm_write_callback)(dmi_addr_t addr, uint32_t data);

// A function for the DTM to call when it wants to perform a DMI read
typedef void (*jtag_vdtm_read_callback)(dmi_addr_t addr, uint32_t *data);

// Dynamically allocate a new instance, initialise it, and pass you a pointer
// (which is opaque to you -- only the implementation file has the struct
// definition)
jtag_vdtm_t *jtag_vdtm_create(uint32_t idcode);

void jtag_dtm_destroy(jtag_vdtm_t *dtm);

// IO functions. You can connect these up to e.g. JTAG bitbang macros in the
// CMSIS-DAP JTAG_DP.c code.
void jtag_vdtm_set_tck(jtag_vdtm_t *dtm, bool tck);

void jtag_vdtm_set_tms(jtag_vdtm_t *dtm, bool tck);

void jtag_vdtm_set_tdi(jtag_vdtm_t *dtm, bool tck);

bool jtag_vdtm_get_tdo(jtag_vdtm_t *dtm);

// Pass in functions which will be called by the DTM to implement DMI
// accesses. Currently these are blocking functions which always succeed.
void jtag_vdtm_set_write_callback(jtag_vdtm_t *dtm, jtag_vdtm_write_callback cb);

void jtag_vdtm_set_read_callback(jtag_vdtm_t *dtm, jtag_vdtm_read_callback cb);

#endif
