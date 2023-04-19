// Copyright (c) Luke Wren 2023
// SPDX-License-Identifier: Apache-2.0 

// Virtual RISC-V JTAG Debug Transport Module implementation

#include "jtag_vdtm.h"

#include <string.h>
#include <stdlib.h>

#define DUMP_DTM 1

#if DUMP_DTM
#include <stdio.h>
#endif

typedef enum jtag_tap_state {
	S_RESET      = 0,
	S_RUN_IDLE   = 1,
	S_SELECT_DR  = 2,
	S_CAPTURE_DR = 3,
	S_SHIFT_DR   = 4,
	S_EXIT1_DR   = 5,
	S_PAUSE_DR   = 6,
	S_EXIT2_DR   = 7,
	S_UPDATE_DR  = 8,
	S_SELECT_IR  = 9,
	S_CAPTURE_IR = 10,
	S_SHIFT_IR   = 11,
	S_EXIT1_IR   = 12,
	S_PAUSE_IR   = 13,
	S_EXIT2_IR   = 14,
	S_UPDATE_IR  = 15
} jtag_tap_state_t;

struct jtag_vdtm {
	uint8_t ir;
	uint64_t shifter;
	uint32_t idcode;
	jtag_tap_state_t tap_state;
	uint32_t dmi_rdata;
	jtag_vdtm_write_callback write_callback;
	jtag_vdtm_read_callback read_callback;
	bool tck;
	bool tms;
	bool tdi;
	bool tdo;
};

jtag_vdtm_t *jtag_vdtm_create(uint32_t idcode) {
	jtag_vdtm_t *dtm = malloc(sizeof(jtag_vdtm_t));
	if (!dtm)
		return dtm;
	memset(dtm, 0, sizeof(*dtm));
	dtm->idcode = idcode;
	return dtm;
}

void jtag_vdtm_set_write_callback(jtag_vdtm_t *dtm, jtag_vdtm_write_callback cb) {
	dtm->write_callback = cb;
}

void jtag_vdtm_set_read_callback(jtag_vdtm_t *dtm, jtag_vdtm_read_callback cb) {
	dtm->read_callback = cb;
}

void jtag_vdtm_set_tms(jtag_vdtm_t *dtm, bool tms) {
	dtm->tms = tms;
}

void jtag_vdtm_set_tdi(jtag_vdtm_t *dtm, bool tdi) {
	dtm->tdi = tdi;
}

bool jtag_vdtm_get_tdo(jtag_vdtm_t *dtm) {
	return dtm->tdo;
}

static void tck_posedge(jtag_vdtm_t *dtm);

static inline bool get_next_tdo(jtag_vdtm_t *dtm) {
	// Note this is evaluated at negedge, so it is based on the *new* TAP
	// state following the most recent posedge.
	if (dtm->tap_state == S_SHIFT_DR || dtm->tap_state == S_SHIFT_IR) {
		return dtm->shifter & 1;
	} else {
		return 0;
	}
}

void jtag_vdtm_set_tck(jtag_vdtm_t *dtm, bool tck) {
	if (tck && !dtm->tck) {
		tck_posedge(dtm);
#if DUMP_DTM > 1
		printf("STEP TMS=%d TDI=%d -> TDO=%d\n", dtm->tms, dtm->tdi, get_next_tdo(dtm));
#endif
	} else if (!tck && dtm->tck) {
		dtm->tdo = get_next_tdo(dtm);
	}
	dtm->tck = tck;
}

// ----------------------------------------------------------------------------
// JTAG interface

// Forward-declare the DTM core interface, to be used by the JTAG code:
static void handle_dmi_write(jtag_vdtm_t *dtm, uint64_t dr_shifter);
static uint64_t handle_dmi_read(jtag_vdtm_t *dtm);
static void handle_dtmcs_write(jtag_vdtm_t *dtm, uint64_t dr_shifter);
static uint64_t handle_dtmcs_read(jtag_vdtm_t *dtm);

static jtag_tap_state_t step_tap_fsm(jtag_tap_state_t state, bool tms) {
	switch (state) {
		case S_RESET      : return tms ? S_RESET     : S_RUN_IDLE  ;
		case S_RUN_IDLE   : return tms ? S_SELECT_DR : S_RUN_IDLE  ;

		case S_SELECT_DR  : return tms ? S_SELECT_IR : S_CAPTURE_DR;
		case S_CAPTURE_DR : return tms ? S_EXIT1_DR  : S_SHIFT_DR  ;
		case S_SHIFT_DR   : return tms ? S_EXIT1_DR  : S_SHIFT_DR  ;
		case S_EXIT1_DR   : return tms ? S_UPDATE_DR : S_PAUSE_DR  ;
		case S_PAUSE_DR   : return tms ? S_EXIT2_DR  : S_PAUSE_DR  ;
		case S_EXIT2_DR   : return tms ? S_UPDATE_DR : S_SHIFT_DR  ;
		case S_UPDATE_DR  : return tms ? S_SELECT_DR : S_RUN_IDLE  ;

		case S_SELECT_IR  : return tms ? S_RESET     : S_CAPTURE_IR;
		case S_CAPTURE_IR : return tms ? S_EXIT1_IR  : S_SHIFT_IR  ;
		case S_SHIFT_IR   : return tms ? S_EXIT1_IR  : S_SHIFT_IR  ;
		case S_EXIT1_IR   : return tms ? S_UPDATE_IR : S_PAUSE_IR  ;
		case S_PAUSE_IR   : return tms ? S_EXIT2_IR  : S_PAUSE_IR  ;
		case S_EXIT2_IR   : return tms ? S_UPDATE_IR : S_SHIFT_IR  ;
		case S_UPDATE_IR  : return tms ? S_SELECT_DR : S_RUN_IDLE  ;

		default:            return S_RESET;
	}
}

#define W_IR 5
#define IR_BYPASS 0x00
#define IR_IDCODE 0x01
#define IR_DTMCS  0x10
#define IR_DMI    0x11

#define ABITS (8 * sizeof(dmi_addr_t))
#define W_DMI (ABITS + 32 + 2)
#define W_DR W_DMI

static inline uint dr_len(uint ir) {
	switch (ir) {
		case IR_DTMCS:  return 32;
		case IR_DMI:    return W_DMI;
		case IR_IDCODE: return 32;
		// Including BYPASS:
		default:        return 1;
	}
}

static void tck_posedge(jtag_vdtm_t *dtm) {
	// Update DTM state based on current TAP FSM state
	switch (dtm->tap_state) {
	case S_RESET:
		dtm->ir = IR_IDCODE;
#if DUMP_DTM
		printf("TAP: RESET\n");
#endif
		break;
	case S_CAPTURE_IR:
		dtm->shifter = dtm->ir;
#if DUMP_DTM
		printf("TAP: CAPTURE IR -> %02x\n", dtm->ir);
#endif
		break;
	case S_SHIFT_IR:
		dtm->shifter = (dtm->shifter >> 1) | ((uint64_t)dtm->tdi << (W_IR - 1));
		break;
	case S_UPDATE_IR:
		dtm->ir = dtm->shifter;
#if DUMP_DTM
		printf("TAP: UPDATE  IR <- %02x\n", dtm->ir);
#endif
		break;
	case S_CAPTURE_DR:
		switch(dtm->ir) {
		case IR_BYPASS:
			dtm->shifter = 0;
			break;
		case IR_IDCODE:
			dtm->shifter = dtm->idcode;
			break;
		case IR_DTMCS:
			dtm->shifter = handle_dtmcs_read(dtm);
			break;
		case IR_DMI:
			dtm->shifter = handle_dmi_read(dtm);
			break;
		default:
			break;
		}
#if DUMP_DTM
		printf("TAP: CAPTURE DR -> %016llx\n", dtm->shifter);
#endif
		break;
	case S_SHIFT_DR:
		dtm->shifter = (dtm->shifter >> 1) | ((uint64_t)dtm->tdi << (dr_len(dtm->ir) - 1));
		break;
	case S_UPDATE_DR:
#if DUMP_DTM
		printf("TAP: UPDATE  DR <- %016llx\n", dtm->shifter);
#endif
		switch(dtm->ir) {
		case IR_DTMCS:
			handle_dtmcs_write(dtm, dtm->shifter);
			break;
		case IR_DMI:
			handle_dmi_write(dtm, dtm->shifter);
		default:
			break;
		}
		break;

	default:
		break;
	}

	// Set new TAP FSM state
	dtm->tap_state = step_tap_fsm(dtm->tap_state, dtm->tms);
}

// ----------------------------------------------------------------------------
// DTM core implementation

#define DMI_OP_WRITE 2
#define DMI_OP_READ 1
#define DMI_OP_NONE 0

static void handle_dmi_write(jtag_vdtm_t *dtm, uint64_t dr_shifter) {
	uint op = dr_shifter & 0x3;
	uint32_t wdata = (dr_shifter >> 2) & 0xffffffffu;
	dmi_addr_t addr = (dr_shifter >> 34) & ((1ull << ABITS) - 1);

	if (op == DMI_OP_WRITE && dtm->write_callback) {
		dtm->write_callback(addr, wdata);
	} else if (op == DMI_OP_READ && dtm->read_callback) {
		dtm->dmi_rdata = dtm->read_callback(addr);
	}
}

static uint64_t handle_dmi_read(jtag_vdtm_t *dtm) {
	return (uint64_t)dtm->dmi_rdata << 2;
}

static void handle_dtmcs_write(jtag_vdtm_t *dtm, uint64_t dr_shifter) {
	// TODO error handling
	(void)dtm;
	return;
}

// version=1 means the 0.13.2 version of the debug spec (the first ratified one)
#define DTMCS_VERSION 1ull
#define DTMCS_ABITS ((uint64_t)ABITS)
#define DTMCS_IDLE_HINT 0ull

static uint64_t handle_dtmcs_read(jtag_vdtm_t *dtm) {
	// TODO error handling
	(void)dtm;
	return
		DTMCS_VERSION   << 0 |
		DTMCS_ABITS     << 4 |
		DTMCS_IDLE_HINT << 12;
}
