// Copyright (c) Luke Wren 2023
// SPDX-License-Identifier: Apache-2.0 

#include "swd_dmi.h"

#include "probe.h"
#include "hardware/gpio.h"

#include <stdlib.h>
#include <string.h>

#define DMI_DEBUG 1
#define DMI_INFO 1

#if DMI_DEBUG || DMI_INFO
#include <stdio.h>
#endif

#if DMI_DEBUG
#define dmi_debug(format,args...) printf(format, ## args)
#else
#define dmi_debug(format,...) ((void)0)
#endif

#if DMI_INFO
#define dmi_info(format,args...) printf(format, ## args)
#else
#define dmi_info(format,...) ((void)0)
#endif

#define PWRUP_ACK_TIMEOUT 10000

struct swd_dmi {
	uint32_t addr_cache;
	uint32_t targetsel;
	uint apsel;
	bool addr_cache_valid;
};

swd_dmi_t *swd_dmi_create(uint32_t targetsel, uint apsel) {
	swd_dmi_t *dmi = malloc(sizeof(swd_dmi_t));
	if (!dmi)
		return dmi;
	memset(dmi, 0, sizeof(*dmi));
	dmi->targetsel = targetsel;
	dmi->apsel = apsel;
	return dmi;
}

// ----------------------------------------------------------------------------
// IO functions

// TODO these should probably be passed as a function table to swd_dmi_create()
// TODO use PIO instead of bitbang

#define PROBE_PIN_SWCLK 2
#define PROBE_PIN_SWDIO 3

static inline void set_swdo(bool x) {
	gpio_put(PROBE_PIN_SWDIO, x);
}

static inline void set_swdo_en(bool x) {
	gpio_set_dir(PROBE_PIN_SWDIO, x);
}

static inline void set_swclk(bool x) {
	gpio_put(PROBE_PIN_SWCLK, x);
}

static inline bool get_swdi(void) {
	return gpio_get(PROBE_PIN_SWDIO);
}

static inline void bitbang_delay(void) {
	// 12 cycles (~0.1 us @ 125 MHz) -> ~5 MHz SWCLK
	asm volatile (
		"   b 1f\n"
		"1: b 1f\n"
		"1: b 1f\n"
		"1: b 1f\n"
		"1: b 1f\n"
		"1: b 1f\n"
		"1     :\n"
	);
}

static void put_bits(const uint8_t *tx, int n_bits) {
	set_swdo_en(1);
	uint8_t shifter = 0;
	for (int i = 0; i < n_bits; ++i) {
		if (i % 8 == 0)
			shifter = tx[i / 8];
		else
			shifter >>= 1;
		set_swdo(shifter & 1u);
		bitbang_delay();
		set_swclk(1);
		bitbang_delay();
		set_swclk(0);
	}

}

static void get_bits(uint8_t *rx, int n_bits) {
	uint8_t shifter = 0;
	set_swdo_en(0);
	for (int i = 0; i < n_bits; ++i) {
		bitbang_delay();
		bool sample = get_swdi();
		set_swclk(1);
		bitbang_delay();
		set_swclk(0);

		shifter = (shifter >> 1) | (sample << 7);
		if (i % 8 == 7)
			rx[i / 8] = shifter;
	}
	if (n_bits % 8 != 0) {
		rx[n_bits / 8] = shifter >> (8 - n_bits % 8);
	}
}

static void hiz_clocks(int n_bits) {
	set_swdo_en(0);
	for (int i = 0; i < n_bits; ++i) {
		bitbang_delay();
		set_swclk(1);
		bitbang_delay();
		set_swclk(0);
	}
}

// ----------------------------------------------------------------------------
// SWD helpers

// Again, cribbing heavily from OpenDAP test utils

typedef enum ap_dp {
	DP = 0,
	AP = 1
} ap_dp_t;

typedef enum swd_status {
	OK           = 1,
	WAIT         = 2,
	FAULT        = 4,
	DISCONNECTED = 7
} swd_status_t;

static uint8_t swd_header(ap_dp_t ap_ndp, bool read_nwrite, uint8_t addr) {
	addr &= 0x3;
	uint8_t parity = (addr >> 1) ^ (addr & 1) ^ (uint8_t)read_nwrite ^ (uint8_t)ap_ndp;
	return
		1u << 0 |                   // Start
		(uint8_t)ap_ndp << 1 |      // APnDP
		(uint8_t)read_nwrite << 2 | // RnW
		addr << 3 |                 // A[3:2]
		parity << 5 |               // parity
		0u << 6 |                   // Stop
		1u << 7;                    // Park
}

static void swd_targetsel(uint32_t id) {
	uint8_t header = swd_header(DP, 0, 3);
	put_bits(&header, 8);
	// No response to TARGETSEL.
	hiz_clocks(5);
	uint8_t txbuf[4];
	for (int i = 0; i < 4; ++i)
		txbuf[i] = (id >> i * 8) & 0xff;
	put_bits(txbuf, 32);
	// Parity
	txbuf[0] = 0;
	for (int i = 0; i < 32; ++i)
		txbuf[0] ^= (id >> i) & 0x1;
	put_bits(txbuf, 1);
}

// Only support ORUNDETECT=1 reads and writes (i.e. the good ones) -- this is
// safe because the writes required to set ORUNDETECT can be constructed to
// not fault.
static inline swd_status_t swd_read(ap_dp_t ap_ndp, uint8_t addr, uint32_t *data) {
	uint8_t header = swd_header(ap_ndp, 1, addr);
	put_bits(&header, 8);
	hiz_clocks(1);
	uint8_t status;
	get_bits(&status, 3);
	uint8_t rxbuf[4];
	get_bits(rxbuf, 32);
	*data = 0;
	for (int i = 0; i < 4; ++i)
		*data = (*data >> 8) | ((uint32_t)rxbuf[i] << 24);
	// Just discard parity bit -- have a separate test for that.
	get_bits(rxbuf, 1);
	// Turnaround for next packet header
	hiz_clocks(1);
	dmi_debug("  SWD R %cP:%x -> %08lx\n", "DA"[(int)ap_ndp], 4 * addr, *data);
	return (swd_status_t)status;
}

static inline swd_status_t swd_write(ap_dp_t ap_ndp, uint8_t addr, uint32_t data) {
	uint8_t header = swd_header(ap_ndp, 0, addr);
	put_bits(&header, 8);
	hiz_clocks(1);
	uint8_t status;
	get_bits(&status, 3);
	hiz_clocks(1);
	uint8_t txbuf[4];
	for (int i = 0; i < 4; ++i)
		txbuf[i] = (data >> i * 8) & 0xff;
	put_bits(txbuf, 32);
	// Parity
	txbuf[0] = 0;
	for (int i = 0; i < 32; ++i)
		txbuf[0] ^= (data >> i) & 0x1;
	put_bits(txbuf, 1);
	dmi_debug("  SWD W %cP:%x <- %08lx\n", "DA"[(int)ap_ndp], 4 * addr, data);
	return (swd_status_t)status;
}

// ----------------------------------------------------------------------------
// DMI implementation

// We're essentially implementing a tiny SWD host inside of the probe
// firmware. There is probably a lot of overlap here with CMSIS-DAP's SWD
// support, but this is still PoC stage.
//
// Link state management is as follows: either it's working, so do nothing, or
// it's not, so turn it off and on again and hope for the best. If that
// doesn't work, do it again until it does.
//
// Connection sequence is as follows:
//
// - A line reset, then SWD-to-Dormant. (If the link is up, down it.)
// - Dormant-to-SWD
// - Line reset
// - TARGETSEL using provided value
// - Read DPIDR to exit reset state
// - Write all ABORT bits
// - Write CDBGPWRUPREQ, CSYSPWRUPREQ, ORUNDETECT = 1
// - Poll for CDBGPWRUPACK/CSYSPWRUPACK (max _n_ times)
// - Check AP ID register indicates a Mem-AP
// - Set up SELECT to point to the useful Mem-AP  registers (CSW TAR DRW)
//
// Reference: ADIv5.2 spec IHI0031F Figure B5-4
// "SWJ-DP selection of JTAG, SWD, and dormant states"

#define DP_REG_DPIDR      0
#define DP_REG_ABORT      0
#define DP_REG_CTRL_STAT  1
#define DP_REG_SELECT     2
#define DP_REG_RDBUF      3
#define DP_REG_TARGETSEL  3

#define DP_BANK_CTRL_STAT 0

#define DP_CTRL_STAT_CSYSPWRUPACK (1u << 31)
#define DP_CTRL_STAT_CSYSPWRUPREQ (1u << 30)
#define DP_CTRL_STAT_CDBGPWRUPACK (1u << 29)
#define DP_CTRL_STAT_CDBGPWRUPREQ (1u << 28)
#define DP_CTRL_STAT_ORUNDETECT   (1u << 0)

#define AP_REG_CSW   0
#define AP_REG_TAR   1
#define AP_REG_DRW   3
#define AP_REG_IDR   3

#define AP_BANK_CSW  (0 << 4)
#define AP_BANK_TAR  (0 << 4)
#define AP_BANK_DRW  (0 << 4)
#define AP_BANK_IDR  (0xf << 4)

// CLASS=8 (Mem-AP) TYPE=2 (APB2/APB3)
#define APIDR_EXPECTED_MASK 0x1e00f
#define APIDR_EXPECTED_DATA 0x10002

static const uint8_t link_down_up[] = {
	// Line reset: at least 50 cycles (56 here)
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	// SWD-to-Dormant
	0xbc, 0xe3,
	// Start of Dormant-to-SWD: Resync the LFSR
	0xff,
	// A 0-bit, then 127 bits of LFSR output
	0x92, 0xf3, 0x09, 0x62,
	0x95, 0x2d, 0x85, 0x86,
	0xe9, 0xaf, 0xdd, 0xe3,
	0xa2, 0x0e, 0xbc, 0x19,
	// Four zero-bits, 8 bits of select sequence, four more zeroes
	0xa0, 0x01,
	// A line reset (50 cyc high) then at least 2 zeroes
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03
};

static const uint link_down_up_bits = sizeof(link_down_up) * 8 - 4;

int swd_dmi_connect(swd_dmi_t *dmi) {
	dmi_debug("swd_dmi_connect targetsel=%08lx apsel=%u\n", dmi->targetsel, dmi->apsel);

	gpio_init(PROBE_PIN_SWDIO);
	gpio_init(PROBE_PIN_SWCLK);
	gpio_set_dir(PROBE_PIN_SWCLK, 1);

	dmi->addr_cache_valid = false;
	// Drive the fixed link cycling sequence, which should put us in Reset
	put_bits(link_down_up, link_down_up_bits);

	// TARGETSEL puts any non-matching DPs in the Reset state into the
	// Deselected state (which is functionally similar to the lockout state).
	// Note there is no response to TARGETSEL.
	if (dmi->targetsel != 0)
		swd_targetsel(dmi->targetsel);

	// DPIDR read required to leave Reset state -- ignore the value, since
	// anything that responds after TARGETSEL is assumed the correct target.
	uint32_t data;
	swd_status_t status = swd_read(DP, DP_REG_DPIDR, &data);
	if (status != OK) {
		dmi_debug("DPIDR read failed\n");
		return -1;
	}
	dmi_debug("Read DPIDR: %08lx\n", data);

	// Clear any outstanding errors via ABORT so that SELECT becomes writable
	status = swd_write(DP, DP_REG_ABORT, 0x1e);
	if (status != OK) {
		return -1;
	}

	// Power up before attempting AP accesses (also take this opportunity to
	// set ORUNDETECT as we don't support legacy SWDv1 fault handling)
	const uint32_t pwr_req_bits = DP_CTRL_STAT_CSYSPWRUPREQ | DP_CTRL_STAT_CDBGPWRUPREQ;
	const uint32_t pwr_ack_bits = DP_CTRL_STAT_CSYSPWRUPACK | DP_CTRL_STAT_CDBGPWRUPACK;
	status = swd_write(DP, DP_REG_SELECT, DP_BANK_CTRL_STAT);
	if (status != OK) {
		return -1;
	}
	status = swd_write(DP, DP_REG_CTRL_STAT,
		pwr_req_bits | DP_CTRL_STAT_ORUNDETECT);
	if (status != OK) {
		return -1;
	}
	int timeout = 0;
	for (; timeout < PWRUP_ACK_TIMEOUT; ++timeout) {
		status = swd_read(DP, DP_REG_CTRL_STAT, &data);
		if (status != OK) {
			return -1;
		}
		if ((data & pwr_ack_bits) == pwr_ack_bits) {
			break;
		}
	}
	if (timeout == PWRUP_ACK_TIMEOUT) {
		dmi_info("PWRUPACK timed out\n");
		return -1;
	}

	// Have a quick squint at the designated AP and check it is a Mem-AP
	(void)swd_write(DP, DP_REG_SELECT, AP_BANK_IDR | (dmi->apsel << 24));
	// TODO from this point forward we should be checking WAIT responses
	(void)swd_read(AP, AP_REG_IDR, &data);
	status = swd_read(DP, DP_REG_RDBUF, &data);
	if (status != OK) {
		return -1;
	}
	if ((data & APIDR_EXPECTED_MASK) != APIDR_EXPECTED_DATA) {
		dmi_info("Bad APIDR: %08lx\n", data);
		return -1;
	}
	dmi_debug("Read APIDR: %08lx\n", data);

	// Set up SELECT to point to CSW/TAR/DRW. Note we don't use the BDx
	// registers as they seem unlikely to be profitable based on the RISC-V
	// DM memory map, and on the additional AP bank switching they entail.
	status = swd_write(DP, DP_REG_SELECT, AP_BANK_CSW | (dmi->apsel << 24));
	if (status != OK) {
		return -1;
	}

	return 0;
}

static inline void set_addr(swd_dmi_t *dmi, uint32_t addr) {
	if (dmi->addr_cache_valid && dmi->addr_cache == addr) {
		dmi_debug("TAR cache hit\n");
	} else {
		dmi_debug("TAR <- %08lx\n", addr);
		swd_write(AP, AP_REG_TAR, addr);
		dmi->addr_cache_valid = true;
		dmi->addr_cache = addr;
	}
}

void swd_dmi_write(swd_dmi_t *dmi, uint32_t addr, uint32_t data) {
	addr <<= 2;
	// TODO wait states -- leaving them out for now as the internal DMI is
	// assumed to have pretty fast access. It's just an APB bus going
	// straight to the DM.
	set_addr(dmi, addr);
	(void)swd_write(AP, AP_REG_DRW, data);
};

void swd_dmi_read(swd_dmi_t *dmi, uint32_t addr, uint32_t *data) {
	addr <<= 2;
	set_addr(dmi, addr);
	(void)swd_read(AP, AP_REG_DRW, data);
	(void)swd_read(DP, DP_REG_RDBUF, data);
}
