/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2014  Florian Kerle <ikcalb>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements Atmel SAM C target specific functions for
 * detecting the device, providing the XML memory map and Flash memory
 * programming.
 *
 * Tested with
 * WIP!
 * 
 * 
 * 
 * *
 */
/* Refer to the SAM C20/21 Datasheets. i.e.:
 * https://ww1.microchip.com/downloads/en/DeviceDoc/SAM_C20_C21_Family_Data_Sheet_60001479G.pdf
 * particularly Sections 13. DSU and 27. NVMCTRL
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

static int samc_flash_erase(struct target_flash *t, target_addr addr, size_t len);
static int samc_flash_write(struct target_flash *f,
                            target_addr dest, const void *src, size_t len);

bool samc_cmd_erase_all(target *t, int argc, const char **argv);
static bool samc_cmd_lock_flash(target *t, int argc, const char **argv);
static bool samc_cmd_unlock_flash(target *t, int argc, const char **argv);
static bool samc_cmd_unlock_bootprot(target *t, int argc, const char **argv);
static bool samc_cmd_lock_bootprot(target *t, int argc, const char **argv);
static bool samc_cmd_read_userrow(target *t, int argc, const char **argv);
static bool samc_cmd_serial(target *t, int argc, const char **argv);
static bool samc_cmd_mbist(target *t, int argc, const char **argv);
static bool samc_cmd_ssb(target *t, int argc, const char **argv);

const struct command_s samc_cmd_list[] = {
	{"erase_mass", (cmd_handler)samc_cmd_erase_all, "Erase entire flash memory"},
	{"lock_flash", (cmd_handler)samc_cmd_lock_flash, "Locks flash against spurious commands"},
	{"unlock_flash", (cmd_handler)samc_cmd_unlock_flash, "Unlocks flash"},
	{"lock_bootprot", (cmd_handler)samc_cmd_lock_bootprot, "Lock the boot protections to maximum"},
	{"unlock_bootprot", (cmd_handler)samc_cmd_unlock_bootprot, "Unlock the boot protections to minimum"},
	{"user_row", (cmd_handler)samc_cmd_read_userrow, "Prints user row from flash"},
	{"serial", (cmd_handler)samc_cmd_serial, "Prints serial number"},
	{"mbist", (cmd_handler)samc_cmd_mbist, "Runs the built-in memory test"},
	{"set_security_bit", (cmd_handler)samc_cmd_ssb, "Sets the Security Bit"},
	{NULL, NULL, NULL}
};

/* Non-Volatile Memory Controller (NVMC) Parameters */
#define SAMC_ROW_SIZE			256
#define SAMC_PAGE_SIZE			64

/* -------------------------------------------------------------------------- */
/* Non-Volatile Memory Controller (NVMC) Registers */
/* -------------------------------------------------------------------------- */

#define SAMC_NVMC			0x41004000
#define SAMC_NVMC_CTRLA			(SAMC_NVMC + 0x0)
#define SAMC_NVMC_CTRLB			(SAMC_NVMC + 0x04)
#define SAMC_NVMC_PARAM			(SAMC_NVMC + 0x08)
#define SAMC_NVMC_INTFLAG		(SAMC_NVMC + 0x14)
#define SAMC_NVMC_STATUS		(SAMC_NVMC + 0x18)
#define SAMC_NVMC_ADDRESS		(SAMC_NVMC + 0x1C)

/* Control A Register (CTRLA) */
#define SAMC_CTRLA_CMD_KEY		0xA500
#define SAMC_CTRLA_CMD_ERASEROW		0x0002
#define SAMC_CTRLA_CMD_WRITEPAGE	0x0004
#define SAMC_CTRLA_CMD_ERASEAUXROW	0x0005
#define SAMC_CTRLA_CMD_WRITEAUXPAGE	0x0006
#define SAMC_CTRLA_CMD_LOCK		0x0040
#define SAMC_CTRLA_CMD_UNLOCK		0x0041
#define SAMC_CTRLA_CMD_PAGEBUFFERCLEAR	0x0044
#define SAMC_CTRLA_CMD_SSB		0x0045
#define SAMC_CTRLA_CMD_INVALL		0x0046

/* Interrupt Flag Register (INTFLAG) */
#define SAMC_NVMC_READY			(1 << 0)

/* Non-Volatile Memory Calibration and Auxiliary Registers */
#define SAMC_NVM_USER_ROW_LOW		0x00804000
#define SAMC_NVM_USER_ROW_HIGH		0x00804004
#define SAMC_NVM_CALIBRATION		0x00806020
#define SAMC_NVM_SERIAL(n)		(0x0080A00C + (0x30 * ((n + 3) / 4)) + \
					 (0x4 * n))

/* -------------------------------------------------------------------------- */
/* Device Service Unit (DSU) Registers */
/* -------------------------------------------------------------------------- */

#define SAMC_DSU			0x41002000
#define SAMC_DSU_EXT_ACCESS		(SAMC_DSU + 0x100)
#define SAMC_DSU_CTRLSTAT		(SAMC_DSU_EXT_ACCESS + 0x0)
#define SAMC_DSU_ADDRESS		(SAMC_DSU_EXT_ACCESS + 0x4)
#define SAMC_DSU_LENGTH			(SAMC_DSU_EXT_ACCESS + 0x8)
#define SAMC_DSU_DID			(SAMC_DSU_EXT_ACCESS + 0x018)
#define SAMC_DSU_PID			(SAMC_DSU + 0x1000)
#define SAMC_DSU_CID			(SAMC_DSU + 0x1010)

/* Control and Status Register (CTRLSTAT) */
#define SAMC_CTRL_CHIP_ERASE		(1 << 4)
#define SAMC_CTRL_MBIST			(1 << 3)
#define SAMC_CTRL_CRC			(1 << 2)
#define SAMC_STATUSA_PERR		(1 << 12)
#define SAMC_STATUSA_FAIL		(1 << 11)
#define SAMC_STATUSA_BERR		(1 << 10)
#define SAMC_STATUSA_CRSTEXT		(1 << 9)
#define SAMC_STATUSA_DONE		(1 << 8)
#define SAMC_STATUSB_PROT		(1 << 16)

/* Device Identification Register (DID) */
#define SAMC_DID_MASK			0xFFFF0000
#define SAMC_DID_CONST_VALUE		0x11010000
#define SAMC_DID_DEVSEL_MASK		0xFF
#define SAMC_DID_DEVSEL_POS		0
#define SAMC_DID_REVISION_MASK		0x0F
#define SAMC_DID_REVISION_POS		8
#define SAMC_DID_SERIES_MASK		0x1F
#define SAMC_DID_SERIES_POS		16
#define SAMC_DID_FAMILY_MASK		0x1F
#define SAMC_DID_FAMILY_POS		23

/* Peripheral ID */
#define SAMC_PID_MASK			0x00F7FFFF
#define SAMC_PID_CONST_VALUE		0x0001FCD0

/* Component ID */
#define SAMC_CID_VALUE			0xB105100D

/* Family parts */
struct samc_part {
	uint8_t devsel;
	char pin;
	uint8_t mem;
	uint8_t variant;
};

static const struct samc_part samc_c21_parts[] = {
	{0x00, 'J', 18, 'A'}, /* SAMC21J18A */
	{0x01, 'J', 17, 'A'}, /* SAMC21J17A */
	{0x02, 'J', 16, 'A'}, /* SAMC21J16A */
	{0x03, 'J', 15, 'A'}, /* SAMC21J15A */
	{0x05, 'G', 18, 'A'}, /* SAMC21G18A */
	{0x06, 'G', 17, 'A'}, /* SAMC21G17A */
	{0x07, 'G', 16, 'A'}, /* SAMC21G16A */
	{0x08, 'G', 15, 'A'}, /* SAMC21G15A */
	{0x0A, 'E', 18, 'A'}, /* SAMC21E18A */
	{0x0B, 'E', 17, 'A'}, /* SAMC21E17A */
	{0x0C, 'E', 16, 'A'}, /* SAMC21E16A */
	{0x0D, 'E', 15, 'A'}, /* SAMC21E15A */
	{0xFF, 0, 0, 0}
};

static const struct samc_part samc_c20_parts[] = {
	{0x00, 'J', 18, 'A'}, /* SAMC20J18A */
	{0x01, 'J', 17, 'A'}, /* SAMC20J17A */
	{0x02, 'J', 16, 'A'}, /* SAMC20J16A */
	{0x03, 'J', 15, 'A'}, /* SAMC20J15A */
	{0x05, 'G', 18, 'A'}, /* SAMC20G18A */
	{0x06, 'G', 17, 'A'}, /* SAMC20G17A */
	{0x07, 'G', 16, 'A'}, /* SAMC20G16A */
	{0x08, 'G', 15, 'A'}, /* SAMC20G15A */
	{0x0A, 'E', 18, 'A'}, /* SAMC20E18A */
	{0x0B, 'E', 17, 'A'}, /* SAMC20E17A */
	{0x0C, 'E', 16, 'A'}, /* SAMC20E16A */
	{0x0D, 'E', 15, 'A'}, /* SAMC20E15A */
	{0xFF, 0, 0, 0}
};

/**
 * Overloads the default cortexm reset function with a version that
 * removes the target from extended reset where required.
 */
void samc_reset(target *t)
{
	/**
	 * SRST is not asserted here as it appears to reset the adiv5
	 * logic, meaning that subsequent adiv5_* calls PLATFORM_FATAL_ERROR.
	 *
	 * This is ok as normally you can just connect the debugger and go,
	 * but if that's not possible (protection or SWCLK being used for
	 * something else) then having SWCLK low on reset should get you
	 * debug access (cold-plugging). TODO: Confirm this
	 *
	 * See the SAM D20 datasheet §12.6 Debug Operation for more
	 * details.
	 *
	 * jtagtap_srst(true);
	 * jtagtap_srst(false);
	 */

	/* Read DHCSR here to clear S_RESET_ST bit before reset */
	target_mem_read32(t, CORTEXM_DHCSR);

	/* Request system reset from NVIC: SRST doesn't work correctly */
	/* This could be VECTRESET: 0x05FA0001 (reset only core)
	 *          or SYSRESETREQ: 0x05FA0004 (system reset)
	 */
	target_mem_write32(t, CORTEXM_AIRCR,
	                   CORTEXM_AIRCR_VECTKEY | CORTEXM_AIRCR_SYSRESETREQ);

	/* Exit extended reset */
	if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) &
	    SAMC_STATUSA_CRSTEXT) {
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_CRSTEXT);
	}

	/* Poll for release from reset */
	while (target_mem_read32(t, CORTEXM_DHCSR) & CORTEXM_DHCSR_S_RESET_ST);

	/* Reset DFSR flags */
	target_mem_write32(t, CORTEXM_DFSR, CORTEXM_DFSR_RESETALL);

	/* Clear any target errors */
	target_check_error(t);
}

/**
 * Overloads the default cortexm detached function with a version that
 * removes the target from extended reset where required.
 *
 * Only required for SAM D20 _Revision B_ Silicon
 */
static void
samd20_revB_detach(target *t)
{
	cortexm_detach(t);

	/* ---- Additional ---- */
	/* Exit extended reset */
	if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) &
	    SAMC_STATUSA_CRSTEXT) {
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMC_DSU_CTRLSTAT,
		                   SAMC_STATUSA_CRSTEXT);
	}
}

/**
 * Overloads the default cortexm halt_resume function with a version
 * that removes the target from extended reset where required.
 *
 * Only required for SAM D20 _Revision B_ Silicon
 */
static void
samd20_revB_halt_resume(target *t, bool step)
{
	target_halt_resume(t, step);

	/* ---- Additional ---- */
	/* Exit extended reset */
	if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) & SAMC_STATUSA_CRSTEXT) {
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMC_DSU_CTRLSTAT,
		                   SAMC_STATUSA_CRSTEXT);
	}
}

/**
 * Overload the default cortexm attach for when the samd is protected.
 *
 * If the samd is protected then the default cortexm attach will
 * fail as the S_HALT bit in the DHCSR will never go high. This
 * function allows users to attach on a temporary basis so they can
 * rescue the device.
 */
bool samc_protected_attach(target *t)
{
	/**
	 * TODO: Notify the user that we're not really attached and
	 * they should issue the 'monitor erase_mass' command to
	 * regain access to the chip.
	 */

	/* Patch back in the normal cortexm attach for next time */
	t->attach = cortexm_attach;

	/* Allow attach this time */
	return true;
}

/**
 * Use the DSU Device Indentification Register to populate a struct
 * describing the SAM D device.
 */
struct samc_descr {
	char family;
	uint8_t series;
	char revision;
	char pin;
	uint8_t mem;
	char variant;
	char package[3];
};
struct samc_descr samc_parse_device_id(uint32_t did)
{
	struct samc_descr samc;
	uint8_t i = 0;
	const struct samc_part *parts = samc_c20_parts;
	memset(samc.package, 0, 3);

	uint8_t family = (did >> SAMC_DID_FAMILY_POS)
	  & SAMC_DID_FAMILY_MASK;
	uint8_t series = (did >> SAMC_DID_SERIES_POS)
	  & SAMC_DID_SERIES_MASK;
	uint8_t revision = (did >> SAMC_DID_REVISION_POS)
	  & SAMC_DID_REVISION_MASK;
	uint8_t devsel = (did >> SAMC_DID_DEVSEL_POS)
	  & SAMC_DID_DEVSEL_MASK;

	/* FIXME: adapt to C (manamana | 21/05/30) */
	/* Family */
	switch (family) {
		case 0: samc.family = 'D'; break;
		case 2: samc.family = 'C'; break;
	}
	/* Series */
	switch (series) {
		case 0: samc.series = 20; break;
		case 1: samc.series = 21; break;
		case 3: samc.series = 11; break;
		default: samc.series = 0; break;
	}
	/* Revision */
	samc.revision = 'A' + revision;

	switch (samc.series) {
	case 20: /* SAM C20 */
		parts = samc_c20_parts;
		switch (devsel / 5) {
			case 0: samc.pin = 'J'; break;
			case 1: samc.pin = 'G'; break;
			case 2: samc.pin = 'E'; break;
			default: samc.pin = 'u'; break;
		}
		samc.mem = 18 - (devsel % 5);
		samc.variant = 'A';
		break;
	case 21: /* SAM C21 */
		parts = samc_c21_parts;
		i = 0;
		while (parts[i].devsel != 0xFF) {
			if (parts[i].devsel == devsel) {
				samc.pin = parts[i].pin;
				samc.mem = parts[i].mem;
				samc.variant = parts[i].variant;
				break;
			}
			i++;
		}
		break;
	}

	return samc;
}

static void samc_add_flash(target *t, uint32_t addr, size_t length)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	if (!f) {			/* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}

	f->start = addr;
	f->length = length;
	f->blocksize = SAMC_ROW_SIZE;
	f->erase = samc_flash_erase;
	f->write = samc_flash_write;
	f->buf_size = SAMC_PAGE_SIZE;
	target_add_flash(t, f);
}

struct samc_priv_s {
	char samc_variant_string[60];
};

bool samc_probe(target *t)
{
	ADIv5_AP_t *ap = cortexm_ap(t);
	uint32_t cid = adiv5_ap_read_pidr(ap, SAMC_DSU_CID);
	uint32_t pid = adiv5_ap_read_pidr(ap, SAMC_DSU_PID);

	/* Check the ARM Coresight Component and Perhiperal IDs */
	if ((cid != SAMC_CID_VALUE) ||
	    ((pid & SAMC_PID_MASK) != SAMC_PID_CONST_VALUE))
		return false;

	/* Read the Device ID */
	uint32_t did = target_mem_read32(t, SAMC_DSU_DID);

	/* If the Device ID matches */
	if ((did & SAMC_DID_MASK) != SAMC_DID_CONST_VALUE)
		return false;

	struct samc_priv_s *priv_storage = calloc(1, sizeof(*priv_storage));
	t->target_storage = (void*)priv_storage;

	uint32_t ctrlstat = target_mem_read32(t, SAMC_DSU_CTRLSTAT);
	struct samc_descr samc = samc_parse_device_id(did);

	/* Protected? */
	bool protected = (ctrlstat & SAMC_STATUSB_PROT);

	/* Part String */
	if (protected) {
		sprintf(priv_storage->samc_variant_string,
		        "Atmel SAM%c%d%c%d%c%s (rev %c) (PROT=1)",
		        samc.family,
		        samc.series, samc.pin, samc.mem,
		        samc.variant,
		        samc.package, samc.revision);
	} else {
		sprintf(priv_storage->samc_variant_string,
		        "Atmel SAM%c%d%c%d%c%s (rev %c)",
		        samc.family,
		        samc.series, samc.pin, samc.mem,
		        samc.variant,
		        samc.package, samc.revision);
	}

	/* Setup Target */
	t->driver = priv_storage->samc_variant_string;
	t->reset = samc_reset;

	if (samc.series == 20 && samc.revision == 'B') {
		/**
		 * These functions check for and
		 * extended reset. Appears to be
		 * related to Errata 35.4.1 ref 12015
		 */
		t->detach      = samd20_revB_detach;
		t->halt_resume = samd20_revB_halt_resume;
	}
	if (protected) {
		/**
		 * Overload the default cortexm attach
		 * for when the samd is protected.
		 * This function allows users to
		 * attach on a temporary basis so they
		 * can rescue the device.
		 */
		t->attach = samc_protected_attach;
	}

	target_add_ram(t, 0x20000000, 0x8000);
	samc_add_flash(t, 0x00000000, 0x40000);
	target_add_commands(t, samc_cmd_list, "SAMC");

	/* If we're not in reset here */
	if (!platform_srst_get_val()) {
		/* We'll have to release the target from
		 * extended reset to make attach possible */
		if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) &
		    SAMC_STATUSA_CRSTEXT) {

			/* Write bit to clear from extended reset */
			target_mem_write32(t, SAMC_DSU_CTRLSTAT,
			                   SAMC_STATUSA_CRSTEXT);
		}
	}

	return true;
}

/**
 * Temporary (until next reset) flash memory locking / unlocking
 */
static void samc_lock_current_address(target *t)
{
	/* Issue the unlock command */
	target_mem_write32(t, SAMC_NVMC_CTRLA,
	                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_LOCK);
}
static void samc_unlock_current_address(target *t)
{
	/* Issue the unlock command */
	target_mem_write32(t, SAMC_NVMC_CTRLA,
	                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_UNLOCK);
}

/**
 * Erase flash row by row
 */
static int samc_flash_erase(struct target_flash *f, target_addr addr, size_t len)
{
	target *t = f->t;
	while (len) {
		/* Write address of first word in row to erase it */
		/* Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address */
		target_mem_write32(t, SAMC_NVMC_ADDRESS, addr >> 1);

		/* Unlock */
		samc_unlock_current_address(t);

		/* Issue the erase command */
		target_mem_write32(t, SAMC_NVMC_CTRLA,
		                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_ERASEROW);
		/* Poll for NVM Ready */
		while ((target_mem_read32(t, SAMC_NVMC_INTFLAG) & SAMC_NVMC_READY) == 0)
			if (target_check_error(t))
				return -1;

		/* Lock */
		samc_lock_current_address(t);

		addr += f->blocksize;
		if (len > f->blocksize)
			len -= f->blocksize;
		else
			len = 0;
	}

	return 0;
}

/**
 * Write flash page by page
 */
static int samc_flash_write(struct target_flash *f,
                            target_addr dest, const void *src, size_t len)
{
	target *t = f->t;

	/* Write within a single page. This may be part or all of the page */
	target_mem_write(t, dest, src, len);

	/* Unlock */
	samc_unlock_current_address(t);

	/* Issue the write page command */
	target_mem_write32(t, SAMC_NVMC_CTRLA,
	                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_WRITEPAGE);

	/* Poll for NVM Ready */
	while ((target_mem_read32(t, SAMC_NVMC_INTFLAG) & SAMC_NVMC_READY) == 0)
		if (target_check_error(t))
			return -1;

	/* Lock */
	samc_lock_current_address(t);

	return 0;
}

/**
 * Uses the Device Service Unit to erase the entire flash
 */
bool samc_cmd_erase_all(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	/* Clear the DSU status bits */
	target_mem_write32(t, SAMC_DSU_CTRLSTAT,
	                   SAMC_STATUSA_DONE | SAMC_STATUSA_PERR |
	                   SAMC_STATUSA_FAIL);

	/* Erase all */
	target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_CTRL_CHIP_ERASE);

	/* Poll for DSU Ready */
	uint32_t status;
	while (((status = target_mem_read32(t, SAMC_DSU_CTRLSTAT)) &
		(SAMC_STATUSA_DONE | SAMC_STATUSA_PERR | SAMC_STATUSA_FAIL)) == 0)
		if (target_check_error(t))
			return false;

	/* Test the protection error bit in Status A */
	if (status & SAMC_STATUSA_PERR) {
		tc_printf(t, "Erase failed due to a protection error.\n");
		return true;
	}

	/* Test the fail bit in Status A */
	if (status & SAMC_STATUSA_FAIL) {
		tc_printf(t, "Erase failed.\n");
		return true;
	}

	tc_printf(t, "Erase successful!\n");

	return true;
}

/**
 * Sets the NVM region lock bits in the User Row. This value is read
 * at startup as the default value for the lock bits, and hence does
 * not take effect until a reset.
 *
 * 0x0000 = Lock, 0xFFFF = Unlock (default)
 */
static bool samc_set_flashlock(target *t, uint16_t value, const char **argv)
{
	(void)argv;
	uint32_t high = target_mem_read32(t, SAMC_NVM_USER_ROW_HIGH);
	uint32_t low = target_mem_read32(t, SAMC_NVM_USER_ROW_LOW);

	/* Write address of a word in the row to erase it */
	/* Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address */
	target_mem_write32(t, SAMC_NVMC_ADDRESS, SAMC_NVM_USER_ROW_LOW >> 1);

	/* Issue the erase command */
	target_mem_write32(t, SAMC_NVMC_CTRLA,
	                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_ERASEAUXROW);

	/* Poll for NVM Ready */
	while ((target_mem_read32(t, SAMC_NVMC_INTFLAG) & SAMC_NVMC_READY) == 0)
		if (target_check_error(t))
			return -1;

	/* Modify the high byte of the user row */
	high = (high & 0x0000FFFF) | ((value << 16) & 0xFFFF0000);

	/* Write back */
	target_mem_write32(t, SAMC_NVM_USER_ROW_LOW, low);
	target_mem_write32(t, SAMC_NVM_USER_ROW_HIGH, high);

	/* Issue the page write command */
	target_mem_write32(t, SAMC_NVMC_CTRLA,
	                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_WRITEAUXPAGE);

	return true;
}

static bool samc_cmd_lock_flash(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	return samc_set_flashlock(t, 0x0000, NULL);
}

static bool samc_cmd_unlock_flash(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	return samc_set_flashlock(t, 0xFFFF, NULL);
}

static bool samc_set_bootprot(target *t, uint16_t value, const char **argv)
{
	(void)argv;
	uint32_t high = target_mem_read32(t, SAMC_NVM_USER_ROW_HIGH);
	uint32_t low = target_mem_read32(t, SAMC_NVM_USER_ROW_LOW);

	/* Write address of a word in the row to erase it */
	/* Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address */
	target_mem_write32(t, SAMC_NVMC_ADDRESS, SAMC_NVM_USER_ROW_LOW >> 1);

	/* Issue the erase command */
	target_mem_write32(t, SAMC_NVMC_CTRLA,
	                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_ERASEAUXROW);

	/* Poll for NVM Ready */
	while ((target_mem_read32(t, SAMC_NVMC_INTFLAG) & SAMC_NVMC_READY) == 0)
		if (target_check_error(t))
			return -1;

	/* Modify the low word of the user row */
	low = (low & 0xFFFFFFF8) | ((value << 0 ) & 0x00000007);

	/* Write back */
	target_mem_write32(t, SAMC_NVM_USER_ROW_LOW, low);
	target_mem_write32(t, SAMC_NVM_USER_ROW_HIGH, high);

	/* Issue the page write command */
	target_mem_write32(t, SAMC_NVMC_CTRLA,
	                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_WRITEAUXPAGE);

	return true;
}

static bool samc_cmd_lock_bootprot(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	return samc_set_bootprot(t, 0, NULL);
}

static bool samc_cmd_unlock_bootprot(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	return samc_set_bootprot(t, 7, NULL);
}

static bool samc_cmd_read_userrow(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	tc_printf(t, "User Row: 0x%08x%08x\n",
		target_mem_read32(t, SAMC_NVM_USER_ROW_HIGH),
		target_mem_read32(t, SAMC_NVM_USER_ROW_LOW));

	return true;
}

/**
 * Reads the 128-bit serial number from the NVM
 */
static bool samc_cmd_serial(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	tc_printf(t, "Serial Number: 0x");

	for (uint32_t i = 0; i < 4; i++) {
		tc_printf(t, "%08x", target_mem_read32(t, SAMC_NVM_SERIAL(i)));
	}

	tc_printf(t, "\n");

	return true;
}

/**
 * Returns the size (in bytes) of the current SAM D20's flash memory.
 */
static uint32_t samc_flash_size(target *t)
{
	/* Read the Device ID */
	uint32_t did = target_mem_read32(t, SAMC_DSU_DID);

	/* Mask off the device select bits */
	uint8_t devsel = did & SAMC_DID_DEVSEL_MASK;

	/* Shift the maximum flash size (256KB) down as appropriate */
	return (0x40000 >> (devsel % 5));
}

/**
 * Runs the Memory Built In Self Test (MBIST)
 */
static bool samc_cmd_mbist(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	/* Write the memory parameters to the DSU */
	target_mem_write32(t, SAMC_DSU_ADDRESS, 0);
	target_mem_write32(t, SAMC_DSU_LENGTH, samc_flash_size(t));

	/* Clear the fail bit */
	target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_FAIL);

	/* Write the MBIST command */
	target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_CTRL_MBIST);

	/* Poll for DSU Ready */
	uint32_t status;
	while (((status = target_mem_read32(t, SAMC_DSU_CTRLSTAT)) &
		(SAMC_STATUSA_DONE | SAMC_STATUSA_PERR | SAMC_STATUSA_FAIL)) == 0)
		if (target_check_error(t))
			return false;

	/* Test the protection error bit in Status A */
	if (status & SAMC_STATUSA_PERR) {
		tc_printf(t, "MBIST not run due to protection error.\n");
		return true;
	}

	/* Test the fail bit in Status A */
	if (status & SAMC_STATUSA_FAIL) {
		tc_printf(t, "MBIST Fail @ 0x%08x\n",
		          target_mem_read32(t, SAMC_DSU_ADDRESS));
	} else {
		tc_printf(t, "MBIST Passed!\n");
	}

	return true;
}
/**
 * Sets the security bit
 */
static bool samc_cmd_ssb(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	/* Issue the ssb command */
	target_mem_write32(t, SAMC_NVMC_CTRLA,
	                   SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_SSB);

	/* Poll for NVM Ready */
	while ((target_mem_read32(t, SAMC_NVMC_INTFLAG) & SAMC_NVMC_READY) == 0)
		if (target_check_error(t))
			return -1;

	tc_printf(t, "Security bit set! "
		  "Scan again, attach and issue 'monitor erase_mass' to reset.\n");

	target_reset(t);
	return true;
}
