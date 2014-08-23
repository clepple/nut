/*!@file
 * @brief Driver for OpenElectrons.com SmartUPS power supply connected via Linux I2C.
 *
 * See http://www.openelectrons.com/pages/33
 * or http://www.openelectrons.com/index.php?module=documents&JAS_DocumentManager_op=viewDocument&JAS_Document_id=9
 * for programming information.
 */
/*
   Copyright (C) 2014  Charles Lepple <clepple+nut@gmail.com>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

/* Requires libi2c-dev from lm_sensors: */
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

#include "main.h"

#define DRIVER_NAME	"OpenElectrons.com SmartUPS I2C driver"
#define DRIVER_VERSION	"0.9"

/* driver description structure */
upsdrv_info_t upsdrv_info = {
	DRIVER_NAME,
	DRIVER_VERSION,
	"Charles Lepple <clepple+nut@gmail.com>",
	DRV_EXPERIMENTAL,
	{ NULL }
};

#define SLAVE_ADDRESS 0x12 /*!< @todo Make address configurable */
#define VENDOR_ID_OFFSET 0x8
#define DEVICE_ID_OFFSET 0x10
#define FIRMWARE_OFFSET 0

static int device_initialized = 0;
static int fsd_latch = 0;

static int select_slave(int fd)
{
	if(ioctl(fd, I2C_SLAVE, SLAVE_ADDRESS) < 0) {
		fatal_with_errno(EXIT_FAILURE, "Could not select slave address 0x%02x", SLAVE_ADDRESS);
	}
	return 0;
}

static int i2c_read_cstring(int fd, int start, int len, char *dest)
{
	int ret;
	unsigned char start_buf = start;

	upsdebugx(3, "%s: selecting offset 0x%02x", __func__, start);
	ret = write(fd, &start_buf, 1);
	if(ret < 0) {
		upslog_with_errno(LOG_NOTICE, "%s: could not write offset 0x%02x for reading", __func__, start);
		return ret;
	}

	upsdebugx(3, "%s: reading %d bytes from offset 0x%02x", __func__, len, start);
	ret = read(fd, dest, len);
	if(ret < 0) {
		upslog_with_errno(LOG_NOTICE, "%s: could not read from offset 0x%02x", __func__, start);
		return ret;
	}

	if(ret != len) {
		upslogx(LOG_NOTICE, "%s: requested %d bytes, got %d", __func__, len, ret);
	}

	dest[ret] = 0;

	upsdebugx(3, "%s: got '%s'", __func__, dest);

	return 0;
}

static void smartups_read_ID(int error_is_fatal)
{
	int ret;
	char vendor[16], model[16], firmware[16];

	upsdebugx(2, "%s: calling select_slave()", __func__);
	select_slave(upsfd);

	ret = i2c_read_cstring(upsfd, VENDOR_ID_OFFSET, 8, vendor);
	if(ret < 0) {
		if(error_is_fatal) {
			fatal_with_errno(EXIT_FAILURE, "Could not read Vendor ID string");
		} else {
			upsdebugx(1, "%s: i2c_read_cstring(VENDOR) returned %d", __func__, ret);
			return;
		}
	}

	upsdebugx(1, "Vendor ID = '%s'", vendor);
	if(!strcmp(vendor, "Openelec")) {
		dstate_setinfo("ups.mfr", "OpenElectrons.com");
	} else {
		dstate_setinfo("ups.mfr", "%s", vendor);
	}

	ret = i2c_read_cstring(upsfd, DEVICE_ID_OFFSET, 8, model);
	if(ret < 0) {
		if(error_is_fatal) {
			fatal_with_errno(EXIT_FAILURE, "Could not read Device ID string");
		} else {
			upsdebugx(1, "%s: i2c_read_cstring(DEVICE_ID) returned %d", __func__, ret);
			return;
		}
	}

	upsdebugx(1, "Device ID = '%s'", model);
	dstate_setinfo("ups.model", "%s", model);

	ret = i2c_read_cstring(upsfd, FIRMWARE_OFFSET, 8, firmware);
	if(ret < 0) {
		if(error_is_fatal) {
			fatal_with_errno(EXIT_FAILURE, "Could not read firmware string");
		} else {
			upsdebugx(1, "%s: i2c_read_cstring(FIRMWARE) returned %d", __func__, ret);
			return;
		}
	}

	upsdebugx(1, "Firmware version = '%s'", firmware);
	dstate_setinfo("ups.firmware", "%s", firmware);

	if(strcmp(firmware, "V1.03")) {
		upsdebugx(1, "Expecting firmware 'V1.03', got '%s'", firmware);
	}

	dstate_setinfo("output.voltage.nominal", "5.0");
	dstate_setinfo("battery.voltage.nominal", "4.5");
	/* This is the only chemistry the charger can do: */
	dstate_setinfo("battery.type", "NiMH");
	dstate_setinfo("ups.delay.shutdown", "50");

	printf("Detected: %s %s (%s)\n", dstate_getinfo("ups.mfr"), model, firmware);
	/* upsh.instcmd = instcmd; */

	device_initialized = 1;
	fsd_latch = 0;
}

void upsdrv_initinfo(void)
{
	smartups_read_ID(1 /* errors are fatal */);
}


#define COMMAND_OFFSET		0x41
#define COMMAND_VALUE		0x53 /* 'S' */
#define RESTART_OPTION		0x42
#define BUTTON_STATE		0x43
#define RESTART_TIME		0x44 /* word */
#define BATTERY_STATE		0x46
#define BATTERY_CURRENT		0x48
#define BATTERY_VOLTAGE		0x4A
#define BATTERY_CAPACITY	0x4C
#define BATTERY_TIME		0x4E
#define BATTERY_TEMPERATURE	0x50
#define BATTERY_HEALTH		0x51
#define OUTPUT_VOLTAGE		0x52
#define OUTPUT_CURRENT		0x54
#define BATTERY_MAX_CAPACITY	0x56
#define SECONDS			0x58
#define READ_OFFSET		RESTART_OPTION
#define READ_LEN		(SECONDS + 2 - RESTART_OPTION + 1)

void upsdrv_updateinfo(void)
{
	char sign;
	int ret, tmp, battery_capacity, battery_max_capacity;
	unsigned char buffer[256];

	if(!device_initialized) {
		smartups_read_ID(0 /* errors are non-fatal */);
	}

	upsdebugx(2, "%s: select address 0x%02x", __func__, READ_OFFSET);
	buffer[0] = READ_OFFSET;
	ret = write(upsfd, buffer, 1);

	if (ret < 0) {
		upslog_with_errno(LOG_ERR, "Could not set address (no ACK?)");
		dstate_datastale();
		device_initialized = 0;
		return;
	}

	upsdebugx(2, "%s: read 0x%02x bytes", __func__, READ_LEN);
	ret = read(upsfd, buffer+READ_OFFSET, READ_LEN);

	if (ret < 0) {
		upslog_with_errno(LOG_ERR, "Could not read data block.");
		dstate_datastale();
		device_initialized = 0;
		return;
	}

	upsdebug_hex(3, "read buffer", buffer+READ_OFFSET, READ_LEN);

	tmp = buffer[BATTERY_CURRENT] | (buffer[BATTERY_CURRENT+1] << 8);
	upsdebugx(1, "Battery current: 0x%04x (%d)", tmp, tmp);

	sign = '+';
	if(tmp & (1<<15)) {
		tmp = 65536 - tmp;
		sign = '-';
	}
	dstate_setinfo("battery.current", "%c%d.%03d",
		sign, (unsigned)tmp / 1000, (unsigned)tmp % 1000);

	/* - * - */

	tmp = buffer[BATTERY_VOLTAGE] | (buffer[BATTERY_VOLTAGE+1] << 8);

	upsdebugx(1, "Battery voltage: 0x%04x (%d)", tmp, tmp);
	dstate_setinfo("battery.voltage", "%d.%03d", (unsigned)tmp / 1000, (unsigned)tmp % 1000);

	/* - * - */

	tmp = buffer[BATTERY_CAPACITY] | (buffer[BATTERY_CAPACITY+1] << 8);
	upsdebugx(1, "Battery capacity: 0x%04x (%d)", tmp, tmp);
	battery_capacity = tmp;

	/* - * - */

	tmp = buffer[BATTERY_TIME] | (buffer[BATTERY_TIME+1] << 8);
	upsdebugx(1, "Battery time: 0x%04x (%d)", tmp, tmp);
	dstate_setinfo("battery.runtime", "%d", tmp);

	/* - * - */

	tmp = buffer[BATTERY_TEMPERATURE];
	upsdebugx(1, "Battery temperature: 0x%02x (%d)", tmp, tmp);
	dstate_setinfo("battery.temperature", "%d", tmp);

	/* - * - */

	tmp = buffer[BATTERY_HEALTH];
	upsdebugx(1, "Battery health: 0x%02x (%d)", tmp, tmp);

	/* - * - */

	/* Output voltage is non-zero, but not right with V1.03 */
#if 0
	tmp = buffer[OUTPUT_VOLTAGE] | (buffer[OUTPUT_VOLTAGE+1] << 8);
	upsdebugx(1, "Output voltage: 0x%04x (%d)", tmp, tmp);

	/* Not sure what a negative voltage means, but that can't be right... */
	if(!(tmp & (1<<15))) {
		dstate_setinfo("output.voltage", "%d.%02d", (unsigned)tmp / 100, (unsigned)tmp % 100);
	} else {
		upsdebugx(1, "Output voltage negative? Skipping.");
	}
#endif

	/* - * - */

	/* Output current is zero with V1.03 */
#if 0
	tmp = buffer[OUTPUT_CURRENT] | (buffer[OUTPUT_CURRENT+1] << 8);
	upsdebugx(1, "Output current: 0x%04x (%d)", tmp, tmp);
#endif
	/* - * - */

	tmp = buffer[BATTERY_MAX_CAPACITY] | (buffer[BATTERY_MAX_CAPACITY+1] << 8);
	upsdebugx(1, "Battery max capacity: 0x%04x (%d)", tmp, tmp);
	battery_max_capacity = tmp;
	dstate_setinfo("battery.charge", "%d", 100 * battery_capacity / battery_max_capacity);

	/* - * - */

	/* Use this for time skew detection? */
#if 1
	tmp = buffer[SECONDS] | (buffer[SECONDS+1] << 8);
	upsdebugx(1, "Seconds: 0x%04x (%d)", tmp, tmp);
	dstate_setinfo("ups.time", "%d", tmp);
#endif

	tmp = buffer[BUTTON_STATE];
	dstate_setinfo("ups.contacts", "%x", tmp & 3);

	if((tmp == 9) || (tmp == 0xa)) {
		fsd_latch = 1;
	}

	/* A button press will reset FSD: */
	if((tmp == 1) || (tmp == 2) || (tmp == 3)) {
		fsd_latch = 0;
	}

	/* - * - */

	tmp = buffer[BATTERY_STATE];
	upsdebugx(1, "Battery state: 0x%04x", tmp);

	status_init();

	switch(tmp) {
		case 0: /* Idle: figuring out battery status */
			status_set("OL");
			break;
		case 1: /* Precharge */
		case 2: /* Charging */
		case 3: /* Top-off */
			status_set("OL CHRG");
			break;
		case 4: /* Charged */
			status_set("OL");
			break;
		case 5:
			status_set("OB");
			break;
		case 6: /* Critical */
		case 7: /* Discharged */
			status_set("OB LB");
			break;
		/* 8 = fault? */
		default:
			upslogx(LOG_NOTICE, "%s: unknown battery state 0x%02x", __func__, tmp);
	}

	if(fsd_latch) {
		status_set("FSD");
	}

	status_commit();

	/* - * - */

	/*
	 * poll_interval = 2;
	 */

	dstate_dataok();
	upsdebugx(2, "done with %s\n", __func__);
}

void upsdrv_shutdown(void)
{
	int ret;
	const unsigned char buffer[] = { COMMAND_OFFSET, COMMAND_VALUE };

	/* tell the UPS to shut down, then return - DO NOT SLEEP HERE */

	/* maybe try to detect the UPS here, but try a shutdown even if
	   it doesn't respond at first if possible */
	select_slave(upsfd);

	ret = write(upsfd, buffer, 2);
	if(ret != 2) {
		upslog_with_errno(LOG_ERR, "Could not send shutdown command (ret = %d)", ret);
	}

	/* you may have to check the line status since the commands
	   for toggling power are frequently different for OL vs. OB */

	/* OL: this must power cycle the load if possible */

	/* OB: the load must remain off until the power returns */
}

/*
static int instcmd(const char *cmdname, const char *extra)
{
	if (!strcasecmp(cmdname, "test.battery.stop")) {
		ser_send_buf(upsfd, ...);
		return STAT_INSTCMD_HANDLED;
	}

	upslogx(LOG_NOTICE, "instcmd: unknown command [%s]", cmdname);
	return STAT_INSTCMD_UNKNOWN;
}
*/

/*
static int setvar(const char *varname, const char *val)
{
	if (!strcasecmp(varname, "ups.test.interval")) {
		ser_send_buf(upsfd, ...);
		return STAT_SET_HANDLED;
	}

	upslogx(LOG_NOTICE, "setvar: unknown variable [%s]", varname);
	return STAT_SET_UNKNOWN;
}
*/

void upsdrv_help(void)
{
}

/* list flags and values that you want to receive via -x */
void upsdrv_makevartable(void)
{
	/* allow '-x xyzzy' */
	/* addvar(VAR_FLAG, "xyzzy", "Enable xyzzy mode"); */

	/* allow '-x foo=<some value>' */
	/* addvar(VAR_VALUE, "foo", "Override foo setting"); */
}

void upsdrv_initups(void)
{
	upsdebugx(1, "%s: opening '%s'", __func__, device_path);

	upsfd = open(device_path, O_RDWR);

	if(upsfd < 0) {
		fatal_with_errno(EXIT_FAILURE, "Could not open I2C bus '%s'", device_path);
	}

	/* to get variables and flags from the command line, use this:
	 *
	 * first populate with upsdrv_makevartable() above, then...
	 *
	 *                   set flag foo : /bin/driver -x foo
	 * set variable 'cable' to '1234' : /bin/driver -x cable=1234
	 *
	 * to test flag foo in your code:
	 *
	 * 	if (testvar("foo"))
	 * 		do_something();
	 *
	 * to show the value of cable:
	 *
	 *      if ((cable = getval("cable")))
	 *		printf("cable is set to %s\n", cable);
	 *	else
	 *		printf("cable is not set!\n");
	 *
	 * don't use NULL pointers - test the return result first!
	 */

	/* the upsh handlers can't be done here, as they get initialized
	 * shortly after upsdrv_initups returns to main.
	 */

	/* don't try to detect the UPS here */
}

void upsdrv_cleanup(void)
{
	if(upsfd >= 0) {
		close(upsfd);
		upsfd = -1;
	}
}
