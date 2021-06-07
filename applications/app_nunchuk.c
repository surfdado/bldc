/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "commands.h"
#include "utils.h"
#include "timeout.h"
#include <string.h>
#include <math.h>
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"

// Settings
#define OUTPUT_ITERATION_TIME_MS		5
#define MAX_CAN_AGE						0.1
#define RPM_FILTER_SAMPLES				8
#define LOCAL_TIMEOUT					2000

// Threads
static THD_FUNCTION(chuk_thread, arg);
static THD_WORKING_AREA(chuk_thread_wa, 128);
static THD_FUNCTION(output_thread, arg);
static THD_WORKING_AREA(output_thread_wa, 128);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile chuck_data chuck_d;
static volatile int chuck_error = 0;
static volatile chuk_config config;
static volatile bool output_running = false;
static volatile systime_t last_update_time;

// Private functions
static void terminal_cmd_nunchuk_status(int argc, const char **argv);

void app_nunchuk_configure(chuk_config *conf) {
	config = *conf;
}

void app_nunchuk_start(void) {
	chuck_d.js_y = 128;
	stop_now = false;
	hw_start_i2c();
	chThdCreateStatic(chuk_thread_wa, sizeof(chuk_thread_wa), NORMALPRIO, chuk_thread, NULL);
}

void app_nunchuk_stop(void) {
	stop_now = true;

	if (is_running) {
		hw_stop_i2c();
	}

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_nunchuk_get_decoded_chuk(void) {
	return ((float)chuck_d.js_y - 128.0) / 128.0;
}

void app_nunchuk_update_output(chuck_data *data) {
	if (!output_running) {
		last_update_time = 0;
		output_running = true;
		chuck_d.js_y = 128;
		chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO, output_thread, NULL);
	}

	chuck_d = *data;
	last_update_time = chVTGetSystemTime();
	timeout_reset();
}

static THD_FUNCTION(chuk_thread, arg) {
	(void)arg;
}

static THD_FUNCTION(output_thread, arg) {
	(void)arg;

	chRegSetThreadName("Nunchuk output");
}

static void terminal_cmd_nunchuk_status(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("Nunchuk Status");
	commands_printf("Output: %s", output_running ? "On" : "Off");
	commands_printf(" ");
}
