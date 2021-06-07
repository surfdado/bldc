/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

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
#include "stm32f4xx_conf.h"
#include "servo_dec.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_PULSES_WITHOUT_POWER		50

// Threads
static THD_FUNCTION(ppm_thread, arg);
static THD_WORKING_AREA(ppm_thread_wa, 128);
static thread_t *ppm_tp;
static volatile bool ppm_rx = false;

// Private functions
static void servodec_func(void);

// Private variables
static volatile bool is_running = false;
static volatile bool stop_now = true;
static volatile ppm_config config;
static volatile int pulses_without_power = 0;
static float input_val = 0.0;
static volatile float direction_hyst = 0;

// Private functions

void app_ppm_configure(ppm_config *conf) {
	config = *conf;
	pulses_without_power = 0;

	if (is_running) {
		servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	}

	direction_hyst = config.max_erpm_for_dir * 0.20;
}

void app_ppm_start(void) {
	stop_now = false;
	chThdCreateStatic(ppm_thread_wa, sizeof(ppm_thread_wa), NORMALPRIO, ppm_thread, NULL);
}

void app_ppm_stop(void) {
	stop_now = true;

	if (is_running) {
		chEvtSignalI(ppm_tp, (eventmask_t) 1);
		servodec_stop();
	}

	while(is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_ppm_get_decoded_level(void) {
	return input_val;
}

static void servodec_func(void) {
	ppm_rx = true;
	chSysLockFromISR();
	chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(ppm_thread, arg) {
	(void)arg;
}
