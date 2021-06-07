/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8

// Threads
static THD_FUNCTION(adc_thread, arg);
static THD_WORKING_AREA(adc_thread_wa, 1024);

// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile float decoded_level2 = 0.0;
static volatile float read_voltage2 = 0.0;
static volatile bool use_rx_tx_as_buttons = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;

void app_adc_configure(adc_config *conf) {
	config = *conf;
	ms_without_power = 0.0;
}

void app_adc_start(bool use_rx_tx) {
	use_rx_tx_as_buttons = use_rx_tx;
	stop_now = false;
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void app_adc_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_adc_get_decoded_level(void) {
	return decoded_level;
}

float app_adc_get_voltage(void) {
	return read_voltage;
}

float app_adc_get_decoded_level2(void) {
	return decoded_level2;
}

float app_adc_get_voltage2(void) {
	return read_voltage2;
}


static THD_FUNCTION(adc_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_ADC");

}
