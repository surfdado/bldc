/*
	Copyright 2019 - 2020 Benjamin Vedder	benjamin@vedder.se

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

#include "conf_general.h"
#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "comm_can.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Macros
#define IS_SPORT_MODE()				(m_set_now == &m_set_sport)
#define IS_ECO_MODE()				(m_set_now == &m_set_eco)
#define LED_ECO_ON()				comm_can_io_board_set_output_digital(255, 1, 1)
#define LED_ECO_OFF()				comm_can_io_board_set_output_digital(255, 1, 0)
#define LED_SPORT_ON()				comm_can_io_board_set_output_digital(255, 2, 1)
#define LED_SPORT_OFF()				comm_can_io_board_set_output_digital(255, 2, 0)
#define LED_LOW_BATT_ON()			comm_can_io_board_set_output_digital(255, 3, 1)
#define LED_LOW_BATT_OFF()			comm_can_io_board_set_output_digital(255, 3, 0)
#define LED_FAULT_ON()				comm_can_io_board_set_output_digital(255, 4, 1)
#define LED_FAULT_OFF()				comm_can_io_board_set_output_digital(255, 4, 0)

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool plot_state = false;
static volatile float plot_sample = 0.0;
static volatile bool m_brake_rear = false;
static volatile bool m_brake_front = false;
static volatile bool m_mode_btn_down = false;
static volatile bool m_kill_sw = false;

// Parameters
#define P_OFFSET_ECO								10
#define P_OFFSET_SPORT								20
#define P_THROTTLE_HYST_ADDR						0
#define P_PEDAL_CURRENT_ADDR						1
#define P_START_GAIN_ADDR							2
#define P_START_GAIN_END_SPEED_ADDR					3
#define P_OUTPUT_POWER_ADDR							4
#define P_TOP_SPEED_ERPM_ADDR						5
#define P_BRAKE_CURRENT_FRONT_ADDR					6
#define P_BRAKE_CURRENT_REAR_ADDR					7
#define P_BRAKE_CURRENT_BOTH_ADDR					8

typedef struct {
	volatile float p_throttle_hyst;
	volatile float p_pedal_current;
	volatile float p_start_gain;
	volatile float p_start_gain_end_speed;
	volatile float p_output_power;
	volatile float p_top_speed_erpm;
	volatile float p_brake_current_front;
	volatile float p_brake_current_rear;
	volatile float p_brake_current_both;
} SETTINGS_T;

static volatile SETTINGS_T m_set_normal;
static volatile SETTINGS_T m_set_eco;
static volatile SETTINGS_T m_set_sport;
static volatile SETTINGS_T *m_set_now = &m_set_normal;
static volatile int m_set_now_offset = 0;

// Private functions
static void terminal_plot(int argc, const char **argv);
static void terminal_info(int argc, const char **argv);
static void terminal_mon(int argc, const char **argv);
static void terminal_restore_settings(int argc, const char **argv);
static void terminal_set_hyst(int argc, const char **argv);
static void terminal_set_pedal_current(int argc, const char **argv);
static void terminal_set_start_gain(int argc, const char **argv);
static void terminal_set_start_gain_end_speed(int argc, const char **argv);
static void terminal_set_output_power(int argc, const char **argv);
static void terminal_set_top_speed_erpm(int argc, const char **argv);
static void terminal_set_brake_power(int argc, const char **argv);

static void restore_settings(void) {
}

static void load_settings(volatile SETTINGS_T *set, int offset) {
}

static void store_settings(volatile SETTINGS_T *set, int offset) {
}

void app_custom_start(void) {
}

void app_custom_stop(void) {
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("Erockit");
}

static void terminal_plot(int argc, const char **argv) {
}

static void print_settings_info(volatile SETTINGS_T *set) {
}

static void terminal_info(int argc, const char **argv) {
}

static void terminal_mon(int argc, const char **argv) {
}

static void terminal_restore_settings(int argc, const char **argv) {
}

static void terminal_set_hyst(int argc, const char **argv) {
}

static void terminal_set_pedal_current(int argc, const char **argv) {
}

static void terminal_set_start_gain(int argc, const char **argv) {
}

static void terminal_set_start_gain_end_speed(int argc, const char **argv) {
}

static void terminal_set_output_power(int argc, const char **argv) {
}

static void terminal_set_top_speed_erpm(int argc, const char **argv) {
}

static void terminal_set_brake_power(int argc, const char **argv) {
}
