/*
	Copyright 2019 Mitch Lustig

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

#include "conf_general.h"

#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "commands.h"
#include "imu/imu.h"
#include "utils.h"
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"
#include "buzzer.h"
#include "app.h"
#include "mcpwm_foc.h"
#include "digital_filter.h"

#include <math.h>
#include <stdio.h>

// Acceleration average
#define ACCEL_ARRAY_SIZE 40

// Data type (Value 5 was removed, and can be reused at a later date, but i wanted to preserve the current value's numbers for UIs)
typedef enum {
	STARTUP = 0,
	RUNNING = 1,
	RUNNING_TILTBACK = 2,
	RUNNING_WHEELSLIP = 3,
	RUNNING_UPSIDEDOWN = 4,
	FAULT_ANGLE_PITCH = 6,
	FAULT_ANGLE_ROLL = 7,
	FAULT_SWITCH_HALF = 8,
	FAULT_SWITCH_FULL = 9,
	FAULT_DUTY = 10,
	FAULT_STARTUP = 11,
	FAULT_REVERSE = 12,
	FAULT_QUICKSTOP = 13
} BalanceState;

typedef enum {
	CENTERING = 0,
	REVERSESTOP,
	TILTBACK_NONE,
	TILTBACK_DUTY,
	TILTBACK_HV,
	TILTBACK_LV,
	TILTBACK_TEMP,
} SetpointAdjustmentType;

typedef enum {
	OFF = 0,
	HALF,
	ON
} SwitchState;

// Balance thread
static THD_FUNCTION(balance_thread, arg);
static THD_WORKING_AREA(balance_thread_wa, 2048); // 2kb stack for this thread

static thread_t *app_thread;

// Config values
static volatile balance_config balance_conf;
static volatile imu_config imu_conf;
static systime_t loop_time;
static float startup_step_size;
static float tiltback_duty_step_size, tiltback_hv_step_size, tiltback_lv_step_size, tiltback_return_step_size;
static float torquetilt_on_step_size, torquetilt_off_step_size, turntilt_step_size;
static float tiltback_variable, tiltback_variable_max_erpm, noseangling_step_size;
static float angular_rate_kp;
static float mc_max_temp_fet;
static float mc_max_temp_mot;
static float mc_current_max;
static float mc_current_min;
static float max_continuous_current;
static bool current_beeping;
static bool duty_beeping;
static bool show_revision;
static float booster_current_acc, booster_angle_acc, booster_ramp_acc;
static float booster_current_brk, booster_angle_brk, booster_ramp_brk;

// Runtime values read from elsewhere
static float pitch_angle, last_pitch_angle, roll_angle, abs_roll_angle;
static float true_pitch_angle;
static float duty_cycle, abs_duty_cycle;
static float erpm, abs_erpm;
static float motor_current;
static float motor_position;
static float adc1, adc2;
static float max_duty_with_margin;
static SwitchState switch_state;

// Feature: ATR (Adaptive Torque Response)
static float rtkp, rtki, rti_limit, rtd_limit;
static float og_tt_target;
static float og_tt_strength;
static float tt_pid_intensity, tt_speedboost_intensity, tt_strength_uphill;
static float integral_tt_impact_uphill, integral_tt_impact_downhill;
static float acceleration, last_erpm;
static float accel_gap;
static float accelhist[ACCEL_ARRAY_SIZE];
static float accelavg;
static float tt_accel_factor;
static int erpm_sign;
static int accelidx;
static int direction_counter;
static bool braking;
static bool atr_disable;

// Feature: Turntilt
static float last_yaw_angle, yaw_angle, abs_yaw_change, last_yaw_change, yaw_change, yaw_aggregate;
static float turntilt_boost_per_erpm, yaw_aggregate_target;
static float turntilt_strength;

// Rumtime state values
static BalanceState state;
static float proportional, integral;
static float last_proportional, abs_proportional;
static float pid_value;
static float setpoint, setpoint_target, setpoint_target_interpolated;
static float noseangling_interpolated;
static float torquetilt_filtered_current, torquetilt_target, torquetilt_interpolated;
static Biquad torquetilt_current_biquad;
static float braketilt_factor, braketilt_target, braketilt_interpolated, brakestep_modifier;
static float turntilt_target, turntilt_interpolated;
static SetpointAdjustmentType setpointAdjustmentType;
static systime_t current_time, last_time, diff_time, loop_overshoot;
static float filtered_loop_overshoot, loop_overshoot_alpha, filtered_diff_time;
static systime_t fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer, fault_switch_half_timer, fault_duty_timer;
static float motor_timeout;
static systime_t brake_timeout;
static systime_t wheelslip_timer, wheelslip_end_timer, overcurrent_timer, tb_highvoltage_timer;
static float switch_warn_buzz_erpm;
static float quickstop_erpm;
static bool is_dual_switch;
static bool traction_control;

// Odometer
static systime_t odo_timer;
static int odometer_dirty;
static uint64_t odometer;

// Darkride aka upside down mode:
static bool is_upside_down;				// the board is upside down
static bool is_upside_down_started;		// dark ride has been engaged
static bool enable_upside_down;			// dark ride mode is enabled (10 seconds after fault)
static bool allow_upside_down;			// dark ride mode feature is ON (roll=180) 
static systime_t disengage_timer;
static systime_t delay_upside_down_fault;

// Feature: Reverse Stop
static float reverse_stop_step_size, reverse_tolerance, reverse_total_erpm;
static systime_t reverse_timer;
static bool use_reverse_stop, runtime_reverse_stop;

// brake amp rate limiting:
static float pid_brake_increment;

// Log values
float balance_setpoint, balance_atr, balance_carve, balance_true_pitch;
int log_balance_state; // not static so we can log it

// Function Prototypes
static void set_current(float current);
static void check_odometer(void);

// Exposed Functions
void app_balance_configure(balance_config *conf, imu_config *conf2) {
	balance_conf = *conf;
	imu_conf = *conf2;
	// Set calculated values from config
	loop_time = US2ST((int)((1000.0 / balance_conf.hertz) * 1000.0));

	motor_timeout = ((1000.0 / balance_conf.hertz)/1000.0) * 20; // Times 20 for a nice long grace period

	startup_step_size = balance_conf.startup_speed / balance_conf.hertz;
	tiltback_duty_step_size = balance_conf.tiltback_duty_speed / balance_conf.hertz;
	tiltback_hv_step_size = balance_conf.tiltback_hv_speed / balance_conf.hertz;
	tiltback_lv_step_size = balance_conf.tiltback_lv_speed / balance_conf.hertz;
	tiltback_return_step_size = balance_conf.tiltback_return_speed / balance_conf.hertz;
	torquetilt_on_step_size = balance_conf.torquetilt_on_speed / balance_conf.hertz;
	torquetilt_off_step_size = balance_conf.torquetilt_off_speed / balance_conf.hertz;
	turntilt_step_size = balance_conf.turntilt_speed / balance_conf.hertz;
	noseangling_step_size = balance_conf.noseangling_speed / balance_conf.hertz;

	mc_max_temp_fet = mc_interface_get_configuration()->l_temp_fet_start - 3;
	mc_max_temp_mot = mc_interface_get_configuration()->l_temp_motor_start - 3;

	mc_current_max = mc_interface_get_configuration()->l_current_max;
	int mcm = mc_current_max;
	float mc_max_reduce = mc_current_max - mcm;
	if (mc_max_reduce >= 0.5) {
		// reduce the max current by X% to save that for torque tilt situations
		// less than 60 peak amps makes no sense though so I'm not allowing it
		mc_current_max = fmaxf(mc_max_reduce * mc_current_max, 60);
	}

	// min current is a positive value here!
	mc_current_min = fabsf(mc_interface_get_configuration()->l_current_min);
	mcm = mc_current_min;
	float mc_min_reduce = fabsf(mc_current_min - mcm);
	if (mc_min_reduce >= 0.5) {
		// reduce the max current by X% to save that for torque tilt situations
		// less than 50 peak breaking amps makes no sense though so I'm not allowing it
		mc_current_min = fmaxf(mc_min_reduce * mc_current_min, 50);
	}

	// Decimals of abs-max specify max continuous current
	float max_abs = mc_interface_get_configuration()->l_abs_current_max;
	int mabs = max_abs;
	max_continuous_current = (max_abs - mabs) * 100;
	if (max_continuous_current < 25) {
		// anything below 25A is suspicious and will be ignored!
		max_continuous_current = mc_current_max;
	}

	// maximum amps change when braking
	pid_brake_increment = 5;//balance_conf.kd_pt1_highpass_frequency / 10;
	if (pid_brake_increment < 0.1) {
		pid_brake_increment = 5;
	}

	rtkp = balance_conf.kp;
	rtki = balance_conf.ki;
	rti_limit = balance_conf.deadzone * 10;

	max_duty_with_margin = mc_interface_get_configuration()->l_max_duty - 0.1;

	if (mc_interface_get_configuration()->m_invert_direction)
		erpm_sign = -1;
	else
		erpm_sign = 1;

	if ((balance_conf.booster_angle == 0) || (balance_conf.booster_current >= 10)) {
		// disable booster current if we detect 5.3 stable or pre ATR24 settings
		booster_current_acc = 0;
		booster_angle_acc = 180;
		booster_ramp_acc = 0;
		booster_current_brk = 0;
		booster_angle_brk = 180;
		booster_ramp_brk = 0;
	}
	else {
		// booster current = x.y
		// x = 0..9 for acceleration (maps to 0..45A)
		// y = 0..9 for braking
		int boost = balance_conf.booster_current;
		booster_current_acc = boost * 5;
		booster_current_brk = (balance_conf.booster_current - boost) * 50;

		// booster angle = x.y
		// x = 0..n for acceleration
		// y = 0..9 for braking
		boost = balance_conf.booster_angle;
		booster_angle_acc = boost;
		booster_angle_brk = (balance_conf.booster_angle - boost) * 10;

		// booster ramp = x.y
		// x = 0..n for acceleration
		// y = 0..9 for braking
		boost = balance_conf.booster_ramp;
		booster_ramp_acc = boost;
		booster_ramp_brk = (balance_conf.booster_ramp - boost) * 10;

		// use multiplication instead of division in the balance loop
		// limit to 10 Amps per degree
		booster_current_acc = fminf(10, booster_current_acc / booster_ramp_acc);
		booster_current_brk = fminf(10, booster_current_brk / booster_ramp_brk);
	}

	// Feature: Reverse Stop (ON if startup_speed ends in .1)
	// startup_speed = x.0: noticeable click on start, no reverse stop
	// startup_speed = x.1: noticeable click on start, reverse stop
	// startup_speed = x.2: stealthy start, no reverse stop
	// startup_speed = x.3: stealthy start + reverse stop
	use_reverse_stop = false;
	runtime_reverse_stop = false;
	reverse_tolerance = 50000;
	reverse_stop_step_size = 100.0 / balance_conf.hertz;
	float startup_speed = balance_conf.startup_speed;
	int ss = (int) startup_speed;
	float ss_rest = startup_speed - ss;
	if ((ss_rest > 0.09) && (ss_rest < 0.11)) {
		use_reverse_stop = true;
	}

	// Init Filters
	// loop overshoot filter hard coded to 3Hz
	loop_overshoot_alpha = 2*M_PI*((float)1/balance_conf.hertz)*3/(2*M_PI*((float)1/balance_conf.hertz)*3+1);

	float ttfilter = balance_conf.torquetilt_filter;
	if (ttfilter == 0) {
		ttfilter = 10;
		atr_disable = true;
	}
	// Torquetilt Current Biquad
	float Fc = ttfilter / balance_conf.hertz;
	biquad_config(&torquetilt_current_biquad, BQ_LOWPASS, Fc);

	angular_rate_kp = balance_conf.yaw_kp;
	if (angular_rate_kp >= 3) {
		// optimized angular P
		angular_rate_kp -= 3;
		angular_rate_kp = fminf(3, angular_rate_kp);
		// Cap the max amps to be contributed by the AngularP component
		rtd_limit = balance_conf.deadzone * 3;
		rtd_limit = fminf(20, rtd_limit);
		rtd_limit = fmaxf(5, rtd_limit);
	}
	else {
		// don't mess with angular P
		rtd_limit = 0;
	}

	// Feature: ATR:
	tt_accel_factor = fmaxf(5, balance_conf.yaw_kd);	// how many amps per acc?
	tt_accel_factor = fminf(30, tt_accel_factor);

	og_tt_strength = 0;
	if (app_get_configuration()->app_nrf_conf.address[0] == 99) {
		og_tt_strength = app_get_configuration()->app_nrf_conf.address[2] / 100;
	}

	int ttstart = balance_conf.torquetilt_start_current;
	tt_speedboost_intensity = balance_conf.torquetilt_start_current - ttstart;
	tt_speedboost_intensity = fminf(0.5, tt_speedboost_intensity); // 50% is more than enough!

	// Torque-Tilt strength is different for up vs downhills
	tt_strength_uphill = balance_conf.torquetilt_strength * 10;
	if (tt_strength_uphill > 3.5)
		tt_strength_uphill = 1.5;
	if (tt_strength_uphill < 0)
		tt_strength_uphill = 0;
	// Downhill strength must be higher since downhill amps tend to be lower than uphill amps
	//tt_strength_downhill = tt_strength_uphill * (1 + balance_conf.yaw_kp / 100);

	// Any value above 0 will increase the board angle to match the slope
	integral_tt_impact_downhill = 1.0 - (float)balance_conf.kd_biquad_lowpass / 100.0;
	integral_tt_impact_uphill = 1.0 - (float)balance_conf.kd_biquad_highpass / 100.0;
	integral_tt_impact_downhill = fminf(integral_tt_impact_downhill, 1.0);
	integral_tt_impact_downhill = fmaxf(integral_tt_impact_downhill, 0.0);
	integral_tt_impact_uphill = fminf(integral_tt_impact_uphill, 1.0);
	integral_tt_impact_uphill = fmaxf(integral_tt_impact_uphill, 0.0);
	
	// Lingering nose tilt after braking
	braketilt_factor = balance_conf.kd_pt1_highpass_frequency;
	if (braketilt_factor > 0) {
		braketilt_factor = 20 - braketilt_factor;
		if (braketilt_factor < 0) {
			braketilt_factor = 5;
		}
		// incorporate negative sign into braketilt factor instead of adding it each balance loop
		braketilt_factor = -(0.5 + braketilt_factor / 5.0);
	}
	brakestep_modifier = fminf(5, balance_conf.kd_pt1_lowpass_frequency);
	if (brakestep_modifier == 0) {
		brakestep_modifier = 1;
	}

	// Feature: Turntilt
	yaw_aggregate_target = fmaxf(50, balance_conf.yaw_ki);			// borrow yaw_ki for aggregate yaw-change target
	turntilt_boost_per_erpm = (float)balance_conf.turntilt_erpm_boost / 100.0 / (float)balance_conf.turntilt_erpm_boost_end;
	turntilt_strength = balance_conf.turntilt_strength;

	// Both switches act as one if erpm is 0
	is_dual_switch = (balance_conf.fault_adc_half_erpm == 0);
	allow_upside_down = (balance_conf.fault_roll == 180);
	enable_upside_down = false;
	is_upside_down = false;

	// Speed at which to warn users about an impending full switch fault
	switch_warn_buzz_erpm = 2000;

	// Speed below which we check for quickstop conditions
	quickstop_erpm = 200;

	// Variable nose angle adjustment / tiltback (setting is per 1000erpm, convert to per erpm)
	tiltback_variable = balance_conf.tiltback_variable / 1000;
	if (tiltback_variable > 0) {
		tiltback_variable_max_erpm = fabsf(balance_conf.tiltback_variable_max / tiltback_variable);
	} else {
		tiltback_variable_max_erpm = 100000;
	}

	// Reset loop time variables
	last_time = 0;
	filtered_loop_overshoot = 0;

	show_revision = true;
}

void app_balance_start(void) {
	// First start only, override state to startup
	state = STARTUP;

	// Allow saving of odometer
	odometer_dirty = 0;
	odometer = mc_interface_get_odometer();

	// Start the balance thread
	app_thread = chThdCreateStatic(balance_thread_wa, sizeof(balance_thread_wa), NORMALPRIO, balance_thread, NULL);
}

void app_balance_stop(void) {
	if(app_thread != NULL){
		chThdTerminate(app_thread);
		chThdWait(app_thread);
	}
	set_current(0);
}

float app_balance_get_pid_output(void) {
	return pid_value;
}
float app_balance_get_pitch_angle(void) {
	return pitch_angle;
}
float app_balance_get_roll_angle(void) {
	return roll_angle;
}
uint32_t app_balance_get_diff_time(void) {
	if (show_revision) {
		return ATR_CUSTOM_VERSION;
	}
	return ST2US(diff_time);
}
float app_balance_get_motor_current(void) {
	return motor_current;
}
uint16_t app_balance_get_state(void) {
	return state;
}
uint16_t app_balance_get_switch_state(void) {
	return switch_state;
}
float app_balance_get_adc1(void) {
	return adc1;
}
float app_balance_get_adc2(void) {
	return adc2;
}
float app_balance_get_debug1(void) {
	return 0;
}
float app_balance_get_debug2(void) {
	return 0;
}

// Internal Functions
static void reset_vars(void){
	// Clear accumulated values.
	integral = 0;
	last_proportional = 0;
	// Set values for startup
	setpoint = pitch_angle;
	setpoint_target_interpolated = pitch_angle;
	setpoint_target = 0;
	noseangling_interpolated = 0;
	torquetilt_target = 0;
	torquetilt_interpolated = 0;
	torquetilt_filtered_current = 0;
	braketilt_target = 0;
	braketilt_interpolated = 0;
	biquad_reset(&torquetilt_current_biquad);
	braketilt_interpolated = 0;
	turntilt_target = 0;
	turntilt_interpolated = 0;
	setpointAdjustmentType = CENTERING;
	state = RUNNING;
	current_time = 0;
	last_time = 0;
	diff_time = 0;
	brake_timeout = 0;
	current_beeping = false;
	duty_beeping = false;
	traction_control = false;
	pid_value = 0;

	// ATR:
	accel_gap = 0;
	direction_counter = 0;

	for (int i=0; i<40; i++)
		accelhist[i] = 0;
	accelidx = 0;
	accelavg = 0;

	// Turntilt:
	last_yaw_angle = 0;
	yaw_aggregate = 0;
}

static float get_setpoint_adjustment_step_size(void){
	switch(setpointAdjustmentType){
		case (CENTERING):
			return startup_step_size;
		case (TILTBACK_DUTY):
			return tiltback_duty_step_size;
		case (TILTBACK_HV):
		case (TILTBACK_TEMP):
			return tiltback_hv_step_size;
		case (TILTBACK_LV):
			return tiltback_lv_step_size;
		case (TILTBACK_NONE):
			return tiltback_return_step_size;
		case (REVERSESTOP):
			return reverse_stop_step_size;
		default:
			;
	}
	return 0;
}

// Read ADCs and determine switch state
static SwitchState check_adcs(void)
{
	SwitchState sw_state;
	adc1 = (((float)ADC_Value[ADC_IND_EXT])/4095) * V_REG;
#ifdef ADC_IND_EXT2
	adc2 = (((float)ADC_Value[ADC_IND_EXT2])/4095) * V_REG;
#else
	adc2 = 0.0;
#endif

	// Calculate switch state from ADC values
	if(balance_conf.fault_adc1 == 0 && balance_conf.fault_adc2 == 0){ // No Switch
		sw_state = ON;
	}else if(balance_conf.fault_adc2 == 0){ // Single switch on ADC1
		if(adc1 > balance_conf.fault_adc1){
			sw_state = ON;
		} else {
			sw_state = OFF;
		}
	}else if(balance_conf.fault_adc1 == 0){ // Single switch on ADC2
		if(adc2 > balance_conf.fault_adc2){
			sw_state = ON;
		} else {
			sw_state = OFF;
		}
	}else{ // Double switch
		if(adc1 > balance_conf.fault_adc1 && adc2 > balance_conf.fault_adc2){
			sw_state = ON;
		}else if(adc1 > balance_conf.fault_adc1 || adc2 > balance_conf.fault_adc2){
			// 5 seconds after stopping we allow starting with a single sensor (e.g. for jump starts)
			bool is_simple_start = ST2S(current_time - disengage_timer) > 5;
			if (is_dual_switch || is_simple_start)
				sw_state = ON;
			else
				sw_state = HALF;
		}else{
			sw_state = OFF;
		}
	}

	/*
	 * Use external buzzer to notify rider of foot switch faults.
	 */
#ifdef HAS_EXT_BUZZER
	if ((sw_state == OFF) && (state <= RUNNING_TILTBACK)) {
		if (abs_erpm > switch_warn_buzz_erpm) {
			// If we're at riding speed and the switch is off => ALERT the user
			// set force=true since this could indicate an imminent shutdown/nosedive
			beep_on(true);
		}
		else {
			// if we drop below riding speed stop buzzing
			beep_off(false);
		}
	}
	else {
		// if the switch comes back on we stop buzzing
		beep_off(false);
	}
#endif

	return sw_state;
}

// Fault checking order does not really matter. From a UX perspective, switch should be before angle.
static bool check_faults(bool ignoreTimers){
	// Aggressive reverse stop in case the board runs off when upside down
	if (is_upside_down) {
		if (erpm > 1000) {
			// erpms are also reversed when upside down!
			if ((ST2MS(current_time - fault_switch_timer) > 100) ||
				(erpm > 2000) ||
				((state == RUNNING_WHEELSLIP) && (ST2S(current_time - delay_upside_down_fault) > 1) &&
				 (ST2MS(current_time - fault_switch_timer) > 30)) ) {
				// Trigger FAULT_REVERSE when board is going reverse AND
				// going > 2mph for more than 100ms
				// going > 4mph
				// detecting wheelslip (aka excorcist wiggle) after the first second
				state = FAULT_REVERSE;
				return true;
			}
		}
		else {
			fault_switch_timer = current_time;
			if (erpm > 300) {
				// erpms are also reversed when upside down!
				if (ST2MS(current_time - fault_angle_roll_timer) > 500){
					state = FAULT_REVERSE;
					return true;
				}
			}
			else {
				fault_angle_roll_timer = current_time;
			}
		}
		if (switch_state == ON) {
			// allow turning it off by engaging foot sensors
			state = FAULT_SWITCH_HALF;
			return true;
		}
	}
	else {
		// Check switch
		// Switch fully open
		if(switch_state == OFF){
			if(ST2MS(current_time - fault_switch_timer) > balance_conf.fault_delay_switch_full || ignoreTimers){
				state = FAULT_SWITCH_FULL;
				return true;
			}
			// low speed (below 6 x half-fault threshold speed):
			else if ((abs_erpm < balance_conf.fault_adc_half_erpm * 6)
					 && (ST2MS(current_time - fault_switch_timer) > balance_conf.fault_delay_switch_half)){
				state = FAULT_SWITCH_FULL;
				return true;
			}
			else if ((abs_erpm < quickstop_erpm) && (fabsf(true_pitch_angle) > 14) && (SIGN(true_pitch_angle) == SIGN(erpm))/* && (!is_dual_switch)*/) {
				// QUICK STOP (now enabled for POSI)
				state = FAULT_QUICKSTOP;
				return true;
			}
		} else {
			fault_switch_timer = current_time;
		}

		// Feature: Reverse-Stop
		if(setpointAdjustmentType == REVERSESTOP){
			//  Taking your foot off entirely while reversing? Ignore delays
			if (switch_state == OFF) {
				state = FAULT_SWITCH_FULL;
				return true;
			}
			if (fabsf(true_pitch_angle) > 15) {
				state = FAULT_REVERSE;
				return true;
			}
			// Above 10 degrees for a half a second? Switch it off
			if ((fabsf(true_pitch_angle) > 10) && (ST2MS(current_time - reverse_timer) > 500)) {
				state = FAULT_REVERSE;
				return true;
			}
			// Above 5 degrees for a full second? Switch it off
			if ((fabsf(true_pitch_angle) > 5) && (ST2MS(current_time - reverse_timer) > 1000)) {
				state = FAULT_REVERSE;
				return true;
			}
			if (reverse_total_erpm > reverse_tolerance * 3) {
				state = FAULT_REVERSE;
				return true;
			}
			if (fabsf(true_pitch_angle) < 5) {
				reverse_timer = current_time;
			}
		}

		// Switch partially open and stopped
		if (!is_dual_switch){
			if((switch_state == HALF || switch_state == OFF) && abs_erpm < balance_conf.fault_adc_half_erpm){
				if(ST2MS(current_time - fault_switch_half_timer) > balance_conf.fault_delay_switch_half || ignoreTimers){
					state = FAULT_SWITCH_HALF;
					return true;
				}
			} else {
				fault_switch_half_timer = current_time;
			}
		}

		// Check roll angle
		if(fabsf(roll_angle) > balance_conf.fault_roll){
			if(ST2MS(current_time - fault_angle_roll_timer) > balance_conf.fault_delay_roll || ignoreTimers){
				state = FAULT_ANGLE_ROLL;
				return true;
			}
		}else{
			fault_angle_roll_timer = current_time;

			if (allow_upside_down) {
				if((fabsf(roll_angle) > 100) && (fabsf(roll_angle) < 135)) {
					state = FAULT_ANGLE_ROLL;
					return true;
				}
			}
		}
	}

	// Check pitch angle
	if(fabsf(true_pitch_angle) > balance_conf.fault_pitch){
		if(ST2MS(current_time - fault_angle_pitch_timer) > balance_conf.fault_delay_pitch || ignoreTimers){
			state = FAULT_ANGLE_PITCH;
			return true;
		}
	}else{
		fault_angle_pitch_timer = current_time;
	}

	// Check for duty
	if(abs_duty_cycle > balance_conf.fault_duty){
		if(ST2MS(current_time - fault_duty_timer) > balance_conf.fault_delay_duty || ignoreTimers){
			state = FAULT_DUTY;
			return true;
		}
	} else {
		fault_duty_timer = current_time;
	}

	return false;
}

static void calculate_setpoint_target(void){
	if (GET_INPUT_VOLTAGE() < balance_conf.tiltback_hv) {
		tb_highvoltage_timer = current_time;
	}

	if(setpointAdjustmentType == CENTERING && setpoint_target_interpolated != setpoint_target){
		// Ignore tiltback during centering sequence
		state = RUNNING;
	}else if (setpointAdjustmentType == REVERSESTOP) {
		// accumalete erpms:
		reverse_total_erpm += erpm;
		if (fabsf(reverse_total_erpm) > reverse_tolerance) {
			// tilt down by 10 degrees after 50k aggregate erpm
			setpoint_target = 10 * (fabsf(reverse_total_erpm)-reverse_tolerance) / 50000;
		}
		else {
			if (fabsf(reverse_total_erpm) <= reverse_tolerance/2) {
				if (erpm >= 0){
					setpointAdjustmentType = TILTBACK_NONE;
					reverse_total_erpm = 0;
					setpoint_target = 0;
					integral = 0;
				}
			}
		}
	}else if ((fabs(acceleration) > 10) &&				// this isn't normal, either wheelslip or wheel getting stuck
			  (SIGN(acceleration) == SIGN(erpm)) &&		// we only act on wheelslip, not when the wheel gets stuck
			  (abs_duty_cycle > 0.3) &&
			  (abs_erpm > 1500))						// acceleration can jump a lot at very low speeds
	{
		state = RUNNING_WHEELSLIP;
		setpointAdjustmentType = TILTBACK_NONE;
		wheelslip_timer = current_time;
		if (is_upside_down) {
			traction_control = true;
		}
	}else if (state == RUNNING_WHEELSLIP) {
		if (fabsf(acceleration) < 10) {
			// acceleration is slowing down, traction control seems to have worked
			traction_control = false;
		}
		// Remain in wheelslip state for at least 500ms to avoid any overreactions
		if (abs_duty_cycle > max_duty_with_margin) {
			wheelslip_timer = current_time;
		}
		else if (ST2S(current_time - wheelslip_timer) > 0.5) {
			if (abs_duty_cycle < 0.7) {
				// Leave wheelslip state only if duty < 70%
				traction_control = false;
				state = RUNNING;
			}
		}
	}else if(abs_duty_cycle > balance_conf.tiltback_duty){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_duty_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_duty_angle;
		}
		setpointAdjustmentType = TILTBACK_DUTY;
		state = RUNNING_TILTBACK;
	}else if(GET_INPUT_VOLTAGE() > balance_conf.tiltback_hv){
		beep_alert(3, 0);	// Triple-beep
		if ((ST2MS(current_time - tb_highvoltage_timer) > 500) ||
			(GET_INPUT_VOLTAGE() > balance_conf.tiltback_hv + 1)) {
			// 500ms have passed or voltage is another volt higher, time for some tiltback
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_hv_angle;
			} else {
				setpoint_target = -balance_conf.tiltback_hv_angle;
			}
			setpointAdjustmentType = TILTBACK_HV;
			state = RUNNING_TILTBACK;
		}
		else {
			// The rider has 500ms to react to the triple-beep, or maybe it was just a short spike
			setpointAdjustmentType = TILTBACK_NONE;
			state = RUNNING;
		}
	}else if(mc_interface_temp_fet_filtered() > mc_max_temp_fet){
		// Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
		beep_alert(3, 1);	// Triple-beep (long beeps)
		if(mc_interface_temp_fet_filtered() > (mc_max_temp_fet + 1)) {
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_lv_angle;
			} else {
				setpoint_target = -balance_conf.tiltback_lv_angle;
			}
			setpointAdjustmentType = TILTBACK_TEMP;
			state = RUNNING_TILTBACK;
		}
		else {
			// The rider has 1 degree Celsius left before we start tilting back
			setpointAdjustmentType = TILTBACK_NONE;
			state = RUNNING;
		}
	}else if(mc_interface_temp_motor_filtered() > mc_max_temp_mot){
		// Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
		beep_alert(3, 1);	// Triple-beep (long beeps)
		if(mc_interface_temp_motor_filtered() > (mc_max_temp_mot + 1)) {
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_lv_angle;
			} else {
				setpoint_target = -balance_conf.tiltback_lv_angle;
			}
			setpointAdjustmentType = TILTBACK_TEMP;
			state = RUNNING_TILTBACK;
		}
		else {
			// The rider has 1 degree Celsius left before we start tilting back
			setpointAdjustmentType = TILTBACK_NONE;
			state = RUNNING;
		}
	}else if(GET_INPUT_VOLTAGE() < balance_conf.tiltback_lv){
		beep_alert(3, 0);	// Triple-beep
		float abs_motor_current = fabsf(motor_current);
		float vdelta = balance_conf.tiltback_lv - GET_INPUT_VOLTAGE();
		float ratio = vdelta * 20 / abs_motor_current;
		// When to do LV tiltback:
		// a) we're 2V below lv threshold
		// b) motor current is small (we cannot assume vsag)
		// c) we have more than 20A per Volt of difference (we tolerate some amount of vsag)
		if ((vdelta > 2) || (abs_motor_current < 5) || (ratio > 1)) {
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_lv_angle;
			} else {
				setpoint_target = -balance_conf.tiltback_lv_angle;
			}
			setpointAdjustmentType = TILTBACK_LV;
			state = RUNNING_TILTBACK;
		}
		else {
			setpointAdjustmentType = TILTBACK_NONE;
			setpoint_target = 0;
			state = RUNNING;
		}
	}else{
		// Normal running
		if (use_reverse_stop && (erpm < -200) && !is_upside_down) {
			setpointAdjustmentType = REVERSESTOP;
			reverse_timer = current_time;
			reverse_total_erpm = 0;
		}
		else {
			setpointAdjustmentType = TILTBACK_NONE;
		}
		setpoint_target = 0;
		state = RUNNING;
	}

	if ((state == RUNNING_WHEELSLIP) && (abs_duty_cycle > max_duty_with_margin)) {
		setpoint_target = 0;
	}
	if (is_upside_down && (state == RUNNING)) {
		state = RUNNING_UPSIDEDOWN;
		if (!is_upside_down_started) {
			// right after flipping when first engaging dark ride we add a 1 second grace period
			// before aggressively checking for board wiggle (based on acceleration)
			is_upside_down_started = true;
			delay_upside_down_fault = current_time;
		}
	}

	if (setpointAdjustmentType == TILTBACK_DUTY) {
		if (balance_conf.tiltback_duty_angle == 0) {
			beep_on(true);
			duty_beeping = true;
		}
	}
	else {
		if (duty_beeping) {
			beep_off(false);
		}
	}
}

static void calculate_setpoint_interpolated(void){
	if(setpoint_target_interpolated != setpoint_target){
		// If we are less than one step size away, go all the way
		if(fabsf(setpoint_target - setpoint_target_interpolated) < get_setpoint_adjustment_step_size()){
			setpoint_target_interpolated = setpoint_target;
		}else if (setpoint_target - setpoint_target_interpolated > 0){
			setpoint_target_interpolated += get_setpoint_adjustment_step_size();
		}else{
			setpoint_target_interpolated -= get_setpoint_adjustment_step_size();
		}
	}
}

static void apply_noseangling(void){
	if (state != RUNNING_WHEELSLIP) {
		// Nose angle adjustment, add variable then constant tiltback
		float noseangling_target = 0;
		if (fabsf(erpm) > tiltback_variable_max_erpm) {
			noseangling_target = fabsf(balance_conf.tiltback_variable_max) * SIGN(erpm);
		} else {
			noseangling_target = tiltback_variable * erpm;
		}

		if(erpm > balance_conf.tiltback_constant_erpm){
			noseangling_target += balance_conf.tiltback_constant;
		} else if(erpm < -balance_conf.tiltback_constant_erpm){
			noseangling_target += -balance_conf.tiltback_constant;
		}

		if(fabsf(noseangling_target - noseangling_interpolated) < noseangling_step_size){
			noseangling_interpolated = noseangling_target;
		}else if (noseangling_target - noseangling_interpolated > 0){
			noseangling_interpolated += noseangling_step_size;
		}else{
			noseangling_interpolated -= noseangling_step_size;
		}
	}
	setpoint += noseangling_interpolated;
}

static float expected_acc;
static void apply_torquetilt(void){
	torquetilt_filtered_current = biquad_process(&torquetilt_current_biquad, motor_current);
	int torque_sign = SIGN(torquetilt_filtered_current);
	float abs_torque = fabsf(torquetilt_filtered_current);
	float torque_offset = balance_conf.torquetilt_start_current;
	float accel_factor2 = tt_accel_factor * 1.3;
	float step_size;

	if ((abs_erpm > 250) && (torque_sign != SIGN(erpm))) {
		// current is negative, so we are braking or going downhill
		// high currents downhill are less likely
		//torquetilt_strength = tt_strength_downhill;
		braking = true;
	}
	else {
		braking = false;
	}

	// Skip torque tilt logic if strength is 0
	if (tt_strength_uphill == 0)
		return;

	if (atr_disable) {
		// Do stock 5.3 torque tilt: (comment from Mitch Lustig)
		// Take abs motor current, subtract start offset, and take the max of that with 0 to
		// get the current above our start threshold (absolute).
		// Then multiply it by "power" to get our desired angle, and min with the limit to respect boundaries.
		// Finally multiply it by sign motor current to get directionality back
		torquetilt_target = fminf(fmaxf((fabsf(torquetilt_filtered_current) - balance_conf.torquetilt_start_current), 0) * balance_conf.torquetilt_strength, balance_conf.torquetilt_angle_limit) * SIGN(torquetilt_filtered_current);

		if((torquetilt_interpolated - torquetilt_target > 0 && torquetilt_target > 0) || (torquetilt_interpolated - torquetilt_target < 0 && torquetilt_target < 0)){
			step_size = torquetilt_off_step_size;
		}else{
			step_size = torquetilt_on_step_size;
		}
	}
	else {
		// Are we dealing with a free-spinning wheel?
		// If yes, don't change the torquetilt till we got traction again
		// instead slightly decrease it each cycle
		if (state == RUNNING_WHEELSLIP) {
			torquetilt_interpolated *= 0.995;
			torquetilt_target *= 0.99;
			braketilt_interpolated *= 0.995;
			braketilt_target *= 0.99;
			setpoint += torquetilt_interpolated + braketilt_interpolated;
			wheelslip_end_timer = current_time;
			return;
		}
		else {
			if (ST2MS(current_time - wheelslip_end_timer) < 100) {
				// for 100ms after wheelslip we still don't do ATR to allow the wheel to decelerate
				if (balance_conf.yaw_current_clamp > 1) beep_alert(1, 0);
				torquetilt_interpolated *= 0.998;
				torquetilt_target *= 0.999;
				braketilt_interpolated *= 0.998;
				braketilt_target *= 0.999;
				setpoint += torquetilt_interpolated + braketilt_interpolated;
				return;
			}
			else if ((fabsf(acceleration) > 10) && (abs_erpm > 1000)) {
				if (balance_conf.yaw_current_clamp > 0) {
					if (balance_conf.yaw_current_clamp > 1) beep_alert(1, 0);
					torquetilt_interpolated *= 0.998;
					torquetilt_target *= 0.999;
					braketilt_interpolated *= 0.998;
					braketilt_target *= 0.999;
					setpoint += torquetilt_interpolated + braketilt_interpolated;
					return;
				}
			}
		}

		float torquetilt_strength = tt_strength_uphill;
		// from 3000 to 6000 erpm gradually crank up the torque response
		if ((abs_erpm > 3000) && (!braking)) {
			float speedboost = (abs_erpm - 3000) / 3000;
			speedboost = fminf(1, speedboost) * tt_speedboost_intensity;
			torquetilt_strength += tt_strength_uphill * speedboost;
		}

		// compare measured acceleration to expected acceleration
		float measured_acc = fmaxf(acceleration, -5);
		measured_acc = fminf(acceleration, 5);

		// expected acceleration is proportional to current (minus an offset, required to balance/maintain speed)
		//XXXXXfloat expected_acc;
		if (abs_torque < 25) {
			expected_acc = (torquetilt_filtered_current - SIGN(erpm) * torque_offset) / tt_accel_factor;
		}
		else {
			// primitive linear approximation of non-linear torque-accel relationship
			expected_acc = (torque_sign * 25 - SIGN(erpm) * torque_offset) / tt_accel_factor;
			expected_acc += torque_sign * (abs_torque - 25) / accel_factor2;
		}

		bool forward = (erpm > 0);
		if ((abs_erpm < 250) && (abs_torque > 30)) {
			forward = (expected_acc > 0);
		}

		float acc_diff = expected_acc - measured_acc;

		if (abs_erpm > 2000)
			accel_gap = 0.9 * accel_gap + 0.1 * acc_diff;
		else if (abs_erpm > 1000)
			accel_gap = 0.95 * accel_gap + 0.05 * acc_diff;
		else if (abs_erpm > 250)
			accel_gap = 0.98 * accel_gap + 0.02 * acc_diff;
		else {
			accel_gap = 0;
		}

		// now torquetilt target is purely based on gap between expected and actual acceleration
		float new_ttt = torquetilt_strength * accel_gap;

		if (!braking && (abs_erpm > 250))  {
			float og_tt_angle_limit = 3;
			float og_tt_start_current = 15;
			og_tt_target = (abs_torque - og_tt_start_current) * og_tt_strength;
			og_tt_target = fminf(fmaxf(og_tt_target, 0), og_tt_angle_limit);
			if (og_tt_target > fabsf(new_ttt)) {
				new_ttt = og_tt_target * torque_sign;
			}
		} else if (abs_erpm <= 250) {
			// TODO: fall back to og_tt!
		}

		// braking also should cause setpoint change lift, causing a delayed lingering nose lift
		if ((braketilt_factor < 0) && braking && (abs_erpm > 2000)) {
			// negative currents alone don't necessarily consitute active braking, look at proportional:
			if (SIGN(proportional) != SIGN(erpm)) {
				float downhill_damper = 1;
				// if we're braking on a downhill we don't want braking to lift the setpoint quite as much
				if (((erpm > 1000) && (accel_gap < -1)) ||
					((erpm < -1000) && (accel_gap > 1))) {
					downhill_damper += fabsf(accel_gap) / 2;
				}
				braketilt_target = proportional / braketilt_factor / downhill_damper;
				if (downhill_damper > 2) {
					// steep downhills, we don't enable this feature at all!
					braketilt_target = 0;
				}
			}
		}
		else {
			braketilt_target = 0;
		}

		torquetilt_target = torquetilt_target * 0.95 + 0.05 * new_ttt;
		torquetilt_target = fminf(torquetilt_target, balance_conf.torquetilt_angle_limit);
		torquetilt_target = fmaxf(torquetilt_target, -balance_conf.torquetilt_angle_limit);

		// Key to keeping the board level and consistent is to determine the appropriate step size!
		// We want to react quickly to changes, but we don't want to overreact to glitches in acceleration data
		// or trigger oscillations...
		if (forward) {
			if (torquetilt_interpolated < 0) {
				// downhill
				if (torquetilt_interpolated < torquetilt_target) {
					// to avoid oscillations we go down slower than we go up
					step_size = torquetilt_off_step_size;
				}
				else {
					// torquetilt is increasing
					if (braking) {
						// braking downhill, do it only half as aggressively as pushing uphill
						// a little short taildrag is acceptable
						step_size = torquetilt_on_step_size;
					}
					else {
						// we arent braking yet there's reverse torquetilt? How does this happen??
						step_size = torquetilt_on_step_size;
					}
				}
			}
			else {
				// uphill or other heavy resistance (grass, mud, etc)
				if ((torquetilt_target > -3) && (torquetilt_interpolated > torquetilt_target)) {
					// torquetilt winding down (current torquetilt is bigger than the target)
					// normal wind down case: to avoid oscillations we go down slower than we go up
					step_size = torquetilt_off_step_size;
				}else{
					// standard case of increasing torquetilt
					step_size = torquetilt_on_step_size;
				}
			}
		}
		else {
			if (torquetilt_interpolated > 0) {
				// downhill
				if (torquetilt_interpolated > torquetilt_target) {
					// to avoid oscillations we go down slower than we go up
					step_size = torquetilt_off_step_size;
				}
				else {
					// torquetilt is increasing
					if (braking) {
						// braking downhill, do it only half as aggressively as pushing uphill
						// a little short taildrag is acceptable
						step_size = torquetilt_on_step_size;
					}
					else {
						// we arent braking yet there's reverse torquetilt? Probably impossible
						step_size = torquetilt_on_step_size;
					}
				}
			}
			else {
				// uphill or other heavy resistance (grass, mud, etc)
				if ((torquetilt_target < 3) && (torquetilt_interpolated < torquetilt_target)) {
					// normal wind down case: to avoid oscillations we go down slower than we go up
					step_size = torquetilt_off_step_size;
				}else{
					// standard case of increasing torquetilt
					step_size = torquetilt_on_step_size;
				}
			}
		}
	}
	// when slow then erpm data is especially choppy, causing fake spikes in acceleration
	// mellow down the reaction to reduce noticeable oscillations
	if (abs_erpm < 500) {
		step_size /= 2;
	}

	if(fabsf(torquetilt_target - torquetilt_interpolated) < step_size){
		torquetilt_interpolated = torquetilt_target;
	}else if (torquetilt_target - torquetilt_interpolated > 0){
		torquetilt_interpolated += step_size;
	}else{
		torquetilt_interpolated -= step_size;
	}

	step_size = torquetilt_off_step_size / brakestep_modifier;
	if(fabsf(braketilt_target) > fabsf(braketilt_interpolated)) {
		step_size = torquetilt_on_step_size * 1.5;
	}
	else if (abs_erpm < 800) {
		step_size = torquetilt_on_step_size;
	}

	if(fabsf(braketilt_target - braketilt_interpolated) < step_size){
		braketilt_interpolated = braketilt_target;
	}else if (braketilt_target - braketilt_interpolated > 0){
		braketilt_interpolated += step_size;
	}else{
		braketilt_interpolated -= step_size;
	}

	setpoint += torquetilt_interpolated + braketilt_interpolated;
}

static void apply_turntilt(void){
	if (turntilt_strength == 0) {
		return;
	}
	// Apply cutzone
	float abs_yaw_scaled = abs_yaw_change * 100;
	if((abs_yaw_scaled < balance_conf.turntilt_start_angle) || (state != RUNNING)) {
		turntilt_target = 0;
	}
	else {
		// Calculate desired angle
		turntilt_target = abs_yaw_change * turntilt_strength;

		// Apply speed scaling
		float boost;
		if(abs_erpm < balance_conf.turntilt_erpm_boost_end){
			boost = 1.0 + abs_erpm * turntilt_boost_per_erpm;
		}else{
			boost = 1.0 + (float)balance_conf.turntilt_erpm_boost / 100.0;
		}
		turntilt_target *= boost;

		// Increase turntilt based on aggregate yaw change (at most: double it)
		float aggregate_damper = 1.0;
		if (abs_erpm < 2000) {
			aggregate_damper = 0.5;
		}
		boost = 1 + aggregate_damper * fabsf(yaw_aggregate) / yaw_aggregate_target;
		boost = fminf(boost, 2);
		turntilt_target *= boost;

		// Limit angle to max angle
		turntilt_target = fminf(turntilt_target, balance_conf.turntilt_angle_limit);

		// Disable below erpm threshold otherwise add directionality
		if(abs_erpm < balance_conf.turntilt_start_erpm){
			turntilt_target = 0;
		}else {
			turntilt_target *= SIGN(erpm);
		}

		// ATR interference: Reduce turntilt_target during moments of high torque response
		float atr_min = 2;
		float atr_max = 5;
		if (SIGN(torquetilt_target) != SIGN(turntilt_target)) {
			// further reduced turntilt during moderate to steep downhills
			atr_min = 1;
			atr_max = 4;
		}
		if (fabsf(torquetilt_target) > atr_min) {
			// Start scaling turntilt when ATR>2, down to 0 turntilt for ATR > 5 degrees
			float atr_scaling = (atr_max - fabsf(torquetilt_target)) / (atr_max-atr_min);
			if (atr_scaling < 0) {
				atr_scaling = 0;
				// during heavy torque response clear the yaw aggregate too
				yaw_aggregate = 0;
			}
			turntilt_target *= atr_scaling;
		}
		if (fabsf(pitch_angle - noseangling_interpolated) > 4) {
			// no setpoint changes during heavy acceleration or braking
			turntilt_target = 0;
			yaw_aggregate = 0;
		}
	}

	// Move towards target limited by max speed
	if(fabsf(turntilt_target - turntilt_interpolated) < turntilt_step_size){
		turntilt_interpolated = turntilt_target;
	}else if (turntilt_target - turntilt_interpolated > 0){
		turntilt_interpolated += turntilt_step_size;
	}else{
		turntilt_interpolated -= turntilt_step_size;
	}

	setpoint += turntilt_interpolated;
}

static void brake(void){
	// Brake timeout logic
	if(balance_conf.brake_timeout > 0 && (abs_erpm > 1 || brake_timeout == 0)){
		brake_timeout = current_time + S2ST(balance_conf.brake_timeout);
	}
	if(brake_timeout != 0 && current_time > brake_timeout){
		return;
	}

	// Reset the timeout
	timeout_reset();
	// Set current
	mc_interface_set_brake_current(balance_conf.brake_current);
}

static void set_current(float current){
	// Reset the timeout
	timeout_reset();
	// Set the current delay
	mc_interface_set_current_off_delay(motor_timeout);
	// Set Current
	mc_interface_set_current(current);
}

static THD_FUNCTION(balance_thread, arg) {
	(void)arg;
	chRegSetThreadName("APP_BALANCE");

	while (!chThdShouldTerminateX()) {
		// Update times
		current_time = chVTGetSystemTimeX();
		if(last_time == 0){
		  last_time = current_time;
		}
		diff_time = current_time - last_time;
		filtered_diff_time = 0.03 * diff_time + 0.97 * filtered_diff_time; // Purely a metric
		last_time = current_time;
		if(balance_conf.loop_time_filter > 0){
			loop_overshoot = diff_time - (loop_time - roundf(filtered_loop_overshoot));
			filtered_loop_overshoot = loop_overshoot_alpha * loop_overshoot + (1-loop_overshoot_alpha)*filtered_loop_overshoot;
		}

		// Read values for GUI
		motor_current = mc_interface_get_tot_current_directional_filtered();
		motor_position = mc_interface_get_pid_pos_now();

		// Get the IMU values
		roll_angle = RAD2DEG_f(imu_ref_get_roll());
		abs_roll_angle = fabsf(roll_angle);

		if (allow_upside_down) {
			if (is_upside_down) {
				if (abs_roll_angle < 120) {
					is_upside_down = false;
				}
			} else if (enable_upside_down) {
				if (abs_roll_angle > 150) {
					is_upside_down = true;
					is_upside_down_started = false;
					pitch_angle = -pitch_angle;
				}
			}
		}

		// True pitch is derived from the secondary IMU filter running with kp=0.3 or 0.2
		true_pitch_angle = RAD2DEG_f(imu_ref_get_pitch());
		last_pitch_angle = pitch_angle;
		pitch_angle = RAD2DEG_f(imu_get_pitch());
		if (is_upside_down) {
			pitch_angle = -pitch_angle;
			true_pitch_angle = -true_pitch_angle;
		}

		duty_cycle = mc_interface_get_duty_cycle_now();
		abs_duty_cycle = fabsf(duty_cycle);
		erpm = mc_interface_get_rpm();
		abs_erpm = fabsf(erpm);

		// Torque tilt:
		float smooth_erpm = erpm_sign * mcpwm_foc_get_smooth_erpm();
		float acceleration_raw = smooth_erpm - last_erpm;
		last_erpm = smooth_erpm;

		accelavg += (acceleration_raw - accelhist[accelidx]) / ACCEL_ARRAY_SIZE;
		accelhist[accelidx] = acceleration_raw;
		accelidx++;
		if (accelidx == ACCEL_ARRAY_SIZE)
			accelidx = 0;

		acceleration = accelavg;
		
		// Turn tilt:
		yaw_angle = imu_ref_get_yaw() * 180.0f / M_PI;
		float new_change = yaw_angle - last_yaw_angle;
		bool unchanged = false;
		if ((new_change == 0) // Exact 0's only happen when the IMU is not updating between loops
			|| (fabsf(new_change) > 100)) // yaw flips signs at 180, ignore those changes
		{
			new_change = last_yaw_change;
			unchanged = true;
		}
		last_yaw_change = new_change;
		last_yaw_angle = yaw_angle;

		// To avoid overreactions at low speed, limit change here:
		new_change = fminf(new_change, 0.10);
		new_change = fmaxf(new_change, -0.10);
		yaw_change = yaw_change * 0.8 + 0.2 * (new_change);
		// Clear the aggregate yaw whenever we change direction
		if (SIGN(yaw_change) != SIGN(yaw_aggregate))
			yaw_aggregate = 0;
		abs_yaw_change = fabsf(yaw_change);
		if ((abs_yaw_change > 0.04) && !unchanged)	// don't count tiny yaw changes towards aggregate
			yaw_aggregate += yaw_change;

		switch_state = check_adcs();

		float pid_prop = 0, pid_integral = 0, pid_angular_rate = 0, pid_booster = 0;
		log_balance_state = state;
		balance_setpoint = setpoint;
		balance_atr = torquetilt_interpolated;
		balance_carve = turntilt_interpolated;
		balance_true_pitch = true_pitch_angle;
		float new_pid_value = 0;
		
		// Control Loop State Logic
		switch(state){
			case (STARTUP):
				// Disable output
				brake();
				if(imu_startup_done()){
					reset_vars();
					state = FAULT_STARTUP; // Trigger a fault so we need to meet start conditions to start

					// Let the rider know that the board is ready (one short beep)
					beep_alert(1, false);
					// Are we within 5V of the LV tiltback threshold? Issue 1 beep for each volt below that
					float bat_volts = GET_INPUT_VOLTAGE();
					float threshold = balance_conf.tiltback_lv + 5;
					if (bat_volts < threshold) {
						int beeps = (int)fminf(6, threshold - bat_volts);
						beep_alert(beeps, true);
					}
				}
				break;
			case (RUNNING):
			case (RUNNING_TILTBACK):
			case (RUNNING_WHEELSLIP):
			case (RUNNING_UPSIDEDOWN):
				log_balance_state = state + (setpointAdjustmentType << 4);
				odometer_dirty = 1;

				// Check for faults
				if(check_faults(false)){
					break;
				}

				enable_upside_down = true;
				disengage_timer = current_time;

				// Calculate setpoint and interpolation
				calculate_setpoint_target();
				calculate_setpoint_interpolated();
				setpoint = setpoint_target_interpolated;
				if (!is_upside_down) {
					apply_noseangling();
					apply_torquetilt();
					apply_turntilt();
				}

				// Do PID maths
				proportional = setpoint - pitch_angle;
				if (true || (state != RUNNING_WHEELSLIP) || (abs_duty_cycle < max_duty_with_margin)) {
					integral = integral + proportional;

					if (!atr_disable) {
						// Produce controlled nose/tail lift with increased torque
						float tt_impact;
						if (torquetilt_interpolated < 0)
							// Downhill tail lift doesn't need to be as high as uphill nose lift
							tt_impact = integral_tt_impact_downhill;
						else {
							tt_impact = integral_tt_impact_uphill;

							const float max_impact_erpm = 2500;
							const float starting_impact = 0.5;
							if (abs_erpm < max_impact_erpm) {
								// Reduced nose lift at lower speeds
								// Creates a value between 0.5 and 1.0
								float erpm_scaling = fmaxf(starting_impact, abs_erpm / max_impact_erpm);
								tt_impact = (1.0 - (1.0 - tt_impact) * erpm_scaling);
							}
						}
						integral -= torquetilt_interpolated * tt_impact;
					}
				} else {
					integral = integral * 0.99;
				}
				// D-term removed, replaced by rate P!!

				if (setpointAdjustmentType == REVERSESTOP) {
					integral = integral * 0.9;
				}
				if (runtime_reverse_stop) {
					if (erpm > 0) {
						// ensure we're only runtime-enabling reverse stop when it's safe
						use_reverse_stop = true;
						runtime_reverse_stop = false;
					}
				}

				pid_prop = (rtkp * proportional);

				if (rtki > 0) {
					pid_integral = (rtki * integral);

					// integral limiting using deadzone
					if(rti_limit > 0) {
						pid_integral = fminf(rti_limit, fabsf(pid_integral));
						pid_integral *= SIGN(integral);
						integral = pid_integral / rtki;
					}
				}

				new_pid_value = pid_prop + pid_integral;

				last_proportional = proportional;

				// True Booster, based on true angle
				float true_proportional = setpoint - true_pitch_angle;
				float abs_proportional = fabsf(true_proportional);
				float boost_angle = braking ? booster_angle_brk : booster_angle_acc;

				if(abs_proportional > boost_angle){
					float boost_ramp = booster_ramp_acc;
					float boost_current = booster_current_acc;
					if (braking) {
						boost_ramp = booster_ramp_brk;
						boost_current = booster_current_brk;
					}
					if(abs_proportional - boost_angle < boost_ramp){
						pid_booster = boost_current * SIGN(true_proportional) * (abs_proportional - boost_angle);
					}else{
						pid_booster = boost_current * SIGN(true_proportional) * boost_ramp;
					}
				}
				new_pid_value += pid_booster;

				// Add angular rate to pid_value:
				float gyro[3];
				imu_get_gyro(gyro);

				pid_angular_rate = -gyro[1] * angular_rate_kp;
				if (is_upside_down) {
					pid_angular_rate = -pid_angular_rate;
				}
				
				// Optimized implementation of Angular Rate P:
				if (rtd_limit > 0) {
					// Allow high dampening, limit reinforcing
					if (SIGN(pid_angular_rate) == SIGN(pid_prop)) {
						// reinforce proportional at half the intensity only
						pid_angular_rate /= 2;
						pid_angular_rate = SIGN(pid_angular_rate) *
							fminf(rtd_limit / 3, fabsf(pid_angular_rate));
					}
					else {
						pid_angular_rate = SIGN(pid_angular_rate) *
							fminf(rtd_limit, fabsf(pid_angular_rate));
					}
				}

				new_pid_value += pid_angular_rate;

				// Current Limiting!
				float current_limit;
				if (braking) {
					current_limit = mc_current_min * (1 + 0.6 * fabsf(torquetilt_interpolated/10));
				}
				else {
					current_limit = mc_current_max * (1 + 0.6 * fabsf(torquetilt_interpolated/10));
				}
				if (fabsf(new_pid_value) > current_limit) {
					new_pid_value = SIGN(new_pid_value) * current_limit;
				}
				else {
					// Over continuous current for more than 3 seconds? Just beep, don't actually limit currents
					if (fabsf(torquetilt_filtered_current) < max_continuous_current) {
						overcurrent_timer = current_time;
						if (current_beeping) {
							current_beeping = false;
							beep_off(false);
						}
					} else {
						if (ST2S(current_time - overcurrent_timer) > 3) {
							beep_on(true);
							current_beeping = true;
						}
					}
				}

				if (traction_control) {
					// freewheel while traction loss is detected
					pid_value = 0;
				}
				else {
					if (braking && (fabsf(pid_value - new_pid_value) > pid_brake_increment)) {
						if (new_pid_value > pid_value) {
							pid_value += pid_brake_increment;
						}
						else {
							pid_value -= pid_brake_increment;
						}
					}
					else {
						pid_value = pid_value * 0.8 + new_pid_value * 0.2;
					}
				}

				// Output to motor
				set_current(pid_value);
				break;
			case (FAULT_ANGLE_PITCH):
			case (FAULT_ANGLE_ROLL):
			case (FAULT_REVERSE):
			case (FAULT_QUICKSTOP):
			case (FAULT_SWITCH_HALF):
			case (FAULT_SWITCH_FULL):
			case (FAULT_STARTUP):
				if (ST2S(current_time - disengage_timer) > 10) {
					// 10 seconds of grace period between flipping the board over and allowing darkride mode...
					if (is_upside_down) {
						beep_alert(1, 1);
					}
					enable_upside_down = false;
					is_upside_down = false;
				}

				check_odometer();

				// Check for valid startup position and switch state
				if(fabsf(pitch_angle) < balance_conf.startup_pitch_tolerance &&
				   (fabsf(roll_angle) < balance_conf.startup_roll_tolerance) &&
				   switch_state == ON) {
					show_revision = false;
					reset_vars();
					break;
				}
				// Ignore roll while it's upside down
				if(is_upside_down && (fabsf(pitch_angle) < balance_conf.startup_pitch_tolerance)) {
					if ((state != FAULT_REVERSE) ||
						// after a reverse fault, wait at least 1 second before allowing to re-engage
						(ST2S(current_time - disengage_timer) > 1)) {
						reset_vars();
					}
					break;
				}
				// Disable output
				brake();
				break;
			case (FAULT_DUTY):
				// We need another fault to clear duty fault.
				// Otherwise duty fault will clear itself as soon as motor pauses, then motor will spool up again.
				// Rendering this fault useless.
				check_faults(true);
				// Disable output
				brake();
				break;
		}

		update_beep_alert();

		// Delay between loops
		chThdSleep(loop_time - roundf(filtered_loop_overshoot));
	}

	// Disable output
	brake();
}

/**
 *	check_odometer: see if we need to write back the odometer during fault state
 */
void check_odometer()
{
	// Make odometer persistent if we've gone 100m or more
	if (odometer_dirty > 0) {
		if (mc_interface_get_odometer() > odometer + 500) {
			if (odometer_dirty == 1) {
				// Wait 10 seconds before writing to avoid writing if immediately continuing to ride
				odo_timer = current_time;
				odometer_dirty++;
			}
			else if (ST2S(current_time - odo_timer) > 10) {
				conf_general_store_backup_data();
				odometer = mc_interface_get_odometer();
				odometer_dirty = 0;
				beep_alert(1, 0);
			}
		}
	}
}
