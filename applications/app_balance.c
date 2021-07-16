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
#include "imu/ahrs.h"
#include "utils.h"
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"
#include "mcpwm_foc.h"
#include "buzzer.h"


#include <math.h>
#include <stdio.h>

// Can
#define MAX_CAN_AGE 0.1

// Data type (Value 5 was removed, and can be reused at a later date, but i wanted to preserve the current value's numbers for UIs)
typedef enum {
	STARTUP = 0,
	RUNNING = 1,
	RUNNING_TILTBACK_DUTY = 2,
	RUNNING_TILTBACK_HIGH_VOLTAGE = 3,
	RUNNING_TILTBACK_LOW_VOLTAGE = 4,
	FAULT_ANGLE_PITCH = 6,
	FAULT_ANGLE_ROLL = 7,
	FAULT_SWITCH_HALF = 8,
	FAULT_SWITCH_FULL = 9,
	FAULT_DUTY = 10,
	FAULT_STARTUP = 11
} BalanceState;

typedef enum {
	CENTERING = 0,
	TILTBACK_DUTY,
	TILTBACK_HV,
	TILTBACK_LV,
	TILTBACK_NONE,
	REVERSESTOP
} SetpointAdjustmentType;

typedef enum {
	OFF = 0,
	HALF,
	ON
} SwitchState;

typedef struct{
	float a0, a1, a2, b1, b2;
	float z1, z2;
} Biquad;

typedef enum {
	BQ_LOWPASS,
	BQ_HIGHPASS
} BiquadType;

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
static bool allow_high_speed_full_switch_faults;

// Feature: Reverse Stop
static float reverse_stop_step_size, reverse_tolerance, reverse_total_erpm;
static systime_t reverse_timer;
static bool use_reverse_stop;

// Feature: Soft Start
#define SOFTSTART_GRACE_PERIOD_MS 100
static systime_t softstart_timer;
static bool use_soft_start;

// Feature: Adaptive Torque Response
static float acceleration, last_erpm, shedfactor;
static float grunt_factor, grunt_filtered, grunt_aggregate, grunt_threshold, atr_intensity;
static float torquetilt_target;
static int erpm_sign;

// Feature: Turntilt
static float last_yaw_angle, yaw_angle, abs_yaw_change, yaw_change, yaw_aggregate;
static float tuntilt_boost_per_erpm, yaw_aggregate_target;

// Runtime values read from elsewhere
static float pitch_angle, last_pitch_angle, roll_angle, abs_roll_angle, abs_roll_angle_sin;
static float gyro[3];
static float duty_cycle, abs_duty_cycle;
static float erpm, abs_erpm;
static float motor_current;
static float motor_position;
static float adc1, adc2;
static SwitchState switch_state;

// Runtime state values
static BalanceState state;
int log_balance_state; // not static so we can log it

static float proportional, integral, derivative;
static float last_proportional, abs_proportional;
static float pid_value;
static float setpoint, setpoint_target, setpoint_target_interpolated;
static float noseangling_interpolated;
static float torquetilt_filtered_current, torquetilt_interpolated;
static Biquad torquetilt_current_biquad, accel_biquad1, accel_biquad2;
static float turntilt_target, turntilt_interpolated;
static SetpointAdjustmentType setpointAdjustmentType;
static systime_t current_time, last_time, diff_time, loop_overshoot;
static float filtered_loop_overshoot, loop_overshoot_alpha, filtered_diff_time;
static systime_t fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer, fault_switch_half_timer, fault_duty_timer;
static float kp, ki, kd, kp_acc, ki_acc, kd_acc, kp_brk, ki_brk, kd_brk;
static float d_pt1_lowpass_state, d_pt1_lowpass_k, d_pt1_highpass_state, d_pt1_highpass_k;
static Biquad d_biquad_lowpass, d_biquad_highpass;
static float motor_timeout;
static systime_t brake_timeout;

// Debug values
static int debug_render_1, debug_render_2;
static int debug_sample_field, debug_sample_count, debug_sample_index;
static int debug_experiment_1, debug_experiment_2, debug_experiment_3, debug_experiment_4, debug_experiment_5, debug_experiment_6;

// Log values
float balance_integral, balance_setpoint, balance_atr, balance_carve, balance_ki;

// Function Prototypes
static void set_current(float current);
static void terminal_render(int argc, const char **argv);
static void terminal_sample(int argc, const char **argv);
static void terminal_experiment(int argc, const char **argv);
static float app_balance_get_debug(int index);
static void app_balance_sample_debug(void);
static void app_balance_experiment(void);

// Utility Functions
float biquad_process(Biquad *biquad, float in) {
    float out = in * biquad->a0 + biquad->z1;
    biquad->z1 = in * biquad->a1 + biquad->z2 - biquad->b1 * out;
    biquad->z2 = in * biquad->a2 - biquad->b2 * out;
    return out;
}
void biquad_config(Biquad *biquad, BiquadType type, float Fc) {
	float K = tanf(M_PI * Fc);	// -0.0159;
	float Q = 0.707; // maximum sharpness (0.5 = maximum smoothness)
	float norm = 1 / (1 + K / Q + K * K);
	if (type == BQ_LOWPASS) {
		biquad->a0 = K * K * norm;
		biquad->a1 = 2 * biquad->a0;
		biquad->a2 = biquad->a0;
	}
	else if (type == BQ_HIGHPASS) {
		biquad->a0 = 1 * norm;
		biquad->a1 = -2 * biquad->a0;
		biquad->a2 = biquad->a0;
	}
	biquad->b1 = 2 * (K * K - 1) * norm;
	biquad->b2 = (1 - K / Q + K * K) * norm;
}
void biquad_reset(Biquad *biquad) {
	biquad->z1 = 0;
	biquad->z2 = 0;
}

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

	// Feature: Reverse Stop (ON if startup_speed ends in .1)
	reverse_stop_step_size = 100.0 / balance_conf.hertz;
	float startup_speed = balance_conf.startup_speed;
	int ss = (int) startup_speed;
	float ss_rest = startup_speed - ss;
	if ((ss_rest > 0.09) && (ss_rest < 0.11)) {
		use_reverse_stop = true;
		reverse_tolerance = 50000;
	}
	else
		use_reverse_stop = false;

	// Feature: Soft Start
	use_soft_start = (balance_conf.startup_speed < 10);

	// if the full switch delay ends in 1, we don't allow high speed full switch faults
	int fullswitch_delay = balance_conf.fault_delay_switch_full / 10;
	int delay_rest = balance_conf.fault_delay_switch_full - (fullswitch_delay * 10);
	allow_high_speed_full_switch_faults = (delay_rest != 1);

	// Feature: ATR
	shedfactor = balance_conf.yaw_kp;
	// guardrails:
	if (shedfactor > 1)
		shedfactor = 0.99;
	if (shedfactor < 0.5)
		shedfactor = 0.98;

	// Feature: Turntilt
	yaw_aggregate_target = balance_conf.yaw_ki;			// borrow yaw_ki for aggregate yaw-change target
	tuntilt_boost_per_erpm = (float)balance_conf.turntilt_erpm_boost / 100.0 / (float)balance_conf.turntilt_erpm_boost_end;

	// minimum aggregate grunt to start torque tilt
	grunt_threshold = balance_conf.yaw_kd;
	if (grunt_threshold < 10)
		grunt_threshold = 50;

	// Guardrails for Onewheel PIDs (outlandish PIDs can break your motor!)
	kp_acc = fminf(balance_conf.kp, 10);
	ki_acc = fminf(balance_conf.ki, 0.01);
	kd_acc = fminf(balance_conf.kd, 1500);
	// Disable asymmetric PIDs for now to be safe
	kp_brk = kp_acc;
	ki_brk = ki_acc;
	kd_brk = kd_acc;

	// Init Filters
	if(balance_conf.loop_time_filter > 0){
		loop_overshoot_alpha = 2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter/(2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter+1);
	}
	if(balance_conf.kd_pt1_lowpass_frequency > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.kd_pt1_lowpass_frequency);
		d_pt1_lowpass_k =  dT / (RC + dT);
	}
	if(balance_conf.kd_pt1_highpass_frequency > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.kd_pt1_highpass_frequency);
		d_pt1_highpass_k =  dT / (RC + dT);
	}
	if(balance_conf.kd_biquad_lowpass > 0){
		float Fc = balance_conf.kd_biquad_lowpass / balance_conf.hertz;
		biquad_config(&d_biquad_lowpass, BQ_LOWPASS, Fc);
	}
	if(balance_conf.kd_biquad_highpass > 0){
		float Fc = balance_conf.kd_biquad_highpass / balance_conf.hertz;
		biquad_config(&d_biquad_highpass, BQ_HIGHPASS, Fc);
	}
	if(balance_conf.torquetilt_filter > 0){ // Torquetilt Current Biquad
		float Fc = balance_conf.torquetilt_filter / balance_conf.hertz;
		biquad_config(&torquetilt_current_biquad, BQ_LOWPASS, Fc);
	}

	// Feature: ATR
	// Borrow "Roll-Steer KP" value to control the acceleration biquad low-pass filter:
	float cutoff_freq = balance_conf.roll_steer_kp;
	if (cutoff_freq < 10)
		cutoff_freq = 10;
	if (cutoff_freq > 100)
		cutoff_freq = 100;
	biquad_config(&accel_biquad1, BQ_LOWPASS, cutoff_freq / ((float)balance_conf.hertz));
	// 2nd biquad filter @50Hz (don't ask me why this works better than a single filter)
	biquad_config(&accel_biquad2, BQ_LOWPASS, 50 / ((float)balance_conf.hertz));

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

	if (mc_interface_get_configuration()->m_invert_direction)
		erpm_sign = -1;
	else
		erpm_sign = 1;
}

void app_balance_start(void) {
	// First start only, override state to startup
	state = STARTUP;
	log_balance_state = state;
	// Register terminal commands
	terminal_register_command_callback(
		"app_balance_render",
		"Render debug values on the balance real time data graph",
		"[Field Number] [Plot (Optional 1 or 2)]",
		terminal_render);
	terminal_register_command_callback(
		"app_balance_sample",
		"Output real time values to the terminal",
		"[Field Number] [Sample Count]",
		terminal_sample);
	terminal_register_command_callback(
		"app_balance_experiment",
		"Output real time values to the experiments graph",
		"[Field Number] [Plot 1-6]",
		terminal_experiment);
	// Start the balance thread
	app_thread = chThdCreateStatic(balance_thread_wa, sizeof(balance_thread_wa), NORMALPRIO, balance_thread, NULL);
}

void app_balance_stop(void) {
	if(app_thread != NULL){
		chThdTerminate(app_thread);
		chThdWait(app_thread);
	}
	set_current(0);
	terminal_unregister_callback(terminal_render);
	terminal_unregister_callback(terminal_sample);
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
	return app_balance_get_debug(debug_render_1);
}
float app_balance_get_debug2(void) {
	return app_balance_get_debug(debug_render_2);
}

// Internal Functions
static void reset_vars(void){
	// Clear accumulated values.
	integral = 0;
	last_proportional = 0;
	d_pt1_lowpass_state = 0;
	d_pt1_highpass_state = 0;
	biquad_reset(&d_biquad_lowpass);
	biquad_reset(&d_biquad_highpass);
	// Set values for startup
	setpoint = pitch_angle;
	setpoint_target_interpolated = pitch_angle;
	setpoint_target = 0;
	noseangling_interpolated = 0;
	torquetilt_interpolated = 0;
	torquetilt_filtered_current = 0;
	biquad_reset(&torquetilt_current_biquad);
	turntilt_target = 0;
	turntilt_interpolated = 0;
	setpointAdjustmentType = CENTERING;
	state = RUNNING;
	current_time = 0;
	last_time = 0;
	diff_time = 0;
	brake_timeout = 0;

	// ATR:
	biquad_reset(&accel_biquad1);
	biquad_reset(&accel_biquad2);
	grunt_aggregate = 0;
	grunt_filtered = 0;
	pid_value = 0;

	if (use_soft_start) {
		// Soft start
		// minimum values (even 0,0,0 is possible) for soft start:
		kp = 1;
		ki = 0;
		kd = 100;
	}
	else {
		// Normal start / quick-start
		kp = kp_acc / 2;
		ki = ki_acc / 2;
		kd = kd_acc / 2;
	}
}

static float get_setpoint_adjustment_step_size(void){
	switch(setpointAdjustmentType){
		case (CENTERING):
			return startup_step_size;
		case (TILTBACK_DUTY):
			return tiltback_duty_step_size;
		case (TILTBACK_HV):
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
			sw_state = HALF;
		}else{
			sw_state = OFF;
		}
	}

	/*
	 * Use external buzzer to notify rider of foot switch faults at speed
	 */
	if (sw_state == OFF) {
		if ((abs_erpm > balance_conf.fault_adc_half_erpm)
			&& (state >= RUNNING)
			&& (state <= RUNNING_TILTBACK_LOW_VOLTAGE))
		{
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
	return sw_state;
}

// Fault checking order does not really matter. From a UX perspective, switch should be before angle.
static bool check_faults(bool ignoreTimers){
	// Check switch
	// Switch fully open
	if(switch_state == OFF){
		if(ST2MS(current_time - fault_switch_timer) > balance_conf.fault_delay_switch_full || ignoreTimers){
			state = FAULT_SWITCH_FULL;
			return true;
		}
		else if ((abs_erpm < balance_conf.fault_adc_half_erpm) && (fabsf(pitch_angle) > 15)) {
			// QUICK STOP
			state = FAULT_SWITCH_FULL;
			return true;
		}
		else if ((abs_erpm > 3000) && !allow_high_speed_full_switch_faults) {
			// above 3k erpm (~7mph on a 11 inch onewheel tire) don't ever produce switch faults!
			fault_switch_timer = current_time;
		}
	} else {
		fault_switch_timer = current_time;
	}

	// Feature: Reverse-Stop
	if(setpointAdjustmentType == REVERSESTOP){
		//  Taking your foot off entirely while reversing? Ignore delays
		if ((fabsf(pitch_angle) > 15) || (switch_state == OFF)) {
			state = FAULT_SWITCH_FULL;
			return true;
		}
		// Above 10 degrees for a half a second? Switch it off
		if ((fabsf(pitch_angle) > 10) && (ST2MS(current_time - reverse_timer) > 500)) {
			state = FAULT_SWITCH_FULL;
			return true;
		}
		// Above 5 degrees for a full second? Switch it off
		if ((fabsf(pitch_angle) > 5) && (ST2MS(current_time - reverse_timer) > 1000)) {
			state = FAULT_SWITCH_FULL;
			return true;
		}
		if (fabsf(pitch_angle) < 5) {
			reverse_timer = current_time;
		}
	}

	// Switch partially open and stopped
	if((switch_state == HALF || switch_state == OFF) && abs_erpm < balance_conf.fault_adc_half_erpm){
		if(ST2MS(current_time - fault_switch_half_timer) > balance_conf.fault_delay_switch_half || ignoreTimers){
			state = FAULT_SWITCH_HALF;
			return true;
		}
	} else {
		fault_switch_half_timer = current_time;
	}

	// Check pitch angle
	if(fabsf(pitch_angle) > balance_conf.fault_pitch){
		if(ST2MS(current_time - fault_angle_pitch_timer) > balance_conf.fault_delay_pitch || ignoreTimers){
			state = FAULT_ANGLE_PITCH;
			return true;
		}
	}else{
		fault_angle_pitch_timer = current_time;
	}

	// Check roll angle
	if(fabsf(roll_angle) > balance_conf.fault_roll){
		if(ST2MS(current_time - fault_angle_roll_timer) > balance_conf.fault_delay_roll || ignoreTimers){
			state = FAULT_ANGLE_ROLL;
			return true;
		}
	}else{
		fault_angle_roll_timer = current_time;
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
	if(setpointAdjustmentType == CENTERING) {
		if (setpoint_target_interpolated != setpoint_target){
			// Ignore tiltback during centering sequence
			state = RUNNING;
			softstart_timer = current_time;
		}
		else if (ST2MS(current_time - softstart_timer) > SOFTSTART_GRACE_PERIOD_MS){
			// After a short delay transition to normal riding
			setpointAdjustmentType = TILTBACK_NONE;
		}
		else if (use_soft_start == false){
			setpointAdjustmentType = TILTBACK_NONE;
		}
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
	}else if(abs_duty_cycle > balance_conf.tiltback_duty){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_duty_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_duty_angle;
		}
		setpointAdjustmentType = TILTBACK_DUTY;
		state = RUNNING_TILTBACK_DUTY;
	}else if(abs_duty_cycle > 0.05 && GET_INPUT_VOLTAGE() > balance_conf.tiltback_hv){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_hv_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_hv_angle;
		}
		setpointAdjustmentType = TILTBACK_HV;
		state = RUNNING_TILTBACK_HIGH_VOLTAGE;
		beep_alert(3, 0);	// Triple-beep
	}else if(abs_duty_cycle > 0.05 && GET_INPUT_VOLTAGE() < balance_conf.tiltback_lv){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_lv_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_lv_angle;
		}
		setpointAdjustmentType = TILTBACK_LV;
		state = RUNNING_TILTBACK_LOW_VOLTAGE;
		beep_alert(3, 0);	// Triple-beep
	}else{
		// Normal running
		if (use_reverse_stop && (erpm < 0)) {
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
	setpoint += noseangling_interpolated;
}

static void apply_torquetilt(void){
	// Skip torque tilt logic if start current is 0
	if (balance_conf.torquetilt_start_current == 0)
		return;

	// Filter current (Biquad)
	if(balance_conf.torquetilt_filter > 0){
		torquetilt_filtered_current = biquad_process(&torquetilt_current_biquad, motor_current);
	}else{
		torquetilt_filtered_current  = motor_current;
	}

	if (SIGN(torquetilt_filtered_current) != SIGN(erpm)) {
		// current is negative, so we are braking or going downhill
		//start_current = torquetilt_brk_start_current;
	}

	// grunt is a measure of how much we're struggling to produce acceleration
	float accel = acceleration;
	// acceleration opposite to current? produce high grunt by treating it as near zero acceleration
	if (SIGN(accel) != SIGN(torquetilt_filtered_current))
		accel = SIGN(torquetilt_filtered_current) * 0.05;
	// grunt factor is always positive!
	grunt_factor = fminf(torquetilt_filtered_current / (accel * 50.0), 20);
	grunt_filtered = grunt_filtered * 0.8 + grunt_factor * 0.2;
	if (grunt_filtered > 1)
		grunt_aggregate += grunt_filtered;
	else
		grunt_aggregate /= 2;

	if (grunt_aggregate > grunt_threshold) {
		/*float*/ atr_intensity = 1;	// this can be a local variable ultimately
		if (grunt_filtered < 5)
			atr_intensity = (grunt_filtered + 3) / 8;

		// Take abs motor current, subtract start offset, and take the max of that with 0 to get the current above our start threshold (absolute).
		// Then multiply it by "power" to get our desired angle, and min with the limit to respect boundaries.
		// Finally multiply it by sign motor current to get directionality back
		torquetilt_target = fabsf(torquetilt_filtered_current) * balance_conf.torquetilt_strength;
		torquetilt_target = fminf(torquetilt_target, balance_conf.torquetilt_angle_limit);
		torquetilt_target *= SIGN(torquetilt_filtered_current);
	}
	else {
		// If the aggregate grunt is below the ratio then we must be just accelerating. No TT here!!
		torquetilt_target = 0;
	}
	//ttt = torquetilt_target;

	// don't get the board too "excited", don't let the nose rise much above 0 ;)
	float max_tilt = 0;//balance_conf.torquetilt_angle_limit / 4;
	bool nose_is_up = (SIGN(last_proportional - torquetilt_interpolated) != SIGN(torquetilt_target));
	float actual_tilt = fabsf(last_proportional - torquetilt_interpolated);
	if (nose_is_up && (fabsf(torquetilt_target) > 0) && (actual_tilt > max_tilt)) {
		torquetilt_target = torquetilt_target - SIGN(torquetilt_target) * actual_tilt * 0.6;
	}

	// Deal with integral windup
	if (SIGN(integral) != SIGN(erpm)) { // integral windup after braking
		if ((torquetilt_target == 0)  && (fabsf(integral) > 2000) && (fabsf(pid_value) < 2)) {
			// we are back to 0 ttt, current is small, yet integral windup is high:
			// resort to brute force integral windup mitigation, shed 1% each cycle:
			// at any speeds
			integral = integral * shedfactor;
		}
	}
	else {
		// integral windup after acceleration
		// we usually don't want to mess with integral windup after accelerating - that's what
		// helps give the onewheel its soft brake feel!
		// there's a few exceptions though:
		// a) when slowly crossing an obstacle windup will become extremely high while speed is very low
		// b) when going up a hill instantly followed by a steep decline - integral windup
		//    will cause a delayed braking response, resulting in a taildrag

		if ((torquetilt_target == 0)  && (fabsf(integral) > 2000)) {
			// This here is for (a) - once pitch crosses to zero at low erpms this should be safe to do!
			if ((SIGN(pitch_angle) == SIGN(erpm)) && (fabsf(erpm) < 1000)) {
				// we are back to 0 ttt, current is small, yet integral windup is high:
				// resort to brute force integral windup mitigation, shed 1% each cycle:
				// but only at low speeds (below 4mph)
				integral *= shedfactor;
				torquetilt_interpolated *= shedfactor;
			}
			// This here is for (b)
			// ???
		}
	}

	float step_size;
	if(((torquetilt_target >= 0) && (torquetilt_interpolated - torquetilt_target > 0)) ||
	   ((torquetilt_target <= 0) && (torquetilt_interpolated - torquetilt_target < 0))){
		step_size = torquetilt_off_step_size;
	}else{
		step_size = torquetilt_on_step_size;
	}

	if(fabsf(torquetilt_target - torquetilt_interpolated) < step_size){
		torquetilt_interpolated = torquetilt_target;
	}else if (torquetilt_target - torquetilt_interpolated > 0){
		torquetilt_interpolated += step_size;
	}else{
		torquetilt_interpolated -= step_size;
	}
	setpoint += torquetilt_interpolated;
}

static void apply_turntilt(void){
	// Apply cutzone
	if(abs_yaw_change * 100 < balance_conf.turntilt_start_angle){
		turntilt_target = 0;
	}
	else {
		// Calculate desired angle
		turntilt_target = abs_yaw_change * balance_conf.turntilt_strength;
		//turntilt_target = abs_roll_angle_sin * balance_conf.turntilt_strength;

		// Apply speed scaling
		float boost;
		if(abs_erpm < balance_conf.turntilt_erpm_boost_end){
			boost = 1.0 + abs_erpm * tuntilt_boost_per_erpm;
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
			if (atr_scaling < 0)
				atr_scaling = 0;
			turntilt_target *= atr_scaling;
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

		// Get the values we want
		last_pitch_angle = pitch_angle;
		pitch_angle = RAD2DEG_f(imu_get_pitch());
		roll_angle = RAD2DEG_f(imu_get_roll());
		abs_roll_angle = fabsf(roll_angle);
		abs_roll_angle_sin = sinf(DEG2RAD_f(abs_roll_angle));
		imu_get_gyro(gyro);
		duty_cycle = mc_interface_get_duty_cycle_now();
		abs_duty_cycle = fabsf(duty_cycle);
		erpm = mc_interface_get_rpm();
		abs_erpm = fabsf(erpm);

		// Turn tilt:
		yaw_angle = imu_get_yaw() * 180.0f / M_PI;
		float new_change = yaw_angle - last_yaw_angle;
		last_yaw_angle = yaw_angle;
		if (fabsf(new_change) > 100)				// yaw flips signs at 180, ignore those changes
			new_change = yaw_change * 0.95;
		// To avoid overreactions at low speed, limit change here:
		new_change = fminf(new_change, 0.12);
		new_change = fmaxf(new_change, -0.12);
		yaw_change = yaw_change * 0.95 + 0.05 * (new_change);
		// Clear the aggregate yaw whenever we change direction
		if (SIGN(yaw_change) != SIGN(yaw_aggregate))
			yaw_aggregate = 0;
		abs_yaw_change = fabsf(yaw_change);
		if (abs_yaw_change > 0.03)				// don't count tiny yaw changes towards aggregate
			yaw_aggregate += yaw_change;

		float smooth_erpm = erpm_sign * mcpwm_foc_get_smooth_erpm();
		acceleration = biquad_process(&accel_biquad1, smooth_erpm - last_erpm);
		acceleration = biquad_process(&accel_biquad2, acceleration);
		last_erpm = smooth_erpm;

		accelavg += (acceleration_raw - accelhist[accelidx]) / ACCEL_ARRAY_SIZE;
		accelhist[accelidx] = acceleration_raw;
		accelidx++;
		if (accelidx == ACCEL_ARRAY_SIZE)
			accelidx = 0;

		acceleration = accelavg;

		switch_state = check_adcs();

		// Control Loop State Logic
		switch(state){
			case (STARTUP):
				// Disable output
				brake();
				if(imu_startup_done()){
					reset_vars();
					state = FAULT_STARTUP; // Trigger a fault so we need to meet start conditions to start

#ifdef HAS_EXT_BUZZER
					// Let the rider know that the board is ready
					beep_on(true);
					chThdSleepMilliseconds(100);
					beep_off(true);

					// Are we within 5V of the LV tiltback threshold? Issue 1 beep for each volt below that
					double bat_volts = GET_INPUT_VOLTAGE();
					double threshold = balance_conf.tiltback_lv + 5;
					if (bat_volts < threshold) {
						chThdSleepMilliseconds(400);
						while (bat_volts < threshold) {
							chThdSleepMilliseconds(200);
							beep_on(1);
							chThdSleepMilliseconds(300);
							beep_off(1);
							threshold -= 1;
						}
					}
#endif
				}
				break;
			case (RUNNING):
			case (RUNNING_TILTBACK_DUTY):
			case (RUNNING_TILTBACK_HIGH_VOLTAGE):
			case (RUNNING_TILTBACK_LOW_VOLTAGE):
				log_balance_state = state + 100 * setpointAdjustmentType;

				// Check for faults
				if(check_faults(false)){
					break;
				}

				// Calculate setpoint and interpolation
				calculate_setpoint_target();
				calculate_setpoint_interpolated();
				setpoint = setpoint_target_interpolated;
				if (setpointAdjustmentType == TILTBACK) {
					apply_noseangling();
					apply_torquetilt();
					apply_turntilt();
				}

				// Do PID maths
				proportional = setpoint - pitch_angle;
				integral = integral + proportional;
				derivative = last_pitch_angle - pitch_angle;

				// Apply D term filters
				if(balance_conf.kd_pt1_lowpass_frequency > 0){
					d_pt1_lowpass_state = d_pt1_lowpass_state + d_pt1_lowpass_k * (derivative - d_pt1_lowpass_state);
					derivative = d_pt1_lowpass_state;
				}
				if(balance_conf.kd_pt1_highpass_frequency > 0){
					d_pt1_highpass_state = d_pt1_highpass_state + d_pt1_highpass_k * (derivative - d_pt1_highpass_state);
					derivative = derivative - d_pt1_highpass_state;
				}
				if(balance_conf.kd_biquad_lowpass > 0){
					derivative = biquad_process(&d_biquad_lowpass, derivative);
				}
				if(balance_conf.kd_biquad_highpass > 0){
					derivative = biquad_process(&d_biquad_highpass, derivative);
				}

				// Switch between breaking PIDs and acceleration PIDs
				float kp_target, ki_target, kd_target;
				if (SIGN(pid_value) == SIGN(erpm)) {
					// braking
					kp_target = kp_brk;
					ki_target = ki_brk;
					kd_target = kd_brk;
				}
				else {
					// acceleration
					kp_target = kp_acc;
					ki_target = ki_acc;
					kd_target = kd_acc;
				}
				if (setpointAdjustmentType == REVERSESTOP) {
					kp_target = 2;
					integral = 0;
				}

				// Use filtering to avoid sudden changes in PID values
				kp = kp * 0.997 + kp_target * 0.003;
				ki = ki * 0.997 + ki_target * 0.003;
				kd = kd * 0.997 + kd_target * 0.003;

				if (use_soft_start && (setpointAdjustmentType == CENTERING)) {
					// soft-start
					float pid_target = (kp * proportional) + (kd * derivative);
					pid_value = 0.05 * pid_target + 0.95 * pid_value;
					// once centering is done, start integral component from 0
					integral = 0;
					ki = 0;
				}
				else {
					pid_value = (kp * proportional) + (ki * integral) + (kd * derivative);
				}

				last_proportional = proportional;

				// Apply Booster
				if (setpointAdjustmentType == TILTBACK) {
					abs_proportional = fabsf(proportional);
					if(abs_proportional > balance_conf.booster_angle){
						if(abs_proportional - balance_conf.booster_angle < balance_conf.booster_ramp){
							pid_value += (balance_conf.booster_current * SIGN(proportional)) * ((abs_proportional - balance_conf.booster_angle) / balance_conf.booster_ramp);
						}else{
							pid_value += balance_conf.booster_current * SIGN(proportional);
						}
					}
				}

				// For logging only:
				balance_integral = integral;
				balance_ki = ki;
				balance_setpoint = setpoint;
				balance_atr = torquetilt_target;
				balance_carve = turntilt_target;

				// Output to motor
				set_current(pid_value);
				break;
			case (FAULT_ANGLE_PITCH):
			case (FAULT_ANGLE_ROLL):
			case (FAULT_SWITCH_HALF):
			case (FAULT_SWITCH_FULL):
			case (FAULT_STARTUP):
				if (log_balance_state != FAULT_DUTY)
					log_balance_state = state;

				// Check for valid startup position and switch state
				if(fabsf(pitch_angle) < balance_conf.startup_pitch_tolerance && fabsf(roll_angle) < balance_conf.startup_roll_tolerance && switch_state == ON){
					reset_vars();
					break;
				}
				// Disable output
				brake();
				break;
			case (FAULT_DUTY):
				log_balance_state = FAULT_DUTY;
				// We need another fault to clear duty fault.
				// Otherwise duty fault will clear itself as soon as motor pauses, then motor will spool up again.
				// Rendering this fault useless.
				check_faults(true);
				// Disable output
				brake();
				break;
		}
		update_beep_alert();

		// Debug outputs
		app_balance_sample_debug();
		app_balance_experiment();

		// Delay between loops
		chThdSleep(loop_time - roundf(filtered_loop_overshoot));
	}
	// in case we leave this force the buzzer off (force=regardless of ongoing multi beeps)
	beep_off(true);

	// Disable output
	brake();
}

// Terminal commands
static void terminal_render(int argc, const char **argv) {
	if (argc == 2 || argc == 3) {
		int field = 0;
		int graph = 1;
		sscanf(argv[1], "%d", &field);
		if(argc == 3){
			sscanf(argv[2], "%d", &graph);
			if(graph < 1 || graph > 2){
				graph = 1;
			}
		}
		if(graph == 1){
			debug_render_1 = field;
		}else{
			debug_render_2 = field;
		}
	} else {
		commands_printf("This command requires one or two argument(s).\n");
	}
}

static void terminal_sample(int argc, const char **argv) {
	if (argc == 3) {
		debug_sample_field = 0;
		debug_sample_count = 0;
		sscanf(argv[1], "%d", &debug_sample_field);
		sscanf(argv[2], "%d", &debug_sample_count);
		debug_sample_index = 0;
	} else {
		commands_printf("This command requires two arguments.\n");
	}
}

static void terminal_experiment(int argc, const char **argv) {
	if (argc == 3) {
		int field = 0;
		int graph = 1;
		sscanf(argv[1], "%d", &field);
		sscanf(argv[2], "%d", &graph);
		switch(graph){
			case (1):
				debug_experiment_1 = field;
				break;
			case (2):
				debug_experiment_2 = field;
				break;
			case (3):
				debug_experiment_3 = field;
				break;
			case (4):
				debug_experiment_4 = field;
				break;
			case (5):
				debug_experiment_5 = field;
				break;
			case (6):
				debug_experiment_6 = field;
				break;
		}
		commands_init_plot("Microseconds", "Balance App Debug Data");
		commands_plot_add_graph("1");
		commands_plot_add_graph("2");
		commands_plot_add_graph("3");
		commands_plot_add_graph("4");
		commands_plot_add_graph("5");
		commands_plot_add_graph("6");
	} else {
		commands_printf("This command requires two arguments.\n");
	}
}

// Debug functions
static float app_balance_get_debug(int index){
	switch(index){
		case(1):
			return motor_position;
		case(2):
			return setpoint;
		case(3):
			return torquetilt_filtered_current;
		case(4):
			return derivative;
		case(5):
			return last_pitch_angle - pitch_angle;
		case(6):
			return motor_current;
		case(7):
			return erpm;
		case(8):
			return abs_erpm;
		case(9):
			return loop_time;
		case(10):
			return diff_time;
		case(11):
			return loop_overshoot;
		case(12):
			return filtered_loop_overshoot;
		case(13):
			return filtered_diff_time;
		default:
			return 0;
	}
}
static void app_balance_sample_debug(){
	if(debug_sample_index < debug_sample_count){
		commands_printf("%f", (double)app_balance_get_debug(debug_sample_field));
		debug_sample_index += 1;
	}
}
static void app_balance_experiment(){
	if(debug_experiment_1 != 0){
		commands_plot_set_graph(0);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_1));
	}
	if(debug_experiment_2 != 0){
		commands_plot_set_graph(1);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_2));
	}
	if(debug_experiment_3 != 0){
		commands_plot_set_graph(2);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_3));
	}
	if(debug_experiment_4 != 0){
		commands_plot_set_graph(3);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_4));
	}
	if(debug_experiment_5 != 0){
		commands_plot_set_graph(4);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_5));
	}
	if(debug_experiment_6 != 0){
		commands_plot_set_graph(5);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_6));
	}
}
