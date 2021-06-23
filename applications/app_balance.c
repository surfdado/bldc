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

#include "app.h"
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
#include "buzzer.h"


#include <math.h>

// Can
#define MAX_CAN_AGE 0.1

// Data type
typedef enum {
	STARTUP = 0,
	RUNNING,
	RUNNING_TILTBACK_DUTY,
	RUNNING_TILTBACK_HIGH_VOLTAGE,
	RUNNING_TILTBACK_LOW_VOLTAGE,
	RUNNING_TILTBACK_CONSTANT,
	FAULT_ANGLE_PITCH,
	FAULT_ANGLE_ROLL,
	FAULT_SWITCH_HALF,
	FAULT_SWITCH_FULL,
	FAULT_DUTY,
	FAULT_STARTUP
} BalanceState;


typedef enum {
	BRAKE_NORMAL,
	BRAKE_DOWNHILL
} BrakeMode;

typedef enum {
	CENTERING = 0,
	TILTBACK,
	REVERSESTOP
} SetpointAdjustmentType;

typedef enum {
	OFF = 0,
	HALF,
	ON
} SwitchState;

typedef enum {
	RIDE_OFF = 0,
	RIDE_IDLE = 1,
	RIDE_FORWARD,
	RIDE_REVERSE,
	BRAKE_FORWARD,
	BRAKE_REVERSE
} RideState;

//float OneKSamples1[100];
//float OneKSamples2[100];
//static int sampleIdx;

// Allow me to go 10km/h even below low voltage
#define TILTBACK_LOW_VOLTAGE_MIN_ERPM 3000
// Give me another 2 Volts of margin at low speeds before doing tiltback there too
#define TILTBACK_LOW_VOLTAGE_SLOW_MARGIN 2
// Audible alert at 1 Volt above tiltback voltage
#define HEADSUP_LOW_VOLTAGE_MARGIN 1
// Softstart grace period
#define SOFTSTART_GRACE_PERIOD_MS 100

static int low_voltage_headsup_done = 0;

// Balance thread
static THD_FUNCTION(balance_thread, arg);
static THD_WORKING_AREA(balance_thread_wa, 2048); // 2kb stack for this thread

static thread_t *app_thread;

// Config values
static volatile balance_config balance_conf;
static volatile imu_config imu_conf;
static float startup_step_size, tiltback_step_size, torquetilt_step_size, torquetilt_step_size_down, turntilt_step_size, reverse_stop_step_size, reverse_tolerance;

// Runtime values read from elsewhere
static float pitch_angle, last_pitch_angle, roll_angle, abs_roll_angle, abs_roll_angle_sin;
static float gyro[3];
static float duty_cycle, abs_duty_cycle;
static float erpm, abs_erpm;
static float motor_current;
static float motor_position;
static float adc1, adc2;
static SwitchState switch_state;

// Rumtime state values
static BalanceState state;
int log_balance_state;	// not static so we can log it
static float proportional, integral, derivative;
static float last_proportional, abs_proportional;
static float pid_value;
static float setpoint, setpoint_target, setpoint_target_interpolated;
static float tiltback_constant, tiltback_erpmbased, tiltback_constant_erpm;
static float torquetilt_brk_start_current, torquetilt_brk_delay;
static float torquetilt_filtered_current, torquetilt_interpolated;
static float turntilt_target, turntilt_interpolated;
static int booster_beep = 0;
static int booster_beeping = 0;
static SetpointAdjustmentType setpointAdjustmentType;
static systime_t current_time, last_time, diff_time;
static systime_t fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer, fault_switch_half_timer, fault_duty_timer, softstart_timer;
static float d_pt1_state, d_pt1_k;
static float max_temp_fet;
static bool use_soft_start;
static bool use_reverse_stop;
static bool allow_high_speed_full_switch_faults;
static bool disable_all_5_3_features;
static float reverse_total_erpm;
static RideState ride_state, new_ride_state;
static float kp, ki, kd, kp_acc, ki_acc, kd_acc, kp_brk, ki_brk, kd_brk;
static float autosuspend_timer, autosuspend_timeout;
static float acceleration, last_erpm;
static float integral_max, integral_min, shedfactor;

float expacc, expki, expkd, expkp, expprop, expsetpoint, ttt;
float exp_grunt_factor, exp_g_max, exp_g_min;

static float bq_z1, bq_z2;
static float bq_a0, bq_a1, bq_a2, bq_b1, bq_b2;
static inline float biquad_filter(float in) {
    float out = in * bq_a0 + bq_z1;
    bq_z1 = in * bq_a1 + bq_z2 - bq_b1 * out;
    bq_z2 = in * bq_a2 - bq_b2 * out;
    return out;
}
static void biquad_config(float Fc) {
	float K = tanf(M_PI * Fc);	// -0.0159;
	float Q = 0.707; // maximum sharpness (0.5 = maximum smoothness)
	float norm = 1 / (1 + K / Q + K * K);
	bq_a0 = K * K * norm;
	bq_a1 = 2 * bq_a0;
	bq_a2 = bq_a0;
	bq_b1 = 2 * (K * K - 1) * norm;
	bq_b2 = (1 - K / Q + K * K) * norm;
}

static float bq2_z1, bq2_z2;
static float bq2_a0, bq2_a1, bq2_a2, bq2_b1, bq2_b2;
static inline float biquad2_filter(float in) {
    float out = in * bq2_a0 + bq2_z1;
    bq2_z1 = in * bq2_a1 + bq2_z2 - bq2_b1 * out;
    bq2_z2 = in * bq2_a2 - bq2_b2 * out;
    return out;
}
void biquad2_config(float Fc);
void biquad2_config(float Fc) {
	float K = tanf(M_PI * Fc);	// -0.0159;
	float Q = 0.707; // maximum sharpness (0.5 = maximum smoothness)
	float norm = 1 / (1 + K / Q + K * K);
	bq2_a0 = K * K * norm;
	bq2_a1 = 2 * bq2_a0;
	bq2_a2 = bq2_a0;
	bq2_b1 = 2 * (K * K - 1) * norm;
	bq2_b2 = (1 - K / Q + K * K) * norm;
}

void app_balance_configure(balance_config *conf, imu_config *conf2) {
	balance_conf = *conf;
	imu_conf = *conf2;
	// Set calculated values from config
	if(balance_conf.kd_pt1_frequency > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.kd_pt1_frequency);
		d_pt1_k =  dT / (RC + dT);
	}
	startup_step_size = balance_conf.startup_speed / balance_conf.hertz;
	tiltback_step_size = balance_conf.tiltback_speed / balance_conf.hertz;
	torquetilt_step_size = balance_conf.torquetilt_speed / balance_conf.hertz;
	turntilt_step_size = balance_conf.turntilt_speed / balance_conf.hertz;
	reverse_stop_step_size = 100.0 / balance_conf.hertz;
	use_soft_start = (balance_conf.startup_speed < 10);

	// if the full switch delay ends in 1, we don't allow high speed full switch faults
	int fullswitch_delay = balance_conf.fault_delay_switch_full / 10;
	int delay_rest = balance_conf.fault_delay_switch_full - (fullswitch_delay * 10);
	allow_high_speed_full_switch_faults = (delay_rest != 1);

	float startup_speed = balance_conf.startup_speed;
	int ss = (int) startup_speed;
	float ss_rest = startup_speed - ss;
	if ((ss_rest > 0.09) && (ss_rest < 0.11)) {
		use_reverse_stop = true;
		reverse_tolerance = 50000;
	}
	else if ((ss_rest > 0.19) && (ss_rest < 0.21)) {
		use_reverse_stop = true;
		reverse_tolerance = 100000;
	}
	else
		use_reverse_stop = false;

	torquetilt_step_size_down = torquetilt_step_size;
	// to avoid oscillations:
	/*if (balance_conf.torquetilt_speed > 2.5)
		torquetilt_step_size_down /= 2;
	else if (balance_conf.torquetilt_speed > 1.5)
	torquetilt_step_size_down /= 1.5;*/

	float torquetilt_start_current = balance_conf.torquetilt_start_current;
	int sc = (int) torquetilt_start_current;
	float sc_rest = torquetilt_start_current - sc;
	if (sc_rest >= 0.3)
		torquetilt_brk_start_current = sc_rest * torquetilt_start_current;
	else
		torquetilt_brk_start_current = torquetilt_start_current;

	torquetilt_brk_delay = 0;

	float torquetilt_filter = balance_conf.torquetilt_filter * 100;
	int ttf = (int) torquetilt_filter;
	float ttf_rest = torquetilt_filter - ttf;
	if (ttf_rest > 0.1)
	{}//torquetilt_brk_delay = ttf_rest * 10 * 1000;	// convert to ms

	float booster_angle = balance_conf.booster_angle;
	int angl = (int) booster_angle;
	float angl_rest = booster_angle - angl;
	if (angl_rest == 0.1)
		booster_beep = 1;

	shedfactor = balance_conf.yaw_kp;
	// guardrails:
	if (shedfactor > 1)
		shedfactor = 0.98;
	if (shedfactor < 0.5)
		shedfactor = 0.95;

	// Guardrails for Onewheel PIDs (outlandish PIDs can break your motor!)
	kp_acc = fminf(balance_conf.kp, 10);
	ki_acc = fminf(balance_conf.ki, 0.01);
	kd_acc = fminf(balance_conf.kd, 1500);
	// Disable asymmetric PIDs for now to be safe
	kp_brk = kp_acc;
	ki_brk = ki_acc;
	kd_brk = kd_acc;

	// Borrow "Roll-Steer KP" value to control the acceleration biquad low-pass filter:
	// 20 works well for me, higher reduces delay but increase noise
	// 10 is a conservative default
	float cutoff_freq = balance_conf.roll_steer_kp;
	if (cutoff_freq < 10)
		cutoff_freq = 10;
	if (cutoff_freq > 30)
		cutoff_freq = 30;
	biquad_config(cutoff_freq / ((float)balance_conf.hertz));
	biquad2_config(cutoff_freq / ((float)balance_conf.hertz));

	// Limit integral buildup, hard coded for now
	if (balance_conf.roll_steer_erpm_kp >= 1) {
		integral_max = balance_conf.roll_steer_erpm_kp * 20000.0; // acceleration
		integral_min = (-1) * balance_conf.roll_steer_erpm_kp * 10000.0; // braking
	}
	else {
		integral_max = 35000.0; // acceleration
		integral_min = -20000.0; // braking
	}

	disable_all_5_3_features = (balance_conf.deadzone != 0);
	if (disable_all_5_3_features) {
		use_soft_start = false;
		use_reverse_stop = false;
	}

	tiltback_constant = 0;
	tiltback_erpmbased = 0;
	tiltback_constant_erpm = 99999;
	if (balance_conf.tiltback_constant != 0) {
		// use erpm ending in "1" to indicate speed dependent tiltback
		tiltback_constant = balance_conf.tiltback_constant;
		tiltback_constant_erpm = balance_conf.tiltback_constant_erpm;
		int tb = tiltback_constant_erpm / 10;
		int tbrest = tiltback_constant_erpm - tb * 10;
		if (tbrest == 1)
			tiltback_erpmbased = tiltback_constant / 10000;
	}

	switch (app_get_configuration()->shutdown_mode) {
	case SHUTDOWN_MODE_OFF_AFTER_10S: autosuspend_timeout = 10; break;
	case SHUTDOWN_MODE_OFF_AFTER_1M: autosuspend_timeout = 60; break;
	case SHUTDOWN_MODE_OFF_AFTER_5M: autosuspend_timeout = 60 * 5; break;
	case SHUTDOWN_MODE_OFF_AFTER_10M: autosuspend_timeout = 60 * 10; break;
	case SHUTDOWN_MODE_OFF_AFTER_30M: autosuspend_timeout = 60 * 30; break;
	case SHUTDOWN_MODE_OFF_AFTER_1H: autosuspend_timeout = 60 * 60; break;
	case SHUTDOWN_MODE_OFF_AFTER_5H: autosuspend_timeout = 60 * 60 * 5; break;
	default:
		autosuspend_timeout = 0;
	}
}

void app_balance_start(void) {
	// First start only, override state to startup
	state = STARTUP;
	// Start the balance thread
	app_thread = chThdCreateStatic(balance_thread_wa, sizeof(balance_thread_wa), NORMALPRIO, balance_thread, NULL);
}

void reset_vars(void){
	// Clear accumulated values.
	//sampleIdx = 0;
	integral = 0;
	last_proportional = 0;
	last_pitch_angle = 0;
	last_erpm = 0;
	d_pt1_state = 0;
	// Set values for startup
	setpoint = pitch_angle;
	setpoint_target_interpolated = pitch_angle;
	setpoint_target = 0;
	torquetilt_interpolated = 0;
	torquetilt_filtered_current = 0;
	turntilt_target = 0;
	turntilt_interpolated = 0;
	setpointAdjustmentType = CENTERING;
	state = RUNNING;
	current_time = 0;
	last_time = 0;
	diff_time = 0;
	max_temp_fet = mc_interface_get_configuration()->l_temp_fet_start;
	new_ride_state = ride_state = RIDE_OFF;

	if (disable_all_5_3_features) {
		kp = kp_acc;
		ki = ki_acc;
		kd = kd_acc;
	}
	else {
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

	exp_g_max = 0;
	exp_g_min = 0;
	pid_value = 0;
	// biquad filter for acceleration:
	bq_z1 = 0;
	bq_z2 = 0;
	bq2_z1 = 0;
	bq2_z2 = 0;
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
float app_balance_get_motor_position(void) {
	return motor_position;
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

float get_setpoint_adjustment_step_size(void){
	switch(setpointAdjustmentType){
		case (CENTERING):
			return startup_step_size;
		case (REVERSESTOP):
			return reverse_stop_step_size;
		case (TILTBACK):
			return tiltback_step_size;
	}
	return 0;
}

// Fault checking order does not really matter. From a UX perspective, switch should be before angle.
bool check_faults(bool ignoreTimers){
	// Check switch
	// Switch fully open
	if(switch_state == OFF){
		// any speed:
		if(ST2MS(current_time - fault_switch_timer) > balance_conf.fault_delay_switch_full || ignoreTimers){
			state = FAULT_SWITCH_FULL;
			return true;
		}
		// low speed (below 4 x half-fault threshold speed):
		else if ((abs_erpm < balance_conf.fault_adc_half_erpm * 4)
				 && (ST2MS(current_time - fault_switch_timer) > balance_conf.fault_delay_switch_half)){
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

	if(setpointAdjustmentType == REVERSESTOP){
		if ((fabsf(pitch_angle) > 15) || (switch_state == OFF)) {
			// Taking your foot off while reversing? Ignore delays
			state = FAULT_SWITCH_FULL;
			return true;
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

void calculate_setpoint_target(void){
	if(setpointAdjustmentType == CENTERING){
		if (setpoint_target_interpolated != setpoint_target){
			// Ignore tiltback during centering sequence
			state = RUNNING;
			softstart_timer = current_time;

			// disable soft start if startup speed is 10 or greater
			if (use_soft_start == false)
				setpointAdjustmentType = TILTBACK;
		}
		else if (ST2MS(current_time - softstart_timer) > SOFTSTART_GRACE_PERIOD_MS){
			// After a short delay transition to normal riding
			setpointAdjustmentType = TILTBACK;
		}
	}
	else if (setpointAdjustmentType == REVERSESTOP) {
		// accumalete erpms:
		reverse_total_erpm += erpm;
		if (fabsf(reverse_total_erpm) > reverse_tolerance) {
			// tilt down by 10 degrees after 50k aggregate erpm
			setpoint_target = 10 * REVERSE_ERPM_REPORTING * (fabsf(reverse_total_erpm)-reverse_tolerance) / 50000;
		}
		else {
			if (fabsf(reverse_total_erpm) <= reverse_tolerance/2) {
				if (REVERSE_ERPM_REPORTING * erpm >= 0){
					setpointAdjustmentType = TILTBACK;
					reverse_total_erpm = 0;
					setpoint_target = 0;
					integral = 0;
				}
			}
		}
	}
	else if(abs_duty_cycle > balance_conf.tiltback_duty){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_angle;
		}
		setpointAdjustmentType = TILTBACK;
		state = RUNNING_TILTBACK_DUTY;
	}else if(abs_duty_cycle > 0.05 && GET_INPUT_VOLTAGE() > balance_conf.tiltback_high_voltage){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_angle;
		}
		setpointAdjustmentType = TILTBACK;
		state = RUNNING_TILTBACK_HIGH_VOLTAGE;
		beep_alert(3, 0);
	}else if(abs_duty_cycle > 0.05 && GET_INPUT_VOLTAGE() < balance_conf.tiltback_low_voltage){
		if (abs_erpm > TILTBACK_LOW_VOLTAGE_MIN_ERPM){
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_angle;
			} else {
				setpoint_target = -balance_conf.tiltback_angle;
			}
			setpointAdjustmentType = TILTBACK;
			state = RUNNING_TILTBACK_LOW_VOLTAGE;
		}
		else if ((abs_erpm > TILTBACK_LOW_VOLTAGE_MIN_ERPM) ||
				 (GET_INPUT_VOLTAGE() < (balance_conf.tiltback_low_voltage - TILTBACK_LOW_VOLTAGE_SLOW_MARGIN))){
			// tiltback only if we're going fast, or if the voltage is REALLY low
			// e.g. if threshold is at 40V, we allow voltage down to 38V if going slow
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_angle;
			} else {
				setpoint_target = -balance_conf.tiltback_angle;
			}
			setpointAdjustmentType = TILTBACK;
			state = RUNNING_TILTBACK_LOW_VOLTAGE;
		}
	}else{
		// Normal running
		if (use_reverse_stop && (REVERSE_ERPM_REPORTING * erpm < 0)) {
			setpointAdjustmentType = REVERSESTOP;
			reverse_total_erpm = 0;
		}
		else if(abs_erpm > tiltback_constant_erpm){
			if (tiltback_erpmbased > 0) {
				// Speed based nose angle adjustment
				setpoint_target = tiltback_erpmbased * erpm;
			}
			else {
				// Fixed nose angle adjustment
				if(erpm > 0){
					setpoint_target = tiltback_constant;
				} else {
					setpoint_target = -tiltback_constant;
				}
			}
			setpointAdjustmentType = TILTBACK;
			state = RUNNING_TILTBACK_CONSTANT;
		}else{
			setpointAdjustmentType = TILTBACK;
			setpoint_target = 0;
			state = RUNNING;
		}
#ifdef HAS_EXT_BUZZER
		if (low_voltage_headsup_done == 0) {
			if(GET_INPUT_VOLTAGE() < balance_conf.tiltback_low_voltage + HEADSUP_LOW_VOLTAGE_MARGIN) {
				if (motor_current < 10) {
					low_voltage_headsup_done = 1;
					beep_alert(3, 0);
				}
			}
		}
#endif

		/*
#ifdef HAS_EXT_BUZZER
		if (mc_interface_temp_fet_filtered() > max_temp_fet) {
			// issue two long beeps if we entered max temp territory for our FETs
			beep_alert(2, 1);
		}
		#endif
		*/
	}
}

void calculate_setpoint_interpolated(void){
	if(setpoint_target_interpolated != setpoint_target){
		// If we are less than one step size away, go all the way
		if(fabsf(setpoint_target - setpoint_target_interpolated) < get_setpoint_adjustment_step_size()){
			setpoint_target_interpolated = setpoint_target;
		}else if (setpoint_target - setpoint_target_interpolated > 0){
			setpoint_target_interpolated += get_setpoint_adjustment_step_size();
		}else{
			if (setpointAdjustmentType == TILTBACK)
				setpoint_target_interpolated -= get_setpoint_adjustment_step_size() / 3;
			else
				setpoint_target_interpolated -= get_setpoint_adjustment_step_size();
		}
	}
}

static float brake_timer;
static BrakeMode brake_mode = BRAKE_NORMAL;

void apply_torquetilt(void){
	float start_current = balance_conf.torquetilt_start_current;
	if (start_current == 0)
		return;

	// Filter current (Basic LPF)
	torquetilt_filtered_current = ((1-balance_conf.torquetilt_filter) * motor_current) + (balance_conf.torquetilt_filter * torquetilt_filtered_current);

	if (SIGN(torquetilt_filtered_current) != SIGN(erpm)) {
		// current is negative, so we are braking or going downhill
		start_current = torquetilt_brk_start_current;

		if (brake_mode == BRAKE_NORMAL) {
			if (ST2MS(current_time - brake_timer) > torquetilt_brk_delay) {
				// enable downhill mode after a delay
				brake_mode = BRAKE_DOWNHILL;
				if (abs_erpm > 500)
				{}//beep_alert(1, 1);
			}
			else {
				// Only a sustained current above the threshold will trigger downhill mode 
				if (fabsf(torquetilt_filtered_current) < start_current) {
					brake_timer = current_time;
				}
				// Never Torque Tilt in BRAKE_NORMAL mode, to allow tail drags etc
				torquetilt_filtered_current = 0;
			}
		}
	}
	else {
		// cruising / accelerating - the only way to exit downhill mode
		if ((brake_mode == BRAKE_DOWNHILL) && (abs_erpm > 500))
		{}//beep_alert(1, 0);
		brake_timer = current_time;
		brake_mode = BRAKE_NORMAL;
	}

	float torquetilt_target;
	float torque_efficiency = (acceleration * 50.0) / torquetilt_filtered_current;
	if (torque_efficiency < 1.5) { // magic ratio
		// Take abs motor current, subtract start offset, and take the max of that with 0 to get the current above our start threshold (absolute).
		// Then multiply it by "power" to get our desired angle, and min with the limit to respect boundaries.
		// Finally multiply it by sign motor current to get directionality back
		torquetilt_target = fminf(fmaxf((fabsf(torquetilt_filtered_current) - start_current), 0) * balance_conf.torquetilt_strength, balance_conf.torquetilt_angle_limit) * SIGN(torquetilt_filtered_current);
	}
	else {
		// If the magic ratio is > 1 then we must be just accelerating. No TT here!!
		torquetilt_target = 0;
	}
	ttt = torquetilt_target;
	float accel = acceleration;
	if (SIGN(accel) != SIGN(torquetilt_filtered_current))
		accel = SIGN(torquetilt_filtered_current) * 0.1;
	exp_grunt_factor = fmaxf(fminf(torquetilt_filtered_current / (accel * 50.0), 20), -20);
	exp_g_max = fmaxf(exp_grunt_factor, exp_g_max);
	exp_g_min = fminf(exp_grunt_factor, exp_g_min);

	// don't get the board too "excited", don't let the nose rise much above 0 ;)
	float max_tilt = balance_conf.torquetilt_angle_limit / 4;
	bool nose_is_up = (SIGN(last_proportional - torquetilt_interpolated) != SIGN(torquetilt_target));
	float actual_tilt = fabsf(last_proportional - torquetilt_interpolated);

	if (nose_is_up && (fabsf(torquetilt_target) > 0) && (actual_tilt > max_tilt)) {
		torquetilt_target = torquetilt_target - SIGN(torquetilt_target) * actual_tilt * 0.5;
		expkp = 1;
	}
	else
		expkp = 0;

	// Deal with integral windup
	if (SIGN(integral) != SIGN(erpm)) { // integral windup after braking
		if ((torquetilt_target == 0)  && (fabsf(integral) > 2000) && (fabsf(pid_value) < 3)) {
			// we are back to 0 ttt, current is small, yet integral windup is high:
			// resort to brute force integral windup mitigation, shed 1% each cycle:
			// but only at low speeds (below 4mph)
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
		if ((torquetilt_target == 0)  && (fabsf(integral) > 2000) && (fabsf(pid_value) < 3) && (fabsf(erpm) < 1000)) {
			// we are back to 0 ttt, current is small, yet integral windup is high:
			// resort to brute force integral windup mitigation, shed 1% each cycle:
			// but only at low speeds (below 4mph)
			integral = integral * shedfactor;
		}
	}

	if(fabsf(torquetilt_target - torquetilt_interpolated) < torquetilt_step_size){
		torquetilt_interpolated = torquetilt_target;
	}else if (torquetilt_target - torquetilt_interpolated > 0){
		torquetilt_interpolated += torquetilt_step_size_down;
	}else{
		torquetilt_interpolated -= torquetilt_step_size;
	}
	setpoint += torquetilt_interpolated;
}

void apply_turntilt(void){
	// Calculate desired angle
	turntilt_target = abs_roll_angle_sin * balance_conf.turntilt_strength;

	// Apply cutzone
	if(abs_roll_angle < balance_conf.turntilt_start_angle){
		turntilt_target = 0;
	}

	// Disable below erpm threshold otherwise add directionality
	if(abs_erpm < balance_conf.turntilt_start_erpm){
		turntilt_target = 0;
	}else {
		turntilt_target *= SIGN(erpm);
	}

	// Apply speed scaling
	if(abs_erpm < balance_conf.turntilt_erpm_boost_end){
		turntilt_target *= 1 + ((balance_conf.turntilt_erpm_boost/100.0f) * (abs_erpm / balance_conf.turntilt_erpm_boost_end));
	}else{
		turntilt_target *= 1 + (balance_conf.turntilt_erpm_boost/100.0f);
	}

	// Limit angle to max angle
	turntilt_target = fminf(turntilt_target, balance_conf.turntilt_angle_limit);

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

static void update_lights(void){
	ride_state = new_ride_state;
	switch (ride_state) {
	case RIDE_OFF:
		LIGHT_FWD_OFF();
		LIGHT_BACK_OFF();
		BRAKE_LIGHT_OFF();
		break;
	case RIDE_IDLE:
		LIGHT_FWD_ON();
		LIGHT_BACK_ON();
		BRAKE_LIGHT_OFF();
		break;
	case RIDE_FORWARD:
		LIGHT_FWD_ON();
		LIGHT_BACK_OFF();
		BRAKE_LIGHT_OFF();
		break;
	case RIDE_REVERSE:
		LIGHT_FWD_OFF();
		LIGHT_BACK_ON();
		BRAKE_LIGHT_OFF();
		break;
	case BRAKE_FORWARD:
		LIGHT_FWD_ON();
		LIGHT_BACK_OFF();
		BRAKE_LIGHT_ON();
		break;
	case BRAKE_REVERSE:
		LIGHT_FWD_OFF();
		LIGHT_BACK_ON();
		BRAKE_LIGHT_ON();
		break;
	}
}

void brake(void){
	// Reset the timeout
	timeout_reset();
	// Set current
	mc_interface_set_brake_current(balance_conf.brake_current);
	beep_off(true);
	// we've stopped riding => turn the lights off
	// TODO: Add delay (to help spot the vehicle after a crash?)
	new_ride_state = RIDE_OFF;
	update_lights();
}

void set_current(float current){
	// Reset the timeout
	timeout_reset();
	// Set current
	mc_interface_set_current(current);
}

void app_balance_stop(void) {
	if(app_thread != NULL){
		chThdTerminate(app_thread);
		chThdWait(app_thread);
	}
	set_current(0);
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
		last_time = current_time;

		// Read values for GUI
		motor_current = mc_interface_get_tot_current_directional_filtered();
		motor_position = mc_interface_get_pid_pos_now();

		// Get the values we want
		pitch_angle = imu_get_pitch() * 180.0f / M_PI;
		roll_angle = imu_get_roll() * 180.0f / M_PI;
		abs_roll_angle = fabsf(roll_angle);
		abs_roll_angle_sin = sinf(abs_roll_angle * M_PI / 180.0f);
		imu_get_gyro(gyro);
		duty_cycle = mc_interface_get_duty_cycle_now();
		abs_duty_cycle = fabsf(duty_cycle);
		erpm = mc_interface_get_rpm();
		abs_erpm = fabsf(erpm);

		//if (sampleIdx == 0) {
		//	OneKSamples1[0] = 999.999;
		//	OneKSamples2[0] = 999.999;
		//	sampleIdx++;
		//}
		float acc[3];
		imu_get_accel(acc);
		/*OneKSamples1[sampleIdx] = motor_current;
		OneKSamples2[sampleIdx] = acc[0];
		sampleIdx++;
		if (sampleIdx == 100)
		sampleIdx = 0;*/

		acceleration = biquad_filter(erpm - last_erpm);
		last_erpm = erpm;

		// For logging only:
		expacc = acceleration;
		expki = integral;
		//expaccmin = fminf(expaccmin, acceleration);
		//expaccmax = fmaxf(expaccmax, acceleration);

		adc1 = (((float)ADC_Value[ADC_IND_EXT])/4095) * V_REG;
#ifdef ADC_IND_EXT2
		adc2 = (((float)ADC_Value[ADC_IND_EXT2])/4095) * V_REG;
#else
		adc2 = 0.0;
#endif

		// Calculate switch state from ADC values
		if(balance_conf.fault_adc1 == 0 && balance_conf.fault_adc2 == 0){ // No Switch
			switch_state = ON;
		}else if(balance_conf.fault_adc2 == 0){ // Single switch on ADC1
			if(adc1 > balance_conf.fault_adc1){
				switch_state = ON;
			} else {
				switch_state = OFF;
			}
		}else if(balance_conf.fault_adc1 == 0){ // Single switch on ADC2
			if(adc2 > balance_conf.fault_adc2){
				switch_state = ON;
			} else {
				switch_state = OFF;
			}
		}else{ // Double switch
			if(adc1 > balance_conf.fault_adc1 && adc2 > balance_conf.fault_adc2){
				switch_state = ON;
			}else if(adc1 > balance_conf.fault_adc1 || adc2 > balance_conf.fault_adc2){
				switch_state = HALF;
			}else{
				switch_state = OFF;
			}
		}

		/*
		 * Use external buzzer to notify rider of foot switch faults.
		 */
#ifdef HAS_EXT_BUZZER
		if (switch_state == OFF) {
			if ((abs_erpm > balance_conf.fault_adc_half_erpm)
				&& (state >= RUNNING)
				&& (state <= RUNNING_TILTBACK_CONSTANT))
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
#endif

		// Control Loop State Logic
		switch(state){
			case (STARTUP):
				while(!imu_startup_done()){
					// Disable output
					brake();
					// Wait
					chThdSleepMilliseconds(50);
				}
				reset_vars();
				state = FAULT_STARTUP; // Trigger a fault so we need to meet start conditions to start
				log_balance_state = state;

				// Let the rider know that the board is ready
				beep_on(1);
				chThdSleepMilliseconds(100);
				beep_off(1);

				// Issue 1 beep for each volt below 45
				double bat_volts = GET_INPUT_VOLTAGE();
				double threshold = balance_conf.tiltback_low_voltage + 5;
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
				autosuspend_timer = -1;

				break;
			case (RUNNING):
			case (RUNNING_TILTBACK_DUTY):
			case (RUNNING_TILTBACK_HIGH_VOLTAGE):
			case (RUNNING_TILTBACK_LOW_VOLTAGE):
			case (RUNNING_TILTBACK_CONSTANT):
				log_balance_state = state + 100 * setpointAdjustmentType;
				autosuspend_timer = -1;

				// Check for faults
				if(check_faults(false)){
					break;
				}

				// Calculate setpoint and interpolation
				calculate_setpoint_target();
				calculate_setpoint_interpolated();
				setpoint = setpoint_target_interpolated;
				if (disable_all_5_3_features == false) {
					if (setpointAdjustmentType == TILTBACK) {
						apply_torquetilt();
						apply_turntilt();
					}
				}

				// Do PID maths
				proportional = setpoint - pitch_angle;
				integral = integral + proportional;


				// Don't include setpoint adjustment in derivative (Courtesy of GrandmaB)
				derivative = last_pitch_angle - pitch_angle;//proportional - last_proportional;
				last_pitch_angle = pitch_angle;
				last_proportional = proportional;

				// Apply D term only filter
				if(balance_conf.kd_pt1_frequency > 0){
					d_pt1_state = d_pt1_state + d_pt1_k * (derivative - d_pt1_state);
					derivative = d_pt1_state;
				}

				if (disable_all_5_3_features == false) {
					// Limit integral
					integral = fminf(integral, integral_max);
					integral = fmaxf(integral, integral_min);

					// Switch between breaking PIDs and acceleration PIDs
					float kp_target, ki_target, kd_target;
					if (SIGN(pid_value) == SIGN(REVERSE_ERPM_REPORTING * erpm)) {
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
				}

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

				expsetpoint = setpoint;

				// Apply Booster
				if (setpointAdjustmentType == TILTBACK) {
					int booster_pid;
					abs_proportional = fabsf(proportional);
					if(abs_proportional > balance_conf.booster_angle){
						if(abs_proportional - balance_conf.booster_angle < balance_conf.booster_ramp){
							booster_pid = balance_conf.booster_current * SIGN(proportional) * ((abs_proportional - balance_conf.booster_angle) / balance_conf.booster_ramp);
						}else{
							booster_pid = balance_conf.booster_current * SIGN(proportional);
						}
						if (booster_beep && !booster_beeping) {
							beep_alert(1, 0);
							booster_beeping = 1;
						}
					}
					else {
						if (booster_beeping) {
							booster_beeping = 0;
						}
						booster_pid = 0;
					}
					pid_value += booster_pid;
				}

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

				if (autosuspend_timer == -1)
					autosuspend_timer = current_time;

				if (autosuspend_timeout &&
					(ST2S(current_time - autosuspend_timer) > autosuspend_timeout)){
					// Timeout: no way to return from here (requires power cycling)

					// beep-beep-beeeep
					beep_on(true);
					chThdSleepMilliseconds(100);
					beep_off(true);
					chThdSleepMilliseconds(100);
					beep_on(true);
					chThdSleepMilliseconds(100);
					beep_off(true);
					chThdSleepMilliseconds(100);
					beep_on(true);
					chThdSleepMilliseconds(500);
					beep_off(true);

					// stop balance app
					app_balance_stop();

					// now reduce power to minimum
					//imu_stop();
					//hw_stop_i2c();
					//LED_GREEN_OFF();
					break;
				}

				new_ride_state = RIDE_OFF;
				// Check for valid startup position and switch state
				if(fabsf(pitch_angle) < balance_conf.startup_pitch_tolerance && fabsf(roll_angle) < balance_conf.startup_roll_tolerance && switch_state == ON){
					reset_vars();
					update_lights();
					break;
				}
				// Disable output
				brake();
				break;
			case (FAULT_DUTY):
				log_balance_state = FAULT_DUTY;
				new_ride_state = RIDE_OFF;
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
		chThdSleepMicroseconds((int)((1000.0 / balance_conf.hertz) * 1000.0));
	}
	// in case we leave this force the buzzer off (force=regardless of ongoing multi beeps)
	beep_off(true);

	// Disable output
	brake();
}
