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
#include "terminal.h"
#include "mcpwm_foc.h"
#include "buzzer.h"


#include <math.h>
#include <stdio.h>

// Can
#define MAX_CAN_AGE 0.1

// Acceleration average
#define ACCEL_ARRAY_SIZE 40

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
	FAULT_STARTUP = 11,
	FAULT_REVERSE = 12
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
static float tiltback_variable, tiltback_variable_max_erpm;
static float tt_pid_intensity, tt_strength_uphill, tt_strength_downhill, integral_tt_impact_uphill, integral_tt_impact_downhill;
static bool allow_high_speed_full_switch_faults;
static float mc_current_max, mc_current_min;
static float mc_max_temp_fet;

// Feature: Reverse Stop
static float reverse_stop_step_size, reverse_tolerance, reverse_total_erpm;
static systime_t reverse_timer;
static bool use_reverse_stop;

// Feature: Soft Start
#define START_GRACE_PERIOD_MS 100
static systime_t softstart_timer;
static bool use_soft_start;
static int start_counter_ms;
static unsigned int start_counter_clicks, start_counter_clicks_max, click_current;

// Feature: Adaptive Torque Response
static float acceleration, acceleration_raw, last_erpm, shedfactor;
static float accel_gap;
static float torquetilt_target;
static int tttarget_lag;
static int erpm_sign;

// Feature: Turntilt
static float last_yaw_angle, yaw_angle, abs_yaw_change, yaw_change, yaw_aggregate;
static float tuntilt_boost_per_erpm, yaw_aggregate_target;

// Feature: PID toning
static float center_boost_angle, center_boost_kp_adder;
static float max_brake_amps, max_derivative;
static float accel_boost_threshold, accel_boost_threshold2, accel_boost_intensity;
#define BOOST_THRESHOLD 8
#define BOOST_THRESHOLD2 14
#define BOOST_INTENSITY 0.5;

// Inactivity Timeout
static float inactivity_timer, inactivity_timeout;

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
static float last_proportional;
static float pid_value;
static float setpoint, setpoint_target, setpoint_target_interpolated;
static float noseangling_interpolated;
static float torquetilt_filtered_current, torquetilt_interpolated;
static Biquad torquetilt_current_biquad, accel_biquad;
static float turntilt_target, turntilt_interpolated;
static SetpointAdjustmentType setpointAdjustmentType;
static systime_t current_time, last_time, diff_time, loop_overshoot;
static float filtered_loop_overshoot, loop_overshoot_alpha, filtered_diff_time;
static systime_t fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer, fault_switch_half_timer, fault_duty_timer, tb_highvoltage_timer;
static float kp, ki, kd, kp_acc, ki_acc, kd_acc, kp_brk, ki_brk, kd_brk;
static float d_pt1_lowpass_state, d_pt1_lowpass_k;
static float motor_timeout;
static systime_t brake_timeout;
static float accelhist[ACCEL_ARRAY_SIZE];
static int accelidx;
static float accelavg;

// Lock
static int lock_state;
static bool is_locked;

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
static void check_lock(void);

// Utility Functions
float biquad_process(Biquad *biquad, float in) {
    float out = in * biquad->a0 + biquad->z1;
    biquad->z1 = in * biquad->a1 + biquad->z2 - biquad->b1 * out;
    biquad->z2 = in * biquad->a2 - biquad->b2 * out;
    return out;
}
void biquad_config(Biquad *biquad, BiquadType type, float Fc) {
	float K = tanf(M_PI * Fc);	// -0.0159;
	float Q = 0.5; // maximum sharpness (0.5 = maximum smoothness)
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

// Wiggle the motor a little bit at different frequencies
// In case the frequency change stuff isn't safe
// we can also wiggle the motor a bit (using 1A) without
// changing the switching frequency
static void play_tune(bool doChangeFreqs) {
	float original_sw = mc_interface_get_configuration()->foc_f_sw;
	float curr = 1;
	int freqs[] = { 2093, 2637, 3135, 4186 };
	for( unsigned int i = 0; i < sizeof(freqs)/sizeof(int); i++ ) {
		if (doChangeFreqs)
			mcpwm_foc_change_sw(freqs[i]);
		mc_interface_set_current(curr);
		chThdSleepMilliseconds(100);
		mc_interface_set_current(0);
		chThdSleepMilliseconds(10);
		curr = -curr;
		if (!doChangeFreqs && i)	// no tune? Limit to 1 back and forth wiggle
			break;
	}
	//go back to original switching frequency
	if (doChangeFreqs)
		mcpwm_foc_change_sw((int)original_sw);
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

	// Feature: Stealthy start vs normal start (noticeable click when engaging)
	start_counter_clicks_max = 2;
	int bc = balance_conf.brake_current;
	click_current = (balance_conf.brake_current - bc) * 100;
	click_current = fminf(click_current, 30);

	// Feature: Reverse Stop (ON if startup_speed ends in .1)
	// startup_speed = x.0: noticeable click on start, no reverse stop
	// startup_speed = x.1: noticeable click on start, reverse stop
	// startup_speed = x.2: stealthy start, no reverse stop
	// startup_speed = x.3: stealthy start + reverse stop
	use_reverse_stop = false;
	reverse_tolerance = 50000;
	reverse_stop_step_size = 100.0 / balance_conf.hertz;
	float startup_speed = balance_conf.startup_speed;
	int ss = (int) startup_speed;
	float ss_rest = startup_speed - ss;
	if ((ss_rest > 0.09) && (ss_rest < 0.11)) {
		use_reverse_stop = true;
	}
	else if ((ss_rest > 0.19) && (ss_rest < 0.21)) {
		start_counter_clicks_max = 0;
	}
	else if ((ss_rest > 0.29) && (ss_rest < 0.31)) {
		start_counter_clicks_max = 0;
		use_reverse_stop = true;
	}

	// Feature: Soft Start
	use_soft_start = (balance_conf.startup_speed < 10);

	// if the full switch delay ends in 1, we don't allow high speed full switch faults
	int fullswitch_delay = balance_conf.fault_delay_switch_full / 10;
	int delay_rest = balance_conf.fault_delay_switch_full - (fullswitch_delay * 10);
	allow_high_speed_full_switch_faults = (delay_rest != 1);

	// Feature: ATR
	shedfactor = 0.996;
	// guardrails:
	if (shedfactor > 1)
		shedfactor = 0.99;
	if (shedfactor < 0.5)
		shedfactor = 0.98;

	// Feature: Turntilt
	yaw_aggregate_target = balance_conf.yaw_ki;			// borrow yaw_ki for aggregate yaw-change target
	tuntilt_boost_per_erpm = (float)balance_conf.turntilt_erpm_boost / 100.0 / (float)balance_conf.turntilt_erpm_boost_end;

	// Braking PID softness
	int brake_pid_scaling = balance_conf.kd_pt1_highpass_frequency;
	int brake_kp_scaling = brake_pid_scaling / 100;
	int brake_ki_scaling = (brake_pid_scaling - (brake_kp_scaling * 100)) / 10;
	int brake_kd_scaling = brake_pid_scaling - (brake_kp_scaling * 100) - (brake_ki_scaling * 10);
	if (brake_kp_scaling < 2) {
		brake_kp_scaling = 10;
		brake_ki_scaling = 10;
		brake_kd_scaling = 10;
	}
	if (brake_ki_scaling < 1)
		brake_ki_scaling = 10;
	if (brake_kd_scaling < 5)
		brake_kd_scaling = 10;

	// Guardrails for Onewheel PIDs (outlandish PIDs can break your motor!)
	kp_acc = fminf(balance_conf.kp, 10);
	ki_acc = fminf(balance_conf.ki, 0.01);
	kd_acc = fminf(balance_conf.kd, 1500);
	// Reduced braking PIDs:
	kp_brk = fminf(kp_acc / 10.0 * brake_kp_scaling, 5);
	ki_brk = fminf(ki_acc / 10.0 * brake_ki_scaling, 0.005);
	kd_brk = fminf(kd_acc / 10.0 * brake_kd_scaling, 1000);

	// How much does Torque-Tilt stiffen PIDs - intensity = 1 doubles PIDs at 6 degree TT
	tt_pid_intensity = balance_conf.booster_current;
	tt_pid_intensity = fminf(tt_pid_intensity, 1.5);
	tt_pid_intensity = fmaxf(tt_pid_intensity, 0);

	// Torque-Tilt strength is different for up vs downhills
	tt_strength_uphill = balance_conf.torquetilt_strength * 10;
	if (tt_strength_uphill > 2.5)
		tt_strength_uphill = 1.5;
	if (tt_strength_uphill < 0)
		tt_strength_uphill = 0;
	// Downhill strength must be higher since downhill amps tend to be lower than uphill amps
	tt_strength_downhill = tt_strength_uphill * (1 + balance_conf.yaw_kp / 100);

	// Any value above 0 will increase the board angle to match the slope
	integral_tt_impact_downhill = 1.0 - (float)balance_conf.kd_biquad_lowpass / 100.0;
	integral_tt_impact_uphill = 1.0 - (float)balance_conf.kd_biquad_highpass / 100.0;
	integral_tt_impact_downhill = fminf(integral_tt_impact_downhill, 1.0);
	integral_tt_impact_downhill = fmaxf(integral_tt_impact_downhill, 0.0);
	integral_tt_impact_uphill = fminf(integral_tt_impact_uphill, 1.0);
	integral_tt_impact_uphill = fmaxf(integral_tt_impact_uphill, 0.0);

	// Init Filters
	if(balance_conf.loop_time_filter > 0){
		loop_overshoot_alpha = 2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter/(2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter+1);
	}

	// Use only pt1 lowpass filter for DTerm (limited to 1..30, default to 10)
	float dt_filter_freq = 10;
	if(balance_conf.kd_pt1_lowpass_frequency >= 1){
		dt_filter_freq = balance_conf.kd_pt1_lowpass_frequency;
	}
	if (dt_filter_freq > 30){
		dt_filter_freq = 10;
	}
	float dT = 1.0 / balance_conf.hertz;
	float RC = 1.0 / ( 2.0 * M_PI * dt_filter_freq);
	d_pt1_lowpass_k =  dT / (RC + dT);

	// Torquetilt Current Biquad
	float tt_filter = balance_conf.torquetilt_filter;
	if (tt_filter == 0)
		tt_filter = 5;
	if (tt_filter > 30)
		tt_filter = 30;
	float Fc = balance_conf.torquetilt_filter / balance_conf.hertz;
	biquad_config(&torquetilt_current_biquad, BQ_LOWPASS, Fc);

	// Feature: PID Toning
	center_boost_angle = balance_conf.booster_angle;
	center_boost_kp_adder = (balance_conf.booster_ramp / 3.5 * kp_acc)  - kp_acc; // scaled to match configured P
	if (center_boost_kp_adder < 0)
		center_boost_kp_adder = 1;
	if (center_boost_angle > 3)
		center_boost_angle = 1;
	center_boost_kp_adder = fminf(center_boost_kp_adder, 7);

	// Feature: Boost
	accel_boost_threshold = BOOST_THRESHOLD;
	accel_boost_threshold2 = BOOST_THRESHOLD2;
	accel_boost_intensity = BOOST_INTENSITY;
	if ((app_get_configuration()->app_nrf_conf.retry_delay == NRF_RETR_DELAY_3750US) &&
		(app_get_configuration()->app_nrf_conf.retries == 13)) {
		// NRF config is used to customize boost parameters
		accel_boost_threshold = (float)app_get_configuration()->app_nrf_conf.address[0];
		accel_boost_threshold2 = (float)app_get_configuration()->app_nrf_conf.address[1];
		accel_boost_intensity = ((float)app_get_configuration()->app_nrf_conf.address[2]) / 10.0;
		// Turn off booster if bogus values attempted
		if ((accel_boost_threshold < 4) || (accel_boost_threshold > 20))
			accel_boost_intensity = 0;
		else if ((accel_boost_threshold2 < accel_boost_threshold) || (accel_boost_threshold2 > 20))
			accel_boost_intensity = 0;
		else if ((accel_boost_intensity < 0) || (accel_boost_intensity > 1))
			accel_boost_intensity = 0;
	}

	// Roll-Steer KP controls max brake amps (for P+D) AND max derivative amps
	max_brake_amps = balance_conf.roll_steer_kp;
	if (max_brake_amps < 10)
		max_brake_amps = mc_interface_get_configuration()->l_current_max / 2;

	int mb = max_brake_amps;
	max_derivative = 100 * (max_brake_amps - mb);
	if (max_derivative < 10)
		max_derivative = mc_interface_get_configuration()->l_current_max / 2;

	// Feature: ATR
	// Borrow "Roll-Steer KP" value to control the acceleration biquad low-pass filter:
	float cutoff_freq = 50; // balance_conf.roll_steer_kp;
	if (cutoff_freq < 10)
		cutoff_freq = 10;
	if (cutoff_freq > 100)
		cutoff_freq = 100;
	biquad_config(&accel_biquad, BQ_LOWPASS, cutoff_freq / ((float)balance_conf.hertz));

	// Variable nose angle adjustment / tiltback (setting is per 1000erpm, convert to per erpm)
	tiltback_variable = balance_conf.tiltback_variable / 1000;
	tiltback_variable_max_erpm = fabsf(balance_conf.tiltback_variable_max / tiltback_variable);

	// Reset loop time variables
	last_time = 0;
	filtered_loop_overshoot = 0;

	if (mc_interface_get_configuration()->m_invert_direction)
		erpm_sign = -1;
	else
		erpm_sign = 1;

	mc_current_max = mc_interface_get_configuration()->l_current_max;
	mc_current_min = mc_interface_get_configuration()->l_current_min;
	mc_max_temp_fet = mc_interface_get_configuration()->l_temp_fet_start - 2;

	switch (app_get_configuration()->shutdown_mode) {
	case SHUTDOWN_MODE_OFF_AFTER_10S: inactivity_timeout = 10; break;
	case SHUTDOWN_MODE_OFF_AFTER_1M: inactivity_timeout = 60; break;
	case SHUTDOWN_MODE_OFF_AFTER_5M: inactivity_timeout = 60 * 5; break;
	case SHUTDOWN_MODE_OFF_AFTER_10M: inactivity_timeout = 60 * 10; break;
	case SHUTDOWN_MODE_OFF_AFTER_30M: inactivity_timeout = 60 * 30; break;
	case SHUTDOWN_MODE_OFF_AFTER_1H: inactivity_timeout = 60 * 60; break;
	case SHUTDOWN_MODE_OFF_AFTER_5H: inactivity_timeout = 60 * 60 * 5; break;
	default:
		inactivity_timeout = 0;
	}
	inactivity_timer = -1;

	// Lock:
	lock_state = -1;
	is_locked = balance_conf.multi_esc;
}

void app_balance_start(void) {
	// First start only, override state to startup
	state = STARTUP;
	play_tune(balance_conf.deadzone == 1);
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
	// Set values for startup
	setpoint = pitch_angle;
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
	biquad_reset(&accel_biquad);
	accel_gap = 0;
	pid_value = 0;
	tttarget_lag = 0;

	for (int i=0; i<40; i++)
		accelhist[i] = 0;
	accelidx = 0;
	accelavg = 0;

	// Start with a minimal backwards push
	float start_offset_angle = balance_conf.startup_pitch_tolerance + 1;
	setpoint_target_interpolated = (fabsf(pitch_angle) - start_offset_angle) * SIGN(pitch_angle);

	// Soft-start vs normal aka quick-start:
	if (use_soft_start) {
		// minimum values (even 0,0,0 is possible) for soft start:
		kp = 1;
		ki = 0;
		kd = 10;
	}
	else {
		// Normal start / quick-start
		kp = kp_acc * 0.8;
		ki = ki_acc;
		kd = kd_acc * 0.5;
	}
	// first 100ms we don't want to use braking PIDs (hard coded to assume loop-Hz=1000)
	start_counter_ms = START_GRACE_PERIOD_MS;
	start_counter_clicks = start_counter_clicks_max;
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

	// Feature: Reverse-Stop
	if(setpointAdjustmentType == REVERSESTOP){
		//  Taking your foot off entirely while reversing? Ignore delays
		if (switch_state == OFF) {
			state = FAULT_SWITCH_FULL;
			return true;
		}
		if (fabsf(pitch_angle) > 15) {
			state = FAULT_REVERSE;
			return true;
		}
		// Above 10 degrees for a half a second? Switch it off
		if ((fabsf(pitch_angle) > 10) && (ST2MS(current_time - reverse_timer) > 500)) {
			state = FAULT_REVERSE;
			return true;
		}
		// Above 5 degrees for a full second? Switch it off
		if ((fabsf(pitch_angle) > 5) && (ST2MS(current_time - reverse_timer) > 1000)) {
			state = FAULT_REVERSE;
			return true;
		}
		if (reverse_total_erpm > reverse_tolerance * 3) {
			state = FAULT_REVERSE;
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
	if (GET_INPUT_VOLTAGE() < balance_conf.tiltback_hv) {
		tb_highvoltage_timer = current_time;
	}

	if(setpointAdjustmentType == CENTERING) {
		if (setpoint_target_interpolated != setpoint_target){
			// Ignore tiltback during centering sequence
			state = RUNNING;
			softstart_timer = current_time;
		}
		else if (ST2MS(current_time - softstart_timer) > START_GRACE_PERIOD_MS){
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
	}else if(GET_INPUT_VOLTAGE() > balance_conf.tiltback_hv){
		if ((ST2MS(current_time - fault_switch_timer) > 500) ||
			(GET_INPUT_VOLTAGE() > balance_conf.tiltback_hv + 1)) {
			// 500ms have passed or voltage is another volt higher, time for some tiltback
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_hv_angle;
			} else {
				setpoint_target = -balance_conf.tiltback_hv_angle;
			}
			setpointAdjustmentType = TILTBACK_HV;
			state = RUNNING_TILTBACK_HIGH_VOLTAGE;
		}
		else {
			// The rider has 500ms to react to the triple-beep, or maybe it was just a short spike
			setpointAdjustmentType = TILTBACK_NONE;
			state = RUNNING;
		}
		beep_alert(3, 0);	// Triple-beep
	}else if(GET_INPUT_VOLTAGE() < balance_conf.tiltback_lv){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_lv_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_lv_angle;
		}
		setpointAdjustmentType = TILTBACK_LV;
		state = RUNNING_TILTBACK_LOW_VOLTAGE;
		beep_alert(3, 0);	// Triple-beep
	}else if(mc_interface_temp_fet_filtered() > mc_max_temp_fet){
		// Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
		beep_alert(3, 1);	// Triple-beep (long beeps)
		if(mc_interface_temp_fet_filtered() > (mc_max_temp_fet + 1)) {
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_lv_angle;
			} else {
				setpoint_target = -balance_conf.tiltback_lv_angle;
			}
			setpointAdjustmentType = TILTBACK_HV;
			state = RUNNING_TILTBACK_LOW_VOLTAGE;
		}
		else {
			// The rider has 1 degree Celsius left before we start tilting back
			setpointAdjustmentType = TILTBACK_NONE;
			state = RUNNING;
		}
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
	// Skip torque tilt logic if strength is 0
	if (balance_conf.torquetilt_strength == 0)
		return;

	torquetilt_filtered_current = biquad_process(&torquetilt_current_biquad, motor_current);
	float abs_torque = fabsf(torquetilt_filtered_current);
	int torque_sign = SIGN(torquetilt_filtered_current);
	float torque_offset = balance_conf.torquetilt_start_current;

	float torquetilt_strength = tt_strength_uphill;
	const float accel_factor = balance_conf.yaw_kd;
	int braking_sign = 1;	// 1 = accel, -1 = braking

	if ((abs_erpm > 250) && (torque_sign != SIGN(erpm))) {
		// current is negative, so we are braking or going downhill
		// high currents downhill are less likely
		torquetilt_strength = tt_strength_downhill;
		braking_sign = -1;
	}

	// compare measured acceleration to expected acceleration
	float measured_acc = fmaxf(acceleration, -5);
	measured_acc = fminf(acceleration, 5);

	// expected acceleration is proportional to current (minus an offset, required to balance/maintain speed)
	float expected_acc = (abs_torque - braking_sign * torque_offset) / accel_factor * torque_sign;

	bool static_climb = false;
	float acc_diff = expected_acc - measured_acc;
	if (abs_erpm > 2000)
		accel_gap = 0.9 * accel_gap + 0.1 * acc_diff;
	else if (abs_erpm > 1000)
		accel_gap = 0.95 * accel_gap + 0.05 * acc_diff;
	else if (abs_erpm > 250)
		accel_gap = 0.98 * accel_gap + 0.02 * acc_diff;
	else {
		// low speed erpms are VERY choppy/noisy - ignore them if we're not trying to actually accelerate
		if (fabsf(expected_acc) < 1)
			accel_gap = 0;
		else if (fabsf(expected_acc) < 1.5) {
			if (fabsf(accel_gap > 1)) {
				// Once the gap is above 1 we get more aggressive
				accel_gap = 0.9 * accel_gap + 0.1 * acc_diff;
				static_climb = true;
			}
			else
				// Until the gap is below 1 we use a strong filter because of noise
				accel_gap = 0.99 * accel_gap + 0.01 * acc_diff;
		}
		else {
			if (fabsf(accel_gap > 1)) {
				accel_gap = 0.9 * accel_gap + 0.1 * acc_diff;
				static_climb = true;
			}
			else
				accel_gap = 0.95 * accel_gap + 0.05 * acc_diff;
		}
	}

	// now torquetilt target is purely based on gap between expected and actual acceleration
	torquetilt_target = torquetilt_strength * accel_gap;
	torquetilt_target = fminf(torquetilt_target, balance_conf.torquetilt_angle_limit);
	torquetilt_target = fmaxf(torquetilt_target, -balance_conf.torquetilt_angle_limit);

	// Deal with integral windup
	int sign_erpm = SIGN(erpm);
	if (SIGN(integral) != sign_erpm) { // integral windup after braking
		if ((torquetilt_target >= 0) && (pid_value > 10)) {
			// we are back to 0 ttt, current is small, yet integral windup is high:
			// resort to brute force integral windup mitigation, shed a tiny fraction each cycle:
			// at any speeds
			if (torquetilt_interpolated < 0)
				torquetilt_interpolated *= 0.98;
			if (fabsf(integral) > 2000) {
				integral *= shedfactor;
				beep_on(1);
			}
			else {
				beep_off(0);
			}
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

		if (torquetilt_target == 0) {
			// This here is for (a) - once pitch crosses to zero at low erpms this should be safe to do!
			if ((SIGN(pitch_angle) == sign_erpm) && (fabsf(erpm) < 1000)) {
				// we are back to 0 ttt, current is small, yet integral windup is high:
				// resort to brute force integral windup mitigation, shed 1% each cycle:
				// but only at low speeds (below 4mph)
				if (fabsf(integral) > 2000) {
					integral *= 0.998;
					beep_on(1);
				}
				else {
					beep_off(0);
				}
				torquetilt_interpolated *= 0.99;//shedfactor;
			}
			// This here is for (b)
			// ???
		}
	}

	float step_size;
	if(((torquetilt_target > -2) && (torquetilt_interpolated > torquetilt_target) && (torquetilt_interpolated > 0)) ||
	   ((torquetilt_target < 2) && (torquetilt_interpolated < torquetilt_target) && (torquetilt_interpolated < 0))) {
		float tttarget_diff = fabsf(torquetilt_interpolated - torquetilt_target);
		if (tttarget_diff > 0.3)
			tttarget_lag++;
		else
			tttarget_lag = 0;

		if ((tttarget_lag > 100) && (abs_erpm < 800))
			step_size = torquetilt_on_step_size / 2;	// we've been going back to center for 100ms now, speed it up!
		else
			step_size = torquetilt_off_step_size;		// to avoid oscillations we go down slower than we go up
	}else{
		tttarget_lag = 0;

		// reduce response speed when going downhill/braking
		if (braking_sign == -1)
			step_size = torquetilt_on_step_size / 2;
		else {
			step_size = torquetilt_on_step_size;
			if (static_climb)
				step_size = step_size * 1.5;
		}
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
	if((abs_yaw_change * 100 < balance_conf.turntilt_start_angle) || (state != RUNNING)) {
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

		// Get the values we want
		last_pitch_angle = pitch_angle;
		pitch_angle = imu_get_pitch() * 180.0f / M_PI;
		roll_angle = imu_get_roll() * 180.0f / M_PI;
		abs_roll_angle = fabsf(roll_angle);
		abs_roll_angle_sin = sinf(abs_roll_angle * M_PI / 180.0f);
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
		acceleration_raw = smooth_erpm - last_erpm;
		//acceleration = biquad_process(&accel_biquad, acceleration_raw);
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
				inactivity_timer = -1;
				break;

			case (RUNNING):
			case (RUNNING_TILTBACK_DUTY):
			case (RUNNING_TILTBACK_HIGH_VOLTAGE):
			case (RUNNING_TILTBACK_LOW_VOLTAGE):
				log_balance_state = state + 100 * setpointAdjustmentType;
				inactivity_timer = -1;

				// Check for faults
				if(check_faults(false)){
					break;
				}

				if(start_counter_ms){
					start_counter_ms--;
				}

				// Calculate setpoint and interpolation
				calculate_setpoint_target();
				calculate_setpoint_interpolated();
				setpoint = setpoint_target_interpolated;
				if ((setpointAdjustmentType != CENTERING) && (setpointAdjustmentType != REVERSESTOP)) {
					apply_noseangling();	// Always do nose angling, even during tiltback situations
					apply_torquetilt();		// Torquetilt remains in effect even during tiltback situations
					apply_turntilt();
				}

				// Do PID maths
				proportional = setpoint - pitch_angle;

				// Integral component, only partially affected by torquetilt
				integral = integral + proportional;
				// Produce controlled nose/tail lift with increased torque
				float tt_impact;
				if (torquetilt_interpolated < 0)
					// Downhill tail lift doesn't need to be as high as uphill nose lift
					tt_impact = integral_tt_impact_downhill;
				else {
					tt_impact = integral_tt_impact_uphill;
					if (abs_erpm < 4000) {
						// Reduced nose lift at lower speeds
						// Creates a value between 0.5 and 1.0
						float erpm_scaling = fmaxf(0.5, abs_erpm / 4000);
						tt_impact = (1.0 - (1.0 - tt_impact) * erpm_scaling);
					}
				}
				integral -= torquetilt_interpolated * tt_impact;

				// Derivative with D term PT1 filter
				derivative = last_pitch_angle - pitch_angle;
				d_pt1_lowpass_state = d_pt1_lowpass_state + d_pt1_lowpass_k * (derivative - d_pt1_lowpass_state);
				derivative = d_pt1_lowpass_state;

				// Identify braking based on angle of the board vs direction of movement
				bool braking = (SIGN(proportional) != SIGN(erpm));

				// Switch between soft breaking PIDs and harder acceleration / torquetilt PIDs
				float kp_target, ki_target, kd_target;
				if (!braking ||
					(fabsf(torquetilt_interpolated) > 1) ||
					(start_counter_ms))
				{
					// acceleration and torquetilt situations
					float pi_multiplier = 1;
					if (fabsf(torquetilt_interpolated) > 2) {
						// torque stiffness
						pi_multiplier = fabsf(torquetilt_interpolated) / 6 * tt_pid_intensity;
						pi_multiplier = fminf(1 + pi_multiplier, 2);
					}
					kp_target = kp_acc * pi_multiplier;
					ki_target = ki_acc * pi_multiplier;
					kd_target = kd_acc;
				}
				else {
					// braking
					kp_target = kp_brk;
					ki_target = ki_brk;
					kd_target = kd_brk;
				}
				if (setpointAdjustmentType == REVERSESTOP) {
					kp_target = 2;
					kd_target = 400;
					integral = 0;
				}

				// Ensure smooth transition between different PID targets
				if (kp_target > kp) {
					// stiffen quickly
					kp = kp * 0.99 + kp_target * 0.01;
					ki = ki * 0.99 + ki_target * 0.01;
					kd = kd * 0.99 + kd_target * 0.01;
				}
				else {
					// loosen slowly
					kp = kp * 0.999 + kp_target * 0.001;
					ki = ki * 0.999 + ki_target * 0.001;
					kd = kd * 0.999 + kd_target * 0.001;
				}

				float pid_prop = 0, pid_derivative = 0, pid_integral = 0;

				if (use_soft_start && (setpointAdjustmentType == CENTERING)) {
					// soft-start
					float pid_target = (kp * proportional) + (kd * derivative);
					pid_value = 0.05 * pid_target + 0.95 * pid_value;
					// once centering is done, start integral component from 0
					integral = 0;
					ki = 0;
				}
				else {
					// P:
					// use higher kp for first few degrees of proportional to keep the board more stable
					pid_prop = kp * proportional;
					float abs_prop = fabsf(proportional);
					float center_boost = fminf(abs_prop, center_boost_angle);
					float accel_boost = 0;
					if(start_counter_ms) {
						pid_prop += center_boost * center_boost_kp_adder * SIGN(proportional) *
							(START_GRACE_PERIOD_MS - start_counter_ms) / START_GRACE_PERIOD_MS;
					}
					else {
						pid_prop += center_boost * center_boost_kp_adder * SIGN(proportional);

						// Acceleration boost
						if ((abs_prop > accel_boost_threshold) && !braking) {
							float boost_prop = abs_prop - accel_boost_threshold;
							accel_boost = boost_prop * kp * accel_boost_intensity;

							if (abs_prop > accel_boost_threshold2) {
								boost_prop = abs_prop - accel_boost_threshold2;
								accel_boost += boost_prop * kp * accel_boost_intensity;
							}
						}

						pid_prop += accel_boost * SIGN(proportional);
					}

					// D:
					pid_derivative = kd * derivative;
					if (fabsf(pid_derivative) > max_derivative) {
						pid_derivative = max_derivative * SIGN(pid_derivative);
					}

					// Treat P+D together
					float new_pd_value = pid_prop + pid_derivative;
					if (SIGN(erpm) != SIGN(new_pd_value)) {
						// limit P and D braking amps while slow on flat ground
						float pid_max = fmaxf(max_brake_amps, fabsf(pid_prop));
						float tt = fabs(torquetilt_interpolated);
						if (tt > 2) {
							// increase pid_max with torque tilt
							pid_max *= (0.75 + tt / 8);
						}
						if (abs_erpm > 2000) {
							// increase pid_max with erpm
							pid_max *= (0.8 + abs_erpm / 10000);
						}
						if (fabsf(new_pd_value) > pid_max) {
							new_pd_value = SIGN(new_pd_value) * pid_max;
						}
					}

					// I:
					pid_integral = ki * integral;

					// smoothen out requested current (introduce ~10ms effective latency):
					pid_value = 0.1 * (new_pd_value + pid_integral) + 0.9 * pid_value;
				}

				last_proportional = proportional;

				// For logging only:
				balance_integral = integral;
				balance_ki = ki;
				balance_setpoint = setpoint;
				balance_atr = torquetilt_target;
				balance_carve = turntilt_target;

				// Output to motor
				if (pid_value > mc_current_max) {
					pid_value = mc_current_max - 3;
					beep_on(1);
				}
				else if (pid_value < mc_current_min) {
					pid_value = mc_current_min + 3;
					beep_on(1);
				}
				else {
					beep_off(0);
				}

				if (start_counter_clicks) {
					start_counter_clicks--;
					if ((start_counter_clicks == 0) || (start_counter_clicks == 2))
						set_current(pid_value - click_current);
					else
						set_current(pid_value + click_current);
				}
				else {
					set_current(pid_value);
				}
				break;
			case (FAULT_ANGLE_PITCH):
			case (FAULT_ANGLE_ROLL):
			case (FAULT_SWITCH_HALF):
			case (FAULT_SWITCH_FULL):
			case (FAULT_STARTUP):
			case (FAULT_REVERSE):
				if (log_balance_state != FAULT_DUTY)
					log_balance_state = state;

				if (inactivity_timer == -1)
					inactivity_timer = current_time;

				if ((inactivity_timeout > 0) &&
					(ST2S(current_time - inactivity_timer) > inactivity_timeout)){

					// triple-beep
					beep_on(true);
					chThdSleepMilliseconds(200);
					beep_off(true);
					chThdSleepMilliseconds(100);
					beep_on(true);
					chThdSleepMilliseconds(200);
					beep_off(true);
					chThdSleepMilliseconds(100);
					beep_on(true);
					chThdSleepMilliseconds(200);
					beep_off(true);

					inactivity_timeout = 10; // beep again in 10 seconds
					inactivity_timer = current_time;
				}

				check_lock();

				// Check for valid startup position and switch state
				if(is_locked == false &&
				   fabsf(pitch_angle) < balance_conf.startup_pitch_tolerance &&
				   fabsf(roll_angle) < balance_conf.startup_roll_tolerance && switch_state == ON){
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

/**
 * check_lock:	perform lock management
 */
static void check_lock() {
	switch(lock_state) {
	case -1: if (switch_state == ON) lock_state = 0;
		break;
	case 0: if (switch_state == OFF) lock_state = 1;
		break;
	case 1: if (adc2 > balance_conf.fault_adc2) lock_state = -1;
		else if (adc1 > balance_conf.fault_adc1) lock_state = 2;
		break;
	case 2: if ((adc2 > balance_conf.fault_adc2) || switch_state == ON) lock_state = -1;
		else if (switch_state == OFF) lock_state = 3;
		break;
	case 3: if (adc1 > balance_conf.fault_adc1) lock_state = -1;
		else if (adc2 > balance_conf.fault_adc2) lock_state = 4;
		break;
	case 4: if ((adc1 > balance_conf.fault_adc1) || switch_state == ON) lock_state = -1;
		else if (switch_state == OFF) lock_state = 5;
		break;
	case 5: if (adc2 > balance_conf.fault_adc2) lock_state = -1;
		else if (adc1 > balance_conf.fault_adc1) lock_state = 6;
		break;
	case 6: if ((adc2 > balance_conf.fault_adc2) || switch_state == ON) lock_state = -1;
		else if (switch_state == OFF) lock_state = 7;
		break;
	case 7: if (adc1 > balance_conf.fault_adc1) lock_state = -1;
		else if (adc2 > balance_conf.fault_adc2) lock_state = 8;
		break;
	case 8:
		lock_state = -1;
		is_locked = !is_locked;				// change lock from locked to non-locked or back
		commands_balance_lock(is_locked);	// store to flash (in balance_conf.multi_esc)
		if (is_locked) {
			beep_alert(2, 1);	// beeeep-beeeep
		}
		else {
			beep_alert(3, 0);	// beep-beep-beep
		}
		break;
	default:;
	}
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
