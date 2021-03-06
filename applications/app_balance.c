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
	CENTERING = 0,
	TILTBACK
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

// Allow me to go 10km/h even below low voltage
#define TILTBACK_LOW_VOLTAGE_MIN_ERPM 3000
// Give me another 2 Volts of margin at low speeds before doing tiltback there too
#define TILTBACK_LOW_VOLTAGE_SLOW_MARGIN 2
// Audible alert at 1 Volt above tiltback voltage
#define HEADSUP_LOW_VOLTAGE_MARGIN 1

#ifdef HAS_EXT_BUZZER
static int low_voltage_headsup_done = 0;
#endif

// Balance thread
static THD_FUNCTION(balance_thread, arg);
static THD_WORKING_AREA(balance_thread_wa, 2048); // 2kb stack for this thread

static thread_t *app_thread;

// Config values
static volatile balance_config balance_conf;
static volatile imu_config imu_conf;
static float startup_step_size, tiltback_step_size;

// Runtime values read from elsewhere
static float pitch_angle, roll_angle;
static float gyro[3];
static float duty_cycle, abs_duty_cycle;
static float erpm, abs_erpm, avg_erpm;
static float motor_current;
static float motor_position;
static float adc1, adc2;
static SwitchState switch_state;

// Rumtime state values
static BalanceState state;
static float proportional, integral, derivative;
static float last_proportional;
static float pid_value;
static float setpoint, setpoint_target, setpoint_target_interpolated;
static SetpointAdjustmentType setpointAdjustmentType;
static float yaw_proportional, yaw_integral, yaw_derivative, yaw_last_proportional, yaw_pid_value, yaw_setpoint;
static systime_t current_time, last_time, diff_time;
static systime_t fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer, fault_switch_half_timer, fault_duty_timer;
static float d_pt1_state, d_pt1_k;
static float max_temp_fet;
static RideState ride_state, new_ride_state;

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
}

void app_balance_start(void) {
	// First start only, override state to startup
	state = STARTUP;
	// Start the balance thread
	app_thread = chThdCreateStatic(balance_thread_wa, sizeof(balance_thread_wa), NORMALPRIO, balance_thread, NULL);
}

void reset_vars(void){
	// Clear accumulated values.
	integral = 0;
	last_proportional = 0;
	yaw_integral = 0;
	yaw_last_proportional = 0;
	d_pt1_state = 0;
	// Set values for startup
	setpoint = pitch_angle;
	setpoint_target_interpolated = pitch_angle;
	setpoint_target = 0;
	setpointAdjustmentType = CENTERING;
	yaw_setpoint = 0;
	state = RUNNING;
	current_time = 0;
	last_time = 0;
	diff_time = 0;
	max_temp_fet = mc_interface_get_configuration()->l_temp_fet_start;
	new_ride_state = ride_state = RIDE_OFF;
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
	} else {
		fault_switch_timer = current_time;
	}

	// Switch partially open and stopped
	if(switch_state == HALF && abs_erpm < balance_conf.fault_adc_half_erpm){
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
	if(setpointAdjustmentType == CENTERING && setpoint_target_interpolated != setpoint_target){
		// Ignore tiltback during centering sequence
		state = RUNNING;
	}else if(abs_duty_cycle > balance_conf.tiltback_duty){
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
		if(balance_conf.tiltback_constant != 0 && abs_erpm > balance_conf.tiltback_constant_erpm){
			// Nose angle adjustment
			if(erpm > 0){
				setpoint_target = balance_conf.tiltback_constant;
			} else {
				setpoint_target = -balance_conf.tiltback_constant;
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
			setpoint_target_interpolated -= get_setpoint_adjustment_step_size();
		}
	}
}

float apply_deadzone(float error){
	if(balance_conf.deadzone == 0){
		return error;
	}

	if(error < balance_conf.deadzone && error > -balance_conf.deadzone){
		return 0;
	} else if(error > balance_conf.deadzone){
		return error - balance_conf.deadzone;
	} else {
		return error + balance_conf.deadzone;
	}
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
	if(balance_conf.multi_esc){
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *msg = comm_can_get_status_msg_index(i);
			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
				comm_can_set_current_brake(msg->id, balance_conf.brake_current);
			}
		}
	}
	beep_off(true);
	// we've stopped riding => turn the lights off
	// TODO: Add delay (to help spot the vehicle after a crash?)
	new_ride_state = RIDE_OFF;
	update_lights();
}

void set_current(float current, float yaw_current){
	// Reset the timeout
	timeout_reset();
	// Set current
	if(balance_conf.multi_esc){
		mc_interface_set_current(current + yaw_current);
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *msg = comm_can_get_status_msg_index(i);

			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
				comm_can_set_current(msg->id, current - yaw_current);// Assume 2 motors, i don't know how to steer 3 anyways
			}
		}
	} else {
		mc_interface_set_current(current);
	}
}

void app_balance_stop(void) {
	if(app_thread != NULL){
		chThdTerminate(app_thread);
		chThdWait(app_thread);
	}
	set_current(0, 0);
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
		imu_get_gyro(gyro);
		duty_cycle = mc_interface_get_duty_cycle_now();
		abs_duty_cycle = fabsf(duty_cycle);
		erpm = mc_interface_get_rpm();
		abs_erpm = fabsf(erpm);
		if(balance_conf.multi_esc){
			avg_erpm = erpm;
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);
				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					avg_erpm += msg->rpm;
				}
			}
			avg_erpm = avg_erpm/2;// Assume 2 motors, i don't know how to steer 3 anyways
		}
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

				// Let the rider know that the board is ready
				beep_on(1);
				chThdSleepMilliseconds(100);
				beep_off(1);

				// Issue 1 beep for each volt below 45
				double bat_volts = GET_INPUT_VOLTAGE();
				if (bat_volts < 45) {
					chThdSleepMilliseconds(400);
					while (bat_volts < 45) {
						chThdSleepMilliseconds(200);
						beep_on(1);
						chThdSleepMilliseconds(300);
						beep_off(1);
						bat_volts = bat_volts + 1;
					}
				}
				break;
			case (RUNNING):
			case (RUNNING_TILTBACK_DUTY):
			case (RUNNING_TILTBACK_HIGH_VOLTAGE):
			case (RUNNING_TILTBACK_LOW_VOLTAGE):
			case (RUNNING_TILTBACK_CONSTANT):

				// Check for faults
				if(check_faults(false)){
					break;
				}

				// Calculate setpoint and interpolation
				calculate_setpoint_target();
				calculate_setpoint_interpolated();

				update_beep_alert();

				// Apply setpoint filtering
				if(setpointAdjustmentType == CENTERING){
					// Ignore filtering during centering
					setpoint = setpoint_target_interpolated;
				}else{
					setpoint = (setpoint * (1-balance_conf.setpoint_pitch_filter)) + (pitch_angle * balance_conf.setpoint_pitch_filter);
					setpoint = (setpoint * (1-balance_conf.setpoint_target_filter)) + (setpoint_target_interpolated * balance_conf.setpoint_target_filter);
				}

				// Clamp setpoint
				if(setpointAdjustmentType != CENTERING){
					if(setpoint - setpoint_target_interpolated > balance_conf.setpoint_filter_clamp){
						setpoint = setpoint_target_interpolated + balance_conf.setpoint_filter_clamp;
					}else if (setpoint - setpoint_target_interpolated < -balance_conf.setpoint_filter_clamp){
						setpoint = setpoint_target_interpolated - balance_conf.setpoint_filter_clamp;
					}
				}

				// Do PID maths
				proportional = setpoint - pitch_angle;
				// Apply deadzone
				proportional = apply_deadzone(proportional);
				// Resume real PID maths
				integral = integral + proportional;
				derivative = proportional - last_proportional;

				// Apply D term only filter
				if(balance_conf.kd_pt1_frequency > 0){
					d_pt1_state = d_pt1_state + d_pt1_k * (derivative - d_pt1_state);
					derivative = d_pt1_state;
				}

				pid_value = (balance_conf.kp * proportional) + (balance_conf.ki * integral) + (balance_conf.kd * derivative);

				last_proportional = proportional;

				// Apply current boost
				if(pid_value > 0){
					pid_value += balance_conf.current_boost;
				}else if(pid_value < 0){
					pid_value -= balance_conf.current_boost;
				}


				if(balance_conf.multi_esc){
					// Calculate setpoint
					if(abs_duty_cycle < .02){
						yaw_setpoint = 0;
					} else if(avg_erpm < 0){
						yaw_setpoint = (-balance_conf.roll_steer_kp * roll_angle) + (balance_conf.roll_steer_erpm_kp * roll_angle * avg_erpm);
					} else{
						yaw_setpoint = (balance_conf.roll_steer_kp * roll_angle) + (balance_conf.roll_steer_erpm_kp * roll_angle * avg_erpm);
					}
					// Do PID maths
					yaw_proportional = yaw_setpoint - gyro[2];
					yaw_integral = yaw_integral + yaw_proportional;
					yaw_derivative = yaw_proportional - yaw_last_proportional;

					yaw_pid_value = (balance_conf.yaw_kp * yaw_proportional) + (balance_conf.yaw_ki * yaw_integral) + (balance_conf.yaw_kd * yaw_derivative);

					if(yaw_pid_value > balance_conf.yaw_current_clamp){
						yaw_pid_value = balance_conf.yaw_current_clamp;
					}else if(yaw_pid_value < -balance_conf.yaw_current_clamp){
						yaw_pid_value = -balance_conf.yaw_current_clamp;
					}

					yaw_last_proportional = yaw_proportional;
				}

				// Output to motor
				set_current(pid_value, yaw_pid_value);

				if (abs_erpm > balance_conf.fault_adc_half_erpm) {
					// we're at riding speed => turn on the forward facing lights
					if (pid_value > -4) {
						if (erpm > 0) {
							new_ride_state = RIDE_FORWARD;
						}
						else {
							new_ride_state = RIDE_REVERSE;
						}
					}
					else {
						if (erpm > 0) {
							new_ride_state = BRAKE_FORWARD;
						}
						else {
							new_ride_state = BRAKE_REVERSE;
						}
					}
				}
				else {
					new_ride_state = RIDE_IDLE;
				}
				if (new_ride_state != ride_state){
					update_lights();
				}
				break;
			case (FAULT_ANGLE_PITCH):
			case (FAULT_ANGLE_ROLL):
			case (FAULT_SWITCH_HALF):
			case (FAULT_SWITCH_FULL):
			case (FAULT_STARTUP):
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
				new_ride_state = RIDE_OFF;
				// We need another fault to clear duty fault.
				// Otherwise duty fault will clear itself as soon as motor pauses, then motor will spool up again.
				// Rendering this fault useless.
				check_faults(true);
				// Disable output
				brake();
				break;
		}

		// Delay between loops
		chThdSleepMicroseconds((int)((1000.0 / balance_conf.hertz) * 1000.0));
	}
	// in case we leave this force the buzzer off (force=regardless of ongoing multi beeps)
	beep_off(true);

	// Disable output
	brake();
}
