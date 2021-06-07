/*
	Copyright 2020 - 2021 Benjamin Vedder	benjamin@vedder.se

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

#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include "app_finn_types.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * HW Connections:
 *
 * btn_left: UART_RX and VCC
 * btn_right: ADC_EXT and VCC
 * limit_sw: ADC_EXT2 and VCC
 *
 * Motor:
 * blue: A
 * yellow: B
 * red: C
 *
 * Motor sensor connector should be plugged in.
 *
 */

// Settings
#define FILTER_CONST			0.02 // Range: 0 - 1. Higher values make the control more aggressive.
#define BUTTON_RATE				5.0 // How fast the home adjustment buttons move the pod. Unit: Deg/s
#define START_DELAY				10.0 // Start delay in seconds

/**
 * Homing procedure
 *
 * The pod will rotate left, stay there for left_time, then rotate right
 * until the limit switch is found or until HOMING_ANGLE_MAX is reached. If
 * HOMING_ANGLE_MAX an error is thrown and the pod will rotate back to the initial
 * position. If a homing error occurs, no new commands are accepted.
 */
#define HOMING_RATE				20.0 // Deg/s
#define HOMING_ANGLE_BACK		-30.0
#define HOMING_ANGLE_MAX		270.0
#define HOMING_BACK_TIME		3.0 // Seconds

#define P_ADDR_OFFSET			0

// Threads
static THD_FUNCTION(control_thread, arg);
static THD_WORKING_AREA(control_thread_wa, 1024);

static THD_FUNCTION(status_thread, arg);
static THD_WORKING_AREA(status_thread_wa, 1024);
static thread_t *status_tp;

// Private variables
static volatile bool stop_now = true;
static volatile bool control_is_running = false;
static volatile bool status_is_running = false;
static volatile POD_STATE m_pod_state;

// Private functions
static bool can_eid_callback(uint32_t id, uint8_t *data, uint8_t len);
static void terminal_mon(int argc, const char **argv);
static void terminal_home(int argc, const char **argv);

void app_custom_start(void) {
}

void app_custom_stop(void) {
}

void app_custom_configure(app_configuration *conf) {
	m_pod_state.pod_id = conf->controller_id;
}

/*
 * Here we handle CAN-frames from the joystick.
 */
static bool can_eid_callback(uint32_t id, uint8_t *data, uint8_t len) {
	bool res = false;

	uint32_t sourceType = (id >> 0) & 0xFF;
	uint32_t sourceIndex = (id >> 8) & 0xFF;
	uint32_t packetType = (id >> 16) & 0xFF;
	uint32_t prio = (id >> 24) & 0x1F;

//	commands_printf("sourceType: %02X sourceIndex: %02X, packetType: %02X, prio: %02X",
//			sourceType, sourceIndex, packetType, prio);

	(void)sourceType;
	(void)sourceIndex;
	(void)prio;

	CAN_PAYLOAD_UNION packet_union;
	memcpy(packet_union.bytes, data, len);

	switch(packetType)
	{
	case CAN_MESSAGE_POD_REQUEST:
	{
		CAN_PACKET_POD_REQUEST *pkt = &packet_union.podRequest;
		uint8_t pktpodid = pkt->pod_id;

		if (m_pod_state.pod_id == pktpodid) {// 0-7
			int16_t ang_int = pkt->req_angle;

			// Ignore angles outside of range
			if (ang_int > 5100.0 || ang_int < -5100.0) {
				res = false;
				break;
			}

			m_pod_state.req_angle = (float)ang_int / 5000.0 * 90.0;
			m_pod_state.last_update = chVTGetSystemTimeX();
		}

		res = true;
		break;
	}

	case CAN_MESSAGE_POD_REQ_FOUR:
	{
		CAN_PACKET_POD_REQ_FOUR *pkt = &packet_union.podReqFour;

		if (m_pod_state.pod_id < 4) {
			int16_t ang_int = pkt->req_angle[m_pod_state.pod_id];

			// Ignore angles outside of range
			if (ang_int > 5100.0 || ang_int < -5100.0) {
				res = false;
				break;
			}

			m_pod_state.req_angle = (float)ang_int / 5000.0 * 90.0;
			m_pod_state.last_update = chVTGetSystemTimeX();
		}

		res = true;
		break;
	}

	case CAN_MESSAGE_POD_REQ_FOUR_HI:
	{
		CAN_PACKET_POD_REQ_FOUR *pkt = &packet_union.podReqFour;
		if (m_pod_state.pod_id >= 4 && m_pod_state.pod_id < 8) {
			int16_t ang_int = pkt->req_angle[m_pod_state.pod_id-4];

			// Ignore angles outside of range
			if (ang_int > 5100.0 || ang_int < -5100.0) {
				res = false;
				break;
			}

			m_pod_state.req_angle = (float)ang_int / 5000.0 * 90.0;
			m_pod_state.last_update = chVTGetSystemTimeX();
		}

		res = true;
		break;
	}

	default:
		break;
	}

	if (res) {
		// Trigger status thread to send update
		chEvtSignal(status_tp, (eventmask_t) 1);
	}

	return res;
}

static THD_FUNCTION(control_thread, arg) {
	(void)arg;

	chRegSetThreadName("Finn AZ");
}

static THD_FUNCTION(status_thread, arg) {
	(void)arg;

	chRegSetThreadName("Finn AZ stat");
}

static void terminal_home(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	m_pod_state.homing_angle_now = HOMING_ANGLE_BACK + m_pod_state.req_angle + m_pod_state.angle_home + m_pod_state.angle_offset;
	m_pod_state.homing_back_time = 0.0;
	m_pod_state.homing_done = false;
	m_pod_state.homing_error = false;

	commands_printf("OK");
}

static void terminal_mon(int argc, const char **argv) {
}
