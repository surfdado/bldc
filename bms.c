/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

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

/**
 * This is the BMS module of the VESC firmware. It mainly supports the VESC BMS, but
 * the intention is to have it extendible to other BMSs too. The first step is
 * to add the BMS you want to support to the BMS_TYPE enum, and then you need to update
 * this module to interpret CAN-messages from it properly.
 */

#include "bms.h"
#include "buffer.h"
#include "utils.h"
#include "datatypes.h"
#include "comm_can.h"
#include "commands.h"
#include "comm_usb.h"
#include <string.h>
#include <math.h>

// Settings
#define MAX_CAN_AGE_SEC				2.0

// Private variables
static volatile bms_config m_conf;
static volatile bms_values m_values;
static volatile bms_soc_soh_temp_stat m_stat_temp_max;
static volatile bms_soc_soh_temp_stat m_stat_soc_min;
static volatile bms_soc_soh_temp_stat m_stat_soc_max;

void bms_init(bms_config *conf) {
	m_conf = *conf;
	memset((void*)&m_values, 0, sizeof(m_values));
	memset((void*)&m_stat_temp_max, 0, sizeof(m_stat_temp_max));
	memset((void*)&m_stat_soc_min, 0, sizeof(m_stat_soc_min));
	memset((void*)&m_stat_soc_max, 0, sizeof(m_stat_soc_max));
	memset((void*)&m_values, 0, sizeof(m_values));
	m_values.can_id = -1;
	m_stat_temp_max.id = -1;
	m_stat_soc_min.id = -1;
	m_stat_soc_max.id = -1;
}

bool bms_process_can_frame(uint32_t can_id, uint8_t *data8, int len, bool is_ext) {
	return false;
}

void bms_update_limits(float *i_in_min, float *i_in_max,
		float i_in_min_conf, float i_in_max_conf) {
}

void bms_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {
}

bms_values *bms_get_values(void) {
	return (bms_values*)&m_values;
}
