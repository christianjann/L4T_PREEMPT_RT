/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef SH_INTERFACE_H_
#define SH_INTERFACE_H_

/* Packet payload structures */

/* Read /dev/shub_cam */
struct __attribute__ ((__packed__)) camera_payload_t {
	uint64_t	timestamp;
	uint32_t	pwm_pulse_num;
};

/* Read /dev/shub_accel */
struct  __attribute__ ((__packed__)) accel_payload_t {
	uint64_t	timestamp;
	uint32_t	pwm_pulse_num;
	uint16_t	accel[3];
};

/* Read /dev/shub_gyro */
struct  __attribute__ ((__packed__)) gyro_payload_t {
	uint64_t	timestamp;
	uint32_t	pwm_pulse_num;
	uint16_t	gyro[3];
};

/* Read /dev/shub_mag */
struct  __attribute__ ((__packed__)) mag_payload_t {
	uint64_t	timestamp;
	uint16_t	mag[3];
};

/* Read /dev/shub_baro */
struct  __attribute__ ((__packed__)) baro_payload_t {
	uint64_t	timestamp;
	uint32_t	baro;
};

/* Write or Read /dev/shub_mcu */
struct  __attribute__ ((__packed__)) mcu_payload_t {
	union {
		uint32_t	cmd;
		uint32_t	rsp;
	};
};

/* Commands from AP to sensor hub - directed to sensor hub controller */
/* Write /dev/shub_mcu */
#define CMD_PING			0x21
#define CMD_START_TS			0x22
#define CMD_STOP_TS			0x23
#define CMD_START			CMD_PING
#define CMD_END				CMD_STOP_TS

/* Responses from sensor hub to AP */
/* Read /dev/shub_mcu */
#define RSP_PING			0x21
#define RSP_START_TS			0x22
#define RSP_STOP_TS			0x23

#endif  /* SH_INTERFACE_H */
