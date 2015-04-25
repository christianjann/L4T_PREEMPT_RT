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

#ifndef SH_PRIVATE_H_
#define SH_PRIVATE_H_

#include "sh_interface.h"

/*
 * SENSOR HUB FIRMWARE INTERFACE
 *
------------------------------------------------------
|            |            |            |             |
|  Start (S) |    Type    |  Payload   |   CRC32     |
|  (1-byte)  |  (1-byte)  |(0-18 bytes)|  (4-bytes)  |
|            |            |            |             |
------------------------------------------------------
*
*
*/

/* Packet start */
#define SENSOR_HUB_START		'S'

/* Packet type */
/* Messages from sensor hub to AP */
/* NOTE: This matches the enum list in client_devs_num in source file */
#define MSG_MCU				0x00 /* Read /dev/shub_mcu */
#define MSG_CAMERA			0x01 /* Read /dev/shub_cam */
#define MSG_ACCEL			0x02 /* Read /dev/shub_accel */
#define MSG_GYRO			0x03 /* Read /dev/shub_gyro */
#define MSG_MAG				0x04 /* Read /dev/shub_mag */
#define MSG_BARO			0x05 /* Read /dev/shub_baro */
#define MSG_SENSOR_START		MSG_MCU
#define MSG_SENSOR_END			MSG_BARO

/* Packet header */
struct __attribute__ ((__packed__)) sensor_hub_pkt_header_t {
	unsigned char	start;
	unsigned char	type;
};

/* Packet payload */
/* Included from sh_interface.h */

/* Biggest possible packet */
struct __attribute__ ((__packed__)) sensor_hub_pkt_t {
	struct sensor_hub_pkt_header_t header;
	union {
		struct camera_payload_t	cam_payload;
		struct accel_payload_t	accel_payload;
		struct gyro_payload_t	gyro_payload;
		struct mag_payload_t	mag_payload;
		struct baro_payload_t	baro_payload;
		struct mcu_payload_t	mcu_payload;
	} payload;
	uint32_t crc32;
};

#endif  /* SH_PRIVATE_H */
