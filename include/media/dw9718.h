/*
 * Copyright (c) 2010-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __DW9718_H__
#define __DW9718_H__

#include <linux/miscdevice.h>
#include <media/nvc_focus.h>
#include <media/nvc.h>

struct dw9718_power_rail {
	struct regulator *vdd;
	struct regulator *vdd_i2c;
	struct regulator *vana;
};

struct dw9718_platform_data {
	int cfg;
	int num;
	int sync;
	const char *dev_name;
	struct nvc_focus_nvc (*nvc);
	struct nvc_focus_cap (*cap);
	int gpio_count;
	struct nvc_gpio_pdata *gpio;
	int (*power_on)(struct dw9718_power_rail *pw);
	int (*power_off)(struct dw9718_power_rail *pw);
	int (*detect)(void *buf, size_t size);
};

/* Register Definitions */
#define DW9718_POWER_DN		0x00
#define DW9718_CONTROL			0x01
#define DW9718_VCM_CODE_MSB		0x02
#define DW9718_VCM_CODE_LSB		0x03
#define DW9718_SWITCH_MODE		0x04
#define DW9718_SACT			0x05
#define DW9718_STATUS			0x06

#endif  /* __DW9718_H__ */
