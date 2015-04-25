/*
 * ar0330.c - ar0330 sensor driver
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <media/ar0330.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "nvc_utilities.h"

struct ar0330_reg {
	u16 addr;
	u16 val;
};

struct ar0330_info {
	struct miscdevice		miscdev_info;
	int				mode;
	struct ar0330_power_rail	power;
	struct ar0330_sensordata	sensor_data;
	struct i2c_client		*i2c_client;
	struct ar0330_platform_data	*pdata;
	struct clk			*mclk;
	struct regmap			*regmap;
	struct mutex			ar0330_camera_lock;
	atomic_t			in_use;
	char devname[16];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
};

#define AR0330_TABLE_WAIT_MS 0
#define AR0330_TABLE_END 1
#define AR0330_MAX_RETRIES 3
#define AR0330_WAIT_MS 100

#define MAX_BUFFER_SIZE 32
#define AR0330_FRAME_LENGTH_ADDR 0x300A
#define AR0330_COARSE_TIME_ADDR 0x3012
#define AR0330_GAIN_ADDR 0x3060

static struct ar0330_reg mode_2304x1536[] = {
	{0x301A, 0x0059},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x31AE, 0x0204},
	{0x301A, 0x0059},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x301A, 0x0058},
	{0x3064, 0x1802},
	{0x3078, 0x0001},
	{0x30BA, 0x002C},
	{0x30FE, 0x0080},
	{0x31E0, 0x0003},
	{0x3ECE, 0x09FF},
	{0x3ED0, 0xE4F6},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x889B},
	{0x3EDC, 0x8863},
	{0x3EDE, 0xAA04},
	{0x3EE0, 0x15F0},
	{0x3EE6, 0x008C},
	{0x3EE8, 0x2024},
	{0x3EEA, 0xFF1F},
	{0x3F06, 0x046A},
	{0x3046, 0x4038},
	{0x3048, 0x8480},
	{0x31E0, 0x0003},
	{0x301A, 0x0058},
	{0x31AE, 0x0202},
	{0x31AC, 0x0A0A},
	{0x31B0, 0x0028},
	{0x31B2, 0x000E},
	{0x31B4, 0x2743},
	{0x31B6, 0x114E},
	{0x31B8, 0x2049},
	{0x31BA, 0x0186},
	{0x31BC, 0x8005},
	{0x31BE, 0x2003},
	{0x302A, 0x0005},
	{0x302C, 0x0002},
	{0x302E, 0x0002},
	{0x3030, 0x0031},
	{0x3036, 0x000A},
	{0x3038, 0x0001},
	{0x31AC, 0x0A0A},
	{0x3004, 0x0000},
	{0x3008, 0x08FF},
	{0x3002, 0x0000},
	{0x3006, 0x05FF},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000},
	{0x300C, 0x04E0},
	{0x300A, 0x0622},
	{0x3014, 0x0000},
	{0x3012, 0x0621},
	{0x3042, 0x0000},
	{0x30BA, 0x002C},
	{0x301A, 0x0058},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x3088, 0x8000},
	{0x3086, 0x4A03},
	{0x3086, 0x4316},
	{0x3086, 0x0443},
	{0x3086, 0x1645},
	{0x3086, 0x4045},
	{0x3086, 0x6017},
	{0x3086, 0x2045},
	{0x3086, 0x404B},
	{0x3086, 0x1244},
	{0x3086, 0x6134},
	{0x3086, 0x4A31},
	{0x3086, 0x4342},
	{0x3086, 0x4560},
	{0x3086, 0x2714},
	{0x3086, 0x3DFF},
	{0x3086, 0x3DFF},
	{0x3086, 0x3DEA},
	{0x3086, 0x2704},
	{0x3086, 0x3D10},
	{0x3086, 0x2705},
	{0x3086, 0x3D10},
	{0x3086, 0x2715},
	{0x3086, 0x3527},
	{0x3086, 0x053D},
	{0x3086, 0x1045},
	{0x3086, 0x4027},
	{0x3086, 0x0427},
	{0x3086, 0x143D},
	{0x3086, 0xFF3D},
	{0x3086, 0xFF3D},
	{0x3086, 0xEA62},
	{0x3086, 0x2728},
	{0x3086, 0x3627},
	{0x3086, 0x083D},
	{0x3086, 0x6444},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C2C},
	{0x3086, 0x4B01},
	{0x3086, 0x432D},
	{0x3086, 0x4643},
	{0x3086, 0x1647},
	{0x3086, 0x435F},
	{0x3086, 0x4F50},
	{0x3086, 0x2604},
	{0x3086, 0x2684},
	{0x3086, 0x2027},
	{0x3086, 0xFC53},
	{0x3086, 0x0D5C},
	{0x3086, 0x0D57},
	{0x3086, 0x5417},
	{0x3086, 0x0955},
	{0x3086, 0x5649},
	{0x3086, 0x5307},
	{0x3086, 0x5302},
	{0x3086, 0x4D28},
	{0x3086, 0x6C4C},
	{0x3086, 0x0928},
	{0x3086, 0x2C28},
	{0x3086, 0x294E},
	{0x3086, 0x5C09},
	{0x3086, 0x6045},
	{0x3086, 0x0045},
	{0x3086, 0x8026},
	{0x3086, 0xA627},
	{0x3086, 0xF817},
	{0x3086, 0x0227},
	{0x3086, 0xFA5C},
	{0x3086, 0x0B17},
	{0x3086, 0x1826},
	{0x3086, 0xA25C},
	{0x3086, 0x0317},
	{0x3086, 0x4427},
	{0x3086, 0xF25F},
	{0x3086, 0x2809},
	{0x3086, 0x1714},
	{0x3086, 0x2808},
	{0x3086, 0x1701},
	{0x3086, 0x4D1A},
	{0x3086, 0x2683},
	{0x3086, 0x1701},
	{0x3086, 0x27FA},
	{0x3086, 0x45A0},
	{0x3086, 0x1707},
	{0x3086, 0x27FB},
	{0x3086, 0x1729},
	{0x3086, 0x4580},
	{0x3086, 0x1708},
	{0x3086, 0x27FA},
	{0x3086, 0x1728},
	{0x3086, 0x5D17},
	{0x3086, 0x0E26},
	{0x3086, 0x8153},
	{0x3086, 0x0117},
	{0x3086, 0xE653},
	{0x3086, 0x0217},
	{0x3086, 0x1026},
	{0x3086, 0x8326},
	{0x3086, 0x8248},
	{0x3086, 0x4D4E},
	{0x3086, 0x2809},
	{0x3086, 0x4C0B},
	{0x3086, 0x6017},
	{0x3086, 0x2027},
	{0x3086, 0xF217},
	{0x3086, 0x535F},
	{0x3086, 0x2808},
	{0x3086, 0x164D},
	{0x3086, 0x1A17},
	{0x3086, 0x0127},
	{0x3086, 0xFA26},
	{0x3086, 0x035C},
	{0x3086, 0x0145},
	{0x3086, 0x4027},
	{0x3086, 0x9817},
	{0x3086, 0x2A4A},
	{0x3086, 0x0A43},
	{0x3086, 0x160B},
	{0x3086, 0x4327},
	{0x3086, 0x9C45},
	{0x3086, 0x6017},
	{0x3086, 0x0727},
	{0x3086, 0x9D17},
	{0x3086, 0x2545},
	{0x3086, 0x4017},
	{0x3086, 0x0827},
	{0x3086, 0x985D},
	{0x3086, 0x2645},
	{0x3086, 0x4B17},
	{0x3086, 0x0A28},
	{0x3086, 0x0853},
	{0x3086, 0x0D52},
	{0x3086, 0x5112},
	{0x3086, 0x4460},
	{0x3086, 0x184A},
	{0x3086, 0x0343},
	{0x3086, 0x1604},
	{0x3086, 0x4316},
	{0x3086, 0x5843},
	{0x3086, 0x1659},
	{0x3086, 0x4316},
	{0x3086, 0x5A43},
	{0x3086, 0x165B},
	{0x3086, 0x4327},
	{0x3086, 0x9C45},
	{0x3086, 0x6017},
	{0x3086, 0x0727},
	{0x3086, 0x9D17},
	{0x3086, 0x2545},
	{0x3086, 0x4017},
	{0x3086, 0x1027},
	{0x3086, 0x9817},
	{0x3086, 0x2022},
	{0x3086, 0x4B12},
	{0x3086, 0x442C},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C00},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	/* stream on */
	{0x301A, 0x005C},
	{AR0330_TABLE_END, 0x00}
};

static struct ar0330_reg mode_1280x720[] = {
	{0x301A, 0x0059},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x31AE, 0x0204},
	{0x301A, 0x0059},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x301A, 0x0058},
	{0x3064, 0x1802},
	{0x3078, 0x0001},
	{0x30BA, 0x002C},
	{0x30FE, 0x0080},
	{0x31E0, 0x0003},
	{0x3ECE, 0x09FF},
	{0x3ED0, 0xE4F6},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x889B},
	{0x3EDC, 0x8863},
	{0x3EDE, 0xAA04},
	{0x3EE0, 0x15F0},
	{0x3EE6, 0x008C},
	{0x3EE8, 0x2024},
	{0x3EEA, 0xFF1F},
	{0x3F06, 0x046A},
	{0x3046, 0x4038},
	{0x3048, 0x8480},
	{0x31E0, 0x0003},
	{0x301A, 0x0058},
	{0x31AE, 0x0201},
	{0x31AC, 0x0A0A},
	{0x31B0, 0x0028},
	{0x31B2, 0x000E},
	{0x31B4, 0x2743},
	{0x31B6, 0x114E},
	{0x31B8, 0x2049},
	{0x31BA, 0x0186},
	{0x31BC, 0x8005},
	{0x31BE, 0x2003},
	{0x302A, 0x0005},
	{0x302C, 0x0004},
	{0x302E, 0x0004},
	{0x3030, 0x0052},
	{0x3036, 0x000A},
	{0x3038, 0x0001},
	{0x31AC, 0x0A0A},
	{0x3004, 0x0200},
	{0x3008, 0x06FF},
	{0x3002, 0x019C},
	{0x3006, 0x046B},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000},
	{0x300C, 0x03F6},
	{0x300A, 0x0328},
	{0x3014, 0x0000},
	{0x3012, 0x0327},
	{0x3042, 0x02B0},
	{0x30BA, 0x002C},
	{0x301A, 0x0058},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x3088, 0x8000},
	{0x3086, 0x4A03},
	{0x3086, 0x4316},
	{0x3086, 0x0443},
	{0x3086, 0x1645},
	{0x3086, 0x4045},
	{0x3086, 0x6017},
	{0x3086, 0x2045},
	{0x3086, 0x404B},
	{0x3086, 0x1244},
	{0x3086, 0x6134},
	{0x3086, 0x4A31},
	{0x3086, 0x4342},
	{0x3086, 0x4560},
	{0x3086, 0x2714},
	{0x3086, 0x3DFF},
	{0x3086, 0x3DFF},
	{0x3086, 0x3DEA},
	{0x3086, 0x2704},
	{0x3086, 0x3D10},
	{0x3086, 0x2705},
	{0x3086, 0x3D10},
	{0x3086, 0x2715},
	{0x3086, 0x3527},
	{0x3086, 0x053D},
	{0x3086, 0x1045},
	{0x3086, 0x4027},
	{0x3086, 0x0427},
	{0x3086, 0x143D},
	{0x3086, 0xFF3D},
	{0x3086, 0xFF3D},
	{0x3086, 0xEA62},
	{0x3086, 0x2728},
	{0x3086, 0x3627},
	{0x3086, 0x083D},
	{0x3086, 0x6444},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C2C},
	{0x3086, 0x4B01},
	{0x3086, 0x432D},
	{0x3086, 0x4643},
	{0x3086, 0x1647},
	{0x3086, 0x435F},
	{0x3086, 0x4F50},
	{0x3086, 0x2604},
	{0x3086, 0x2684},
	{0x3086, 0x2027},
	{0x3086, 0xFC53},
	{0x3086, 0x0D5C},
	{0x3086, 0x0D57},
	{0x3086, 0x5417},
	{0x3086, 0x0955},
	{0x3086, 0x5649},
	{0x3086, 0x5307},
	{0x3086, 0x5302},
	{0x3086, 0x4D28},
	{0x3086, 0x6C4C},
	{0x3086, 0x0928},
	{0x3086, 0x2C28},
	{0x3086, 0x294E},
	{0x3086, 0x5C09},
	{0x3086, 0x6045},
	{0x3086, 0x0045},
	{0x3086, 0x8026},
	{0x3086, 0xA627},
	{0x3086, 0xF817},
	{0x3086, 0x0227},
	{0x3086, 0xFA5C},
	{0x3086, 0x0B17},
	{0x3086, 0x1826},
	{0x3086, 0xA25C},
	{0x3086, 0x0317},
	{0x3086, 0x4427},
	{0x3086, 0xF25F},
	{0x3086, 0x2809},
	{0x3086, 0x1714},
	{0x3086, 0x2808},
	{0x3086, 0x1701},
	{0x3086, 0x4D1A},
	{0x3086, 0x2683},
	{0x3086, 0x1701},
	{0x3086, 0x27FA},
	{0x3086, 0x45A0},
	{0x3086, 0x1707},
	{0x3086, 0x27FB},
	{0x3086, 0x1729},
	{0x3086, 0x4580},
	{0x3086, 0x1708},
	{0x3086, 0x27FA},
	{0x3086, 0x1728},
	{0x3086, 0x5D17},
	{0x3086, 0x0E26},
	{0x3086, 0x8153},
	{0x3086, 0x0117},
	{0x3086, 0xE653},
	{0x3086, 0x0217},
	{0x3086, 0x1026},
	{0x3086, 0x8326},
	{0x3086, 0x8248},
	{0x3086, 0x4D4E},
	{0x3086, 0x2809},
	{0x3086, 0x4C0B},
	{0x3086, 0x6017},
	{0x3086, 0x2027},
	{0x3086, 0xF217},
	{0x3086, 0x535F},
	{0x3086, 0x2808},
	{0x3086, 0x164D},
	{0x3086, 0x1A17},
	{0x3086, 0x0127},
	{0x3086, 0xFA26},
	{0x3086, 0x035C},
	{0x3086, 0x0145},
	{0x3086, 0x4027},
	{0x3086, 0x9817},
	{0x3086, 0x2A4A},
	{0x3086, 0x0A43},
	{0x3086, 0x160B},
	{0x3086, 0x4327},
	{0x3086, 0x9C45},
	{0x3086, 0x6017},
	{0x3086, 0x0727},
	{0x3086, 0x9D17},
	{0x3086, 0x2545},
	{0x3086, 0x4017},
	{0x3086, 0x0827},
	{0x3086, 0x985D},
	{0x3086, 0x2645},
	{0x3086, 0x4B17},
	{0x3086, 0x0A28},
	{0x3086, 0x0853},
	{0x3086, 0x0D52},
	{0x3086, 0x5112},
	{0x3086, 0x4460},
	{0x3086, 0x184A},
	{0x3086, 0x0343},
	{0x3086, 0x1604},
	{0x3086, 0x4316},
	{0x3086, 0x5843},
	{0x3086, 0x1659},
	{0x3086, 0x4316},
	{0x3086, 0x5A43},
	{0x3086, 0x165B},
	{0x3086, 0x4327},
	{0x3086, 0x9C45},
	{0x3086, 0x6017},
	{0x3086, 0x0727},
	{0x3086, 0x9D17},
	{0x3086, 0x2545},
	{0x3086, 0x4017},
	{0x3086, 0x1027},
	{0x3086, 0x9817},
	{0x3086, 0x2022},
	{0x3086, 0x4B12},
	{0x3086, 0x442C},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C00},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	{0x3086, 0x0000},
	/* stream on */
	{0x301A, 0x0058},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x3088, 0x80BA},
	{0x3086, 0x0253},
	{0x30CE, 0x0010},
	{0x301A, 0x015C},
	{AR0330_TABLE_END, 0x00}
};

static struct ar0330_reg mode_1280x960[] = {
	{0x3052, 0xa114},
	{0x304A, 0x0070},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x301A, 0x0058},
	{0x302A, 0x0005},
	{0x302C, 0x0004},
	{0x302E, 0x0003},
	{0x3030, 0x005F},
	{0x3036, 0x000A},
	{0x3038, 0x0001},
	{0x31AC, 0x0A0A},
	{0x31AE, 0x0201},
	{0x31B0, 0x003D},
	{0x31B2, 0x0018},
	{0x31B4, 0x4F56},
	{0x31B6, 0x4214},
	{0x31B8, 0x308B},
	{0x31BA, 0x028A},
	{0x31BC, 0x8008},
	{0x3002, 0x0126},
	{0x3004, 0x0206},
	{0x3006, 0x04E5},
	{0x3008, 0x0705},
	{0x300A, 0x0449},
	{0x300C, 0x0482},
	{0x3012, 0x0448},
	{0x3014, 0x0000},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000},
	{0x3042, 0x0000},
	{0x30BA, 0x006C},
	{0x31E0, 0x0303},
	{0x3064, 0x1802},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},
	{0x3088, 0x80BA},
	{0x3086, 0x0253},
	{0x30CE, 0x0010},
	{0x301A, 0x035C},
	{AR0330_TABLE_END, 0x00}
};

enum {
	AR0330_MODE_2304X1536,
	AR0330_MODE_1280X720,
	AR0330_MODE_1280X960,
};

static struct ar0330_reg *mode_table[] = {
	[AR0330_MODE_2304X1536] = mode_2304x1536,
	[AR0330_MODE_1280X720] = mode_1280x720,
	[AR0330_MODE_1280X960] = mode_1280x960,
};

static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static inline void
ar0330_get_frame_length_regs(struct ar0330_reg *regs, u32 frame_length)
{
	regs->addr = AR0330_FRAME_LENGTH_ADDR;
	regs->val = frame_length & 0xffff;
}

static inline void
ar0330_get_coarse_time_regs(struct ar0330_reg *regs, u32 coarse_time)
{
	regs->addr = AR0330_COARSE_TIME_ADDR;
	regs->val = coarse_time & 0xffff;
}

static inline void
ar0330_get_gain_reg(struct ar0330_reg *regs, u16 gain)
{
	regs->addr = AR0330_GAIN_ADDR;
	regs->val = gain;
}

static inline int
ar0330_read_reg(struct ar0330_info *info, u16 addr, u16 *val)
{
	return regmap_read(info->regmap, addr, (unsigned int *) val);
}

static int
ar0330_write_reg(struct ar0330_info *info, u16 addr, u16 val)
{
	int err;

	err = regmap_write(info->regmap, addr, val);

	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int
ar0330_write_table(struct ar0330_info *info,
				 const struct ar0330_reg table[],
				 const struct ar0330_reg override_list[],
				 int num_override_regs)
{
	int err;
	const struct ar0330_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != AR0330_TABLE_END; next++) {
		if (next->addr == AR0330_TABLE_WAIT_MS) {
			msleep_range(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = ar0330_write_reg(info, next->addr, val);
		if (err) {
			pr_err("%s:ar0330_write_table:%d", __func__, err);
			return err;
		}
	}
	return 0;
}

static int ar0330_get_flash_cap(struct ar0330_info *info)
{
	struct ar0330_flash_control *fctl;

	dev_dbg(&info->i2c_client->dev, "%s: %p\n", __func__, info->pdata);
	if (info->pdata) {
		fctl = &info->pdata->flash_cap;
		dev_dbg(&info->i2c_client->dev,
			"edg: %x, st: %x, rpt: %x, dl: %x\n",
			fctl->edge_trig_en,
			fctl->start_edge,
			fctl->repeat,
			fctl->delay_frm);

		if (fctl->enable)
			return 0;
	}
	return -ENODEV;
}

static inline int ar0330_set_flash_control(
	struct ar0330_info *info, struct ar0330_flash_control *fc)
{
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	return 0;
}

static int
ar0330_set_mode(struct ar0330_info *info, struct ar0330_mode *mode)
{
	int sensor_mode;
	int err;
	struct ar0330_reg reg_list[8];

	pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
			 __func__, mode->xres, mode->yres, mode->frame_length,
			 mode->coarse_time, mode->gain);

	if (mode->xres == 2304 && mode->yres == 1536) {
		sensor_mode = AR0330_MODE_2304X1536;
	} else if (mode->xres == 1280 && mode->yres == 720) {
		sensor_mode = AR0330_MODE_1280X720;
	} else if (mode->xres == 1280 && mode->yres == 960) {
		sensor_mode = AR0330_MODE_1280X960;
	} else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
			 __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	ar0330_get_frame_length_regs(reg_list, mode->frame_length);
	ar0330_get_coarse_time_regs(reg_list + 1, mode->coarse_time);
	ar0330_get_gain_reg(reg_list + 2, mode->gain);

	err = ar0330_write_table(info,
				mode_table[sensor_mode],
				reg_list, 3);
	if (err)
		return err;
	info->mode = sensor_mode;
	pr_info("[AR0330]: stream on.\n");
	return 0;
}

static int
ar0330_get_status(struct ar0330_info *info, u8 *dev_status)
{
	*dev_status = 0;
	return 0;
}

static int
ar0330_set_frame_length(struct ar0330_info *info, u32 frame_length,
						 bool group_hold)
{
	struct ar0330_reg reg_list[2];
	int i = 0;
	int ret;

	ar0330_get_frame_length_regs(reg_list, frame_length);

	for (i = 0; i < 1; i++) {
		ret = ar0330_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int
ar0330_set_coarse_time(struct ar0330_info *info, u32 coarse_time,
						 bool group_hold)
{
	int ret;

	struct ar0330_reg reg_list[2];
	int i = 0;

	ar0330_get_coarse_time_regs(reg_list, coarse_time);

	for (i = 0; i < 1; i++) {
		ret = ar0330_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int
ar0330_set_gain(struct ar0330_info *info, u16 gain, bool group_hold)
{
	int ret;
	struct ar0330_reg reg_list;

	ar0330_get_gain_reg(&reg_list, gain);

	ret = ar0330_write_reg(info, reg_list.addr, reg_list.val);
	if (ret)
		return ret;

	return 0;
}

static int
ar0330_set_group_hold(struct ar0330_info *info, struct ar0330_ae *ae)
{
	int count = 0;
	bool group_hold_enabled = false;

	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		group_hold_enabled = true;

	if (ae->gain_enable)
		ar0330_set_gain(info, ae->gain, false);
	if (ae->coarse_time_enable)
		ar0330_set_coarse_time(info, ae->coarse_time, false);
	if (ae->frame_length_enable)
		ar0330_set_frame_length(info, ae->frame_length, false);

	return 0;
}

static int ar0330_get_sensor_id(struct ar0330_info *info)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	if (info->sensor_data.fuse_id_size)
		return 0;

	/* Note 1: If the sensor does not have power at this point
	Need to supply the power, e.g. by calling power on function */

	/*ret |= ar0330_write_reg(info, 0x3B02, 0x00);
	ret |= ar0330_write_reg(info, 0x3B00, 0x01);
	for (i = 0; i < 9; i++) {
		ret |= ar0330_read_reg(info, 0x3B24 + i, &bak);
		info->sensor_data.fuse_id[i] = bak;
	}

	if (!ret)
		info->sensor_data.fuse_id_size = i;*/

	/* Note 2: Need to clean up any action carried out in Note 1 */

	return ret;
}

static void ar0330_mclk_disable(struct ar0330_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int ar0330_mclk_enable(struct ar0330_info *info)
{
	int err;
	unsigned long mclk_init_rate = 24000000;

	dev_dbg(&info->i2c_client->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);

	err = clk_set_rate(info->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(info->mclk);
	return err;
}

static long
ar0330_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct ar0330_info *info = file->private_data;

	switch (cmd) {
	case AR0330_IOCTL_SET_POWER:
		if (!info->pdata)
			break;
		if (arg && info->pdata->power_on) {
			err = ar0330_mclk_enable(info);
			if (!err)
				err = info->pdata->power_on(&info->power);
			if (err < 0)
				ar0330_mclk_disable(info);
		}
		if (!arg && info->pdata->power_off) {
			info->pdata->power_off(&info->power);
			ar0330_mclk_disable(info);
		}
		break;
	case AR0330_IOCTL_SET_MODE:
	{
		struct ar0330_mode mode;
		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(struct ar0330_mode))) {
			pr_err("%s:Failed to get mode from user.\n", __func__);
			return -EFAULT;
		}
		return ar0330_set_mode(info, &mode);
	}
	case AR0330_IOCTL_SET_FRAME_LENGTH:
		return ar0330_set_frame_length(info, (u32)arg, true);
	case AR0330_IOCTL_SET_COARSE_TIME:
		return ar0330_set_coarse_time(info, (u32)arg, true);
	case AR0330_IOCTL_SET_GAIN:
		return ar0330_set_gain(info, (u16)arg, true);
	case AR0330_IOCTL_GET_STATUS:
	{
		u8 status;

		err = ar0330_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 1)) {
			pr_err("%s:Failed to copy status to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	case AR0330_IOCTL_GET_SENSORDATA:
	{
		err = ar0330_get_sensor_id(info);

		if (err) {
			pr_err("%s:Failed to get fuse id info.\n", __func__);
			return err;
		}
		if (copy_to_user((void __user *)arg, &info->sensor_data,
				sizeof(struct ar0330_sensordata))) {
			pr_info("%s:Failed to copy fuse id to user space\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
	case AR0330_IOCTL_SET_GROUP_HOLD:
	{
		struct ar0330_ae ae;
		if (copy_from_user(&ae, (const void __user *)arg,
			sizeof(struct ar0330_ae))) {
			pr_info("%s:fail group hold\n", __func__);
			return -EFAULT;
		}
		return ar0330_set_group_hold(info, &ae);
	}
	case AR0330_IOCTL_SET_FLASH_MODE:
	{
		struct ar0330_flash_control values;

		dev_dbg(&info->i2c_client->dev,
			"AR0330_IOCTL_SET_FLASH_MODE\n");
		if (copy_from_user(&values,
			(const void __user *)arg,
			sizeof(struct ar0330_flash_control))) {
			err = -EFAULT;
			break;
		}
		err = ar0330_set_flash_control(info, &values);
		break;
	}
	case AR0330_IOCTL_GET_FLASH_CAP:
		err = ar0330_get_flash_cap(info);
		break;
	default:
		pr_err("%s:unknown cmd.\n", __func__);
		err = -EINVAL;
	}

	return err;
}

static int ar0330_power_on(struct ar0330_power_rail *pw)
{
	int err;
	struct ar0330_info *info = container_of(pw, struct ar0330_info, power);

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd || !pw->dvdd)))
		return -EFAULT;

	gpio_set_value(info->pdata->cam2_gpio, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto ar0330_avdd_fail;

	err = regulator_enable(pw->dvdd);
	if (err)
		goto ar0330_dvdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto ar0330_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(info->pdata->cam2_gpio, 1);

	usleep_range(300, 310);

	return 1;


ar0330_iovdd_fail:
	regulator_disable(pw->dvdd);

ar0330_dvdd_fail:
	regulator_disable(pw->avdd);

ar0330_avdd_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int ar0330_power_off(struct ar0330_power_rail *pw)
{
	struct ar0330_info *info = container_of(pw, struct ar0330_info, power);

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd || !pw->dvdd)))
		return -EFAULT;

	usleep_range(1, 2);
	gpio_set_value(info->pdata->cam2_gpio, 0);
	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);

	return 0;
}

static int
ar0330_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*miscdev = file->private_data;
	struct ar0330_info *info;

	info = container_of(miscdev, struct ar0330_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		pr_info("%s:BUSY!\n", __func__);
		return -EBUSY;
	}

	file->private_data = info;

	return 0;
}

static int
ar0330_release(struct inode *inode, struct file *file)
{
	struct ar0330_info *info = file->private_data;

	file->private_data = NULL;

	/* warn if device is already released */
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}

static int ar0330_power_put(struct ar0330_power_rail *pw)
{
	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;
	pw->dvdd = NULL;

	return 0;
}

static int ar0330_regulator_get(struct ar0330_info *info,
	struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = regulator_get(&info->i2c_client->dev, vreg_name);
	if (unlikely(IS_ERR(reg))) {
		dev_err(&info->i2c_client->dev, "%s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&info->i2c_client->dev, "%s: %s\n",
			__func__, vreg_name);

	*vreg = reg;
	return err;
}

static int ar0330_power_get(struct ar0330_info *info)
{
	struct ar0330_power_rail *pw = &info->power;
	int err = 0;

	err |= ar0330_regulator_get(info, &pw->avdd, "vana"); /* ananlog 2.7v */
	err |= ar0330_regulator_get(info, &pw->dvdd, "vdig"); /* digital 1.2v */
	err |= ar0330_regulator_get(info, &pw->iovdd, "vif"); /* IO 1.8v */

	return err;
}

static const struct file_operations ar0330_fileops = {
	.owner = THIS_MODULE,
	.open = ar0330_open,
	.unlocked_ioctl = ar0330_ioctl,
	.release = ar0330_release,
};

static struct miscdevice ar0330_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ar0330",
	.fops = &ar0330_fileops,
};

static struct of_device_id ar0330_of_match[] = {
	{ .compatible = "nvidia,ar0330", },
	{ },
};

MODULE_DEVICE_TABLE(of, ar0330_of_match);

static struct ar0330_platform_data *ar0330_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct ar0330_platform_data *board_info_pdata;
	const struct of_device_id *match;

	match = of_match_device(ar0330_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_info_pdata = devm_kzalloc(&client->dev, sizeof(*board_info_pdata),
			GFP_KERNEL);
	if (!board_info_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	board_info_pdata->cam2_gpio = of_get_named_gpio(np, "cam1-gpios", 0);
	board_info_pdata->ext_reg = of_property_read_bool(np, "nvidia,ext_reg");

	board_info_pdata->power_on = ar0330_power_on;
	board_info_pdata->power_off = ar0330_power_off;

	return board_info_pdata;
}

static int
ar0330_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ar0330_info *info;
	int err;
	const char *mclk_name;

	pr_err("[AR0330]: probing sensor.\n");

	info = devm_kzalloc(&client->dev,
			sizeof(struct ar0330_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s:Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	info->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(info->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(info->regmap));
		return -ENODEV;
	}

	if (client->dev.of_node)
		info->pdata = ar0330_parse_dt(client);
	else
		info->pdata = client->dev.platform_data;

	if (!info->pdata) {
		pr_err("[AR0330]:%s:Unable to get platform data\n", __func__);
		return -EFAULT;
	}

	info->i2c_client = client;
	atomic_set(&info->in_use, 0);
	info->mode = -1;

	mclk_name = info->pdata->mclk_name ?
		    info->pdata->mclk_name : "default_mclk";
	info->mclk = devm_clk_get(&client->dev, mclk_name);
	if (IS_ERR(info->mclk)) {
		dev_err(&client->dev, "%s: unable to get clock %s\n",
			__func__, mclk_name);
		return PTR_ERR(info->mclk);
	}

	if (info->pdata->dev_name != NULL)
		strncpy(info->devname, info->pdata->dev_name,
			sizeof(info->devname) - 1);
	else
		strncpy(info->devname, "ar0330", sizeof(info->devname) - 1);

	ar0330_power_get(info);

	memcpy(&info->miscdev_info,
		&ar0330_device,
		sizeof(struct miscdevice));

	info->miscdev_info.name = info->devname;

	err = misc_register(&info->miscdev_info);
	if (err) {
		pr_err("%s:Unable to register misc device!\n", __func__);
		goto ar0330_probe_fail;
	}

	i2c_set_clientdata(client, info);

	mutex_init(&info->ar0330_camera_lock);
	pr_err("[AR0330]: end of probing sensor.\n");
	return 0;

ar0330_probe_fail:
	ar0330_power_put(&info->power);

	return err;
}

static int
ar0330_remove(struct i2c_client *client)
{
	struct ar0330_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ar0330_device);
	mutex_destroy(&info->ar0330_camera_lock);

	ar0330_power_put(&info->power);

	return 0;
}

static const struct i2c_device_id ar0330_id[] = {
	{ "ar0330", 0 },
	{ "ar0330.1", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ar0330_id);

static struct i2c_driver ar0330_i2c_driver = {
	.driver = {
		.name = "ar0330",
		.owner = THIS_MODULE,
	},
	.probe = ar0330_probe,
	.remove = ar0330_remove,
	.id_table = ar0330_id,
};

static int __init ar0330_init(void)
{
	pr_info("[AR0330] sensor driver loading\n");
	return i2c_add_driver(&ar0330_i2c_driver);
}

static void __exit ar0330_exit(void)
{
	i2c_del_driver(&ar0330_i2c_driver);
}

module_init(ar0330_init);
module_exit(ar0330_exit);
