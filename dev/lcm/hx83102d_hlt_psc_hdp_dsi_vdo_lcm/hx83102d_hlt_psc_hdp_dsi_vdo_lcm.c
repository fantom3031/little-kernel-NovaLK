/********************************************
 ** Copyright (C), 2019-2020, OPPO Mobile Comm Corp., Ltd.
 ** ODM_WT_EDIT
 ** File: hx83102d_hlt_psc_hdp_dsi_vdo_lcm.c
 ** Description: LCM driver for realme C15 (RMX2180) HX83102D + HLT panel
 **
 ** Sources:
 **   - AndroidQ kernel driver (init table, DSI params, suspend table)
 **   - stock RUI2 C.13 LK for project 20701, recovered from lk.img:
 **     GPIO45=LCM reset, GPIO150=bias ENP, GPIO151=bias ENN,
 **     NT5038 bias IC on I2C channel 3 @ 0x3E (regs 0x00/0x01 = 0x14),
 **     compare_id reads DSI 0xDA and expects 0x55.
 ********************************************/

#define LOG_TAG "LCM_HX83102D_HLT"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#include <platform/boot_mode.h>
#endif

#define LCM_LOGI(string, args...) dprintf(0, "[LK/" LOG_TAG "]" string, ##args)

/* GPIO map recovered from the stock C15 LK (project 20701) */
#define LCM_RESET_PIN                                        (GPIO45 | 0x80000000)
#ifndef GPIO_LCD_BIAS_ENP_PIN
#define GPIO_LCD_BIAS_ENP_PIN                                (GPIO150 | 0x80000000)
#endif
#ifndef GPIO_LCD_BIAS_ENN_PIN
#define GPIO_LCD_BIAS_ENN_PIN                                (GPIO151 | 0x80000000)
#endif

#ifdef BUILD_LK
/* NT5038 bias IC: I2C channel 3, 7-bit address 0x3E (write byte 0x7C) */
#define NT5038_SLAVE_ADDR_WRITE  0x7C
#define I2C_I2C_LCD_BIAS_CHANNEL 3

static struct mt_i2c_t NT5038_i2c;

static int nt5038_i2c_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;
	NT5038_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;
	NT5038_i2c.addr = (NT5038_SLAVE_ADDR_WRITE >> 1);
	NT5038_i2c.mode = ST_MODE;
	NT5038_i2c.speed = 100;
	len = 2;
	ret_code = i2c_write(&NT5038_i2c, write_data, len);
	LCM_LOGI("%s: i2c_write reg 0x%x = 0x%x, ret %d\n", __func__, addr, value, ret_code);
	return ret_code;
}
#endif

static LCM_UTIL_FUNCS lcm_util;

#define MDELAY(n)        (lcm_util.mdelay(n))
#define UDELAY(n)        (lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define LCM_DSI_CMD_MODE    0
#define FRAME_WIDTH         (720)
#define FRAME_HEIGHT        (1600)

#define LCM_PHYSICAL_WIDTH  (67930)
#define LCM_PHYSICAL_HEIGHT (150960)

#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY      0xFFFB
#define REGFLAG_END_OF_TABLE 0xFFFD

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

/* Power-on init table: verbatim from the AndroidQ kernel driver
 * (hx83102d_hlt_psc_hdp_dsi_vdo_lcm.c, init_setting_vdo). */
static struct LCM_setting_table init_setting_vdo[] = {
{0x51,2,{0x00,0x00}},
{0x53,1,{0x2C}},
{0xBD,0x01,{0x00}},
{0x11,0,{0x00}},
{REGFLAG_DELAY, 120, {}},

{0xB9,0x03,{0x83,0x10,0x2D}},
{0xB1,0x0B,{0x22,0x44,0x31,0x31,0x22,0x34,0x2F,0x57,0x0D,0x0D,0x0D}},
{0xB2,0x0E,{0x00,0x00,0x06,0x40,0x00,0x08,0xBE,0x37,0x13,0x00,0x00,0x01,0x85,0xE0}},
{0xB4,0x0E,{0x58,0x4F,0x58,0x4F,0x58,0x52,0x58,0x4F,0x05,0xFF,0x03,0x03,0x00,0xFF}},
{0xCC,0x01,{0x02}},
{0xD3,0x19,{0x00,0x00,0x01,0x01,0x00,0x08,0x00,0x47,0x00,0x43,0x33,0x0B,0x0B,0x00,0x00,0x32,0x10,0x07,0x00,0x07,0x54,0x15,0xA8,0x05,0xA8}},
{0xD5,0x2C,{0x18,0x18,0x18,0x18,0x3A,0x3A,0x18,0x18,0x20,0x21,0x22,0x23,0x19,0x19,0x19,0x19,0x04,0x05,0x06,0x07,0x00,0x01,0x02,0x03,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
{0xD6,0x2C,{0x18,0x18,0x19,0x19,0x3A,0x3A,0x18,0x18,0x23,0x22,0x21,0x20,0x18,0x18,0x19,0x19,0x03,0x02,0x01,0x00,0x07,0x06,0x05,0x04,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
{0xE7,0x02,{0xFF,0x0F}},
{0xBD,0x01,{0x01}},
{0xE7,0x01,{0x01}},
{0xBD,0x01,{0x00}},
{0xD8,0x24,{0x2A,0xAA,0xAA,0x00,0x00,0x00,0x2A,0xAA,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xBD,0x01,{0x01}},
{0xD8,0x24,{0x3F,0xFF,0xFF,0x00,0x00,0x00,0x3F,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xBD,0x01,{0x02}},
{0xD8,0x0C,{0xFF,0xFF,0xFF,0xFF,0xFF,0xF0,0xFF,0xFF,0xFF,0xFF,0xFF,0xF0}},
{0xBD,0x01,{0x03}},
{0xD8,0x18,{0xAA,0xAA,0xAA,0xAA,0xAA,0xA0,0xAA,0xAA,0xAA,0xAA,0xAA,0xA0,0xBF,0xFF,0xFF,0xAA,0xAA,0xA0,0xBF,0xFF,0xFF,0xAA,0xAA,0xA0}},
{0xBD,0x01,{0x00}},
{0xE0,0x2E,{0x00,0x02,0x07,0x0C,0x10,0x15,0x2A,0x31,0x3B,0x3A,0x5A,0x66,0x72,0x89,0x8D,0x9D,0xA9,0xC1,0xC4,0x62,0x6B,0x77,0x7F,0x00,0x02,0x07,0x0C,0x10,0x15,0x2A,0x31,0x3B,0x3A,0x5A,0x66,0x72,0x89,0x8D,0x9D,0xA9,0xC1,0xC4,0x62,0x6B,0x77,0x7F}},
{0xBA,0x13,{0x71,0x23,0xA8,0x93,0xB2,0xC0,0xC0,0x01,0x10,0x00,0x00,0x00,0x0D,0x3D,0x82,0x77,0x04,0x01,0x00}},
{0xBF,0x07,{0xFC,0x00,0x04,0x9E,0xF6,0x00,0x51}},
{0xCB,0x05,{0x00,0x13,0x00,0x02,0x58}},
{0xC0,0x06,{0x32,0x32,0x00,0x00,0x19,0x21}},
{0xBD,0x01,{0x01}},
{0xCB,0x01,{0x01}},
{0xBD,0x01,{0x02}},
{0xB4,0x08,{0x42,0x00,0x33,0x00,0x33,0x88,0xB3,0x00}},
{0xB1,0x03,{0x7F,0x03,0xF5}},
{0xBD,0x01,{0x00}},
{0xC1,0x01,{0x00}},
{0x29,0,{0x00}},
{REGFLAG_DELAY, 20, {}},
{0x35,1,{0x00}},
{0xC9,4,{0x44,0x08,0x54,0x01}},
{0xE9,1,{0xD0}},
{0xE4,9,{0x0F,0x1E,0x50,0x50,0x00,0x00,0x05,0x01,0x14}},
{0xE9,1,{0x3F}},
{0x5E,2,{0x01,0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}},
};

/* Suspend table (stock LK timing: 20/120 ms) */
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;
		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
				table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH / 1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT / 1000;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode_enable = 0;

	/* The C15 panel runs on three data lanes */
	params->dsi.LANE_NUM = LCM_THREE_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 218;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 16;
	params->dsi.horizontal_backporch = 30;
	params->dsi.horizontal_frontporch = 18;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.ssc_disable = 1;
	params->dsi.data_rate = 733;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9D;
}

/* Stock LK init_power (recovered from lk.img):
 *   configure GPIO45/150/151 as outputs, delay 1,
 *   ENP (150) = 1, delay 3, ENN (151) = 1,
 *   NT5038: reg 0x00 = 0x14, reg 0x01 = 0x14, delay 6.
 * (kernel equivalent: SET_LCM_VSP_PIN(1); MDELAY(3); SET_LCM_VSN_PIN(1)
 *  plus display_bias_setting(0x14)). */
static void lcm_init_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);

	mt_set_gpio_mode(LCM_RESET_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(LCM_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);

	MDELAY(1);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(3);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);

#ifdef BUILD_LK
	nt5038_i2c_write_byte(0x00, 0x14);
	MDELAY(1);
	nt5038_i2c_write_byte(0x01, 0x14);
	MDELAY(1);
#endif
	MDELAY(6);

	LCM_LOGI("%s: exit\n", __func__);
}

/* Stock LK keeps suspend/resume power as no-ops; the rails stay up and
 * lcm_resume() re-runs the full init table. */
static void lcm_suspend_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
}

static void lcm_resume_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
}

/* Stock LK init: reset pulse (0 -> 10 ms -> 1 -> 30 ms) then the init table. */
static void lcm_init(void)
{
	LCM_LOGI("%s: enter\n", __func__);

	MDELAY(5);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(30);

	push_table(NULL, init_setting_vdo,
		sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);

	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_suspend(void)
{
	LCM_LOGI("%s: enter\n", __func__);

	push_table(NULL, lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);

	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_resume(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	lcm_init();
	LCM_LOGI("%s: exit\n", __func__);
}

/* Stock LK compare_id (recovered from lk.img): reset pulse, then DSI read
 * of register 0xDA; the panel returns 0x55.
 * Three zero-reads (probe before power/DSI ready) assume the panel is
 * present, so a single-panel build is never de-selected. */
static unsigned int lcm_compare_id(void)
{
	unsigned char buffer[2] = {0};
	unsigned int id = 0;
	int retry = 0;

	LCM_LOGI("%s: Reading LCM ID\n", __func__);

	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(30);

	for (retry = 0; retry < 3; retry++) {
		buffer[0] = 0;
		MDELAY(10);
		read_reg_v2(0xDA, buffer, 1);
		id = buffer[0];
		LCM_LOGI("%s: attempt %d, ID = 0x%02x\n", __func__, retry + 1, id);

		if (id == 0x55)
			return 1;
		if (id != 0x00)
			return 0;
	}

	LCM_LOGI("%s: no response, assuming LCM present\n", __func__);
	return 1;
}

/* ESD check: register 0x0A must read back 0x9D.
 * Return convention: 0 = healthy, non-zero = ESD detected. */
static unsigned int lcm_esd_check(void)
{
	unsigned char buffer[2] = {0};
	unsigned int id = 0;

	read_reg_v2(0x0A, buffer, 1);
	id = buffer[0];
	LCM_LOGI("%s: Read ID = 0x%02x (expected 0x9D)\n", __func__, id);

	if (id == 0x9D)
		return 0;
	return 1;
}

/* 12-bit DBV via DSI 0x51 (two data bytes). LK passes 0..4095 here
 * (cust_leds scales with NOVALK_BACKLIGHT_DBV_MAX=4095). */
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	if (level > 4095)
		level = 4095;

	bl_level[0].para_list[0] = (level >> 8) & 0x0F;
	bl_level[0].para_list[1] = level & 0xFF;
	push_table(handle, bl_level,
		sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}

/* Legacy path: LK's disp_lcm_set_backlight() only calls set_backlight(). */
static void lcm_setbacklight(unsigned int level)
{
	lcm_setbacklight_cmdq(NULL, level);
}

LCM_DRIVER hx83102d_hlt_psc_hdp_dsi_vdo_lcm_drv = {
	.name = "hx83102d_hlt_psc_hdp_dsi_vdo_lcm",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.suspend_power = lcm_suspend_power,
	.resume_power = lcm_resume_power,
	.compare_id = lcm_compare_id,
	.set_backlight = lcm_setbacklight,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.esd_check = lcm_esd_check,
};
