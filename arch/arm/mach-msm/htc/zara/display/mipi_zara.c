/* linux/arch/arm/mach-msm/zara/display/mipi-zara.c
 *
 * Copyright (c) 2013 HTC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/gpio.h>

#include <mach/panel_id.h>
#include "../../../drivers/video/msm/msm_fb.h"
#include "../../../drivers/video/msm/mipi_dsi.h"
#include <linux/leds.h>
#include <mach/board.h>
#include "mipi_zara.h"
#include "../board-zara.h"

static struct mipi_dsi_panel_platform_data *mipi_zara_pdata;

static struct dsi_cmd_desc *init_on_cmds = NULL;
static struct dsi_cmd_desc *display_on_cmds = NULL;
static struct dsi_cmd_desc *display_off_cmds = NULL;
static struct dsi_cmd_desc *backlight_cmds = NULL;

static int init_on_cmds_count = 0;
static int display_on_cmds_count = 0;
static int display_off_cmds_count = 0;
static int backlight_cmds_count = 0;
static atomic_t lcd_power_state;

static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00};

static char led_pwm[2] = {0x51, 0xF0};
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char pwr_ctrl_AVDD[] = {0xB6, 0x34};
static char pwr_ctrl_AVEE[] = {0xB7, 0x34};
static char pwr_ctrl_VCL[] = {0xB8, 0x13};
static char pwr_ctrl_VGH[] = {0xB9, 0x24};
static char pwr_ctrl_VGLX[] = {0xBA, 0x23};
static char set_VGMP_VGSP_vol[] = {0xBC, 0x00, 0x88, 0x00};
static char set_VGMN_VGSN_vol[] = {0xBD, 0x00, 0x84, 0x00};
static char GPO_ctrl[] = {0xC0, 0x04, 0x00};
static char gamma_curve_ctrl[] = {0xCF, 0x04};


static char cmd_set_page1[] = {
	0xF0, 0x55, 0xAA, 0x52,
	0x08, 0x01
};
static char gamma_corr_red1[] = {
	0xD1, 0x00, 0x00, 0x00,
	0x48, 0x00, 0x71, 0x00,
	0x95, 0x00, 0xA4, 0x00,
	0xC1, 0x00, 0xD4, 0x00,
	0xFA
};
static char gamma_corr_red2[] = {
	0xD2, 0x01, 0x22, 0x01,
	0x5F, 0x01, 0x89, 0x01,
	0xCC, 0x02, 0x03, 0x02,
	0x05, 0x02, 0x38, 0x02,
	0x71
};
static char gamma_corr_red3[] = {
	0xD3, 0x02, 0x90, 0x02,
	0xC9, 0x02, 0xF4, 0x03,
	0x1A, 0x03, 0x35, 0x03,
	0x52, 0x03, 0x62, 0x03,
	0x76
};
static char gamma_corr_red4[] = {
	0xD4, 0x03, 0x8F, 0x03,
	0xC0
};
static char normal_display_mode_on[] = {0x13, 0x00};

static char cmd_set_page0[] = {
	0xF0, 0x55, 0xAA, 0x52,
	0x08, 0x00
};
static char disp_opt_ctrl[] = {
	0xB1, 0x68, 0x00, 0x01
};
static char disp_scan_line_ctrl[] = {0xB4, 0x78};
static char eq_ctrl[] = {
	0xB8, 0x01, 0x02, 0x02,
	0x02
};
static char inv_drv_ctrl[] = {0xBC, 0x00, 0x00, 0x00};
static char display_timing_control[] = {
	0xC9, 0x63, 0x06, 0x0D,
	0x1A, 0x17, 0x00
};
static char write_ctrl_display[] = {0x53, 0x24};
static char te_on[] = {0x35, 0x00};
static char pwm_freq_ctrl[] = {0xE0, 0x01, 0x03};
static char pwr_blk_enable[] = {
	0xFF, 0xAA, 0x55, 0x25,
	0x01};
static char set_para_idx[] = {0x6F, 0x0A};
static char pwr_blk_sel[] = {0xFA, 0x03};
static char bkl_off[] = {0x53, 0x00};
static char set_cabc_level[] = {0x55, 0x92};
static char vivid_color_setting[] = {
	0xD6, 0x00, 0x05, 0x10,
	0x17, 0x22, 0x26, 0x29,
	0x29, 0x26, 0x23, 0x17,
	0x12, 0x06, 0x02, 0x01,
	0x00};
static char cabc_still[] = {
	0xE3, 0xFF, 0xFB, 0xF3,
	0xEC, 0xE2, 0xCA, 0xC3,
	0xBC, 0xB5, 0xB3
};
static char idx_13[] = {0x6F, 0x13};
static char idx_14[] = {0x6F, 0x14};
static char idx_15[] = {0x6F, 0x15};
static char val_80[] = {0xF5, 0x80};
static char val_FF[] = {0xF5, 0xFF};
static char ctrl_ie_sre[] = {
	0xD4, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};
static char skin_tone_setting1[] = {
	0xD7, 0x30, 0x30, 0x30,
	0x28, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00};
static char skin_tone_setting2[] = {
	0xD8, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x28,
	0x30, 0x00};

static struct dsi_cmd_desc lg_novatek_cmd_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(cmd_set_page1), cmd_set_page1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(pwr_ctrl_AVDD), pwr_ctrl_AVDD},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(pwr_ctrl_AVEE), pwr_ctrl_AVEE},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(pwr_ctrl_VCL), pwr_ctrl_VCL},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(pwr_ctrl_VGH), pwr_ctrl_VGH},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(pwr_ctrl_VGLX), pwr_ctrl_VGLX},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(set_VGMP_VGSP_vol), set_VGMP_VGSP_vol},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(set_VGMN_VGSN_vol), set_VGMN_VGSN_vol},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(GPO_ctrl), GPO_ctrl},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(gamma_curve_ctrl), gamma_curve_ctrl},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(gamma_corr_red1), gamma_corr_red1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(gamma_corr_red2), gamma_corr_red2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(gamma_corr_red3), gamma_corr_red3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(gamma_corr_red4), gamma_corr_red4},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(exit_sleep), exit_sleep},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(cmd_set_page0), cmd_set_page0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(disp_opt_ctrl), disp_opt_ctrl},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(disp_scan_line_ctrl), disp_scan_line_ctrl},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(eq_ctrl), eq_ctrl},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(inv_drv_ctrl), inv_drv_ctrl},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(display_timing_control), display_timing_control},
	{DTYPE_DCS_WRITE, 1, 0, 0, 1, sizeof(te_on), te_on},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(pwm_freq_ctrl), pwm_freq_ctrl},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(cabc_still), cabc_still},

	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(write_ctrl_display), write_ctrl_display},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(set_cabc_level), set_cabc_level},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(ctrl_ie_sre), ctrl_ie_sre},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(vivid_color_setting), vivid_color_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(skin_tone_setting1), skin_tone_setting1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(skin_tone_setting2), skin_tone_setting2},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(pwr_blk_enable), pwr_blk_enable},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(set_para_idx), set_para_idx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(pwr_blk_sel), pwr_blk_sel},

	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(idx_13), idx_13},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(val_80), val_80},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(idx_14), idx_14},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(val_FF), val_FF},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(idx_15), idx_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(val_FF), val_FF},
	{DTYPE_DCS_WRITE, 1, 0, 0, 1, sizeof(normal_display_mode_on), normal_display_mode_on},
};

static struct dsi_cmd_desc lg_novatek_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 130,
		sizeof(enter_sleep), enter_sleep},
};


static struct dsi_cmd_desc lg_novatek_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 1, sizeof(display_on), display_on},
};

static struct dsi_cmd_desc lg_novatek_cmd_backlight_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(led_pwm), led_pwm},
};


static int zara_send_display_cmds(struct dsi_cmd_desc *cmd, int cnt, bool clk_ctrl)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = cmd;
	cmdreq.cmds_cnt = cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	if (clk_ctrl)
		cmdreq.flags |= CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	ret = mipi_dsi_cmdlist_put(&cmdreq);
	if (ret < 0)
		pr_err("%s failed (%d)\n", __func__, ret);
	return ret;
}

// int mipi_lcd_on = 1;
static int mipi_zara_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct msm_panel_info *pinfo;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

//	if (mipi_lcd_on)
//		return 0;

	pinfo = &mfd->panel_info;
	mipi  = &mfd->panel_info.mipi;

	zara_send_display_cmds(init_on_cmds, init_on_cmds_count, false);

	atomic_set(&lcd_power_state, 1);

	pr_debug("Init done\n");

/*	gpio_set_value(MSM_LCD_RSTz, 0);
	msleep(1);
	gpio_set_value(MSM_LCD_RSTz, 1);
	msleep(20);

			zara_send_display_cmds(zara_video_on_cmds,
					zara_video_on_cmds_count, false);
			pr_info("%s: panel_type video mode (%d)", __func__, panel_type);
		pr_info("%s: panel_type not supported!(%d)", __func__, panel_type);
	mipi_lcd_on = 1; */

	return 0;
}

static int mipi_zara_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	return 0;
}

static int mipi_zara_display_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	bool clk_ctrl;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_op_mode_config(DSI_VIDEO_MODE);

	clk_ctrl = (mfd->panel_info.type == MIPI_VIDEO_PANEL);
	zara_send_display_cmds(display_on_cmds, display_on_cmds_count, clk_ctrl);

	return 0;
}

static int mipi_zara_display_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	bool clk_ctrl;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	clk_ctrl = (mfd->panel_info.type == MIPI_VIDEO_PANEL);
	zara_send_display_cmds(display_off_cmds, display_off_cmds_count, clk_ctrl);


	atomic_set(&lcd_power_state, 0);

	pr_err("%s\n", __func__);

	return 0;
}

#define BRI_SETTING_MIN                 30
#define BRI_SETTING_DEF                 143
#define BRI_SETTING_MAX                 255

static unsigned char zara_shrink_pwm(int val)
{
	unsigned int pwm_min, pwm_default, pwm_max;
	unsigned char shrink_br = BRI_SETTING_MAX;

	pwm_min = 7;
	pwm_default = 86;
	pwm_max = 255;

	if (val <= 0) {
		shrink_br = 0;
	} else if (val > 0 && (val < BRI_SETTING_MIN)) {
			shrink_br = pwm_min;
	} else if ((val >= BRI_SETTING_MIN) && (val <= BRI_SETTING_DEF)) {
			shrink_br = (val - BRI_SETTING_MIN) * (pwm_default - pwm_min) /
		(BRI_SETTING_DEF - BRI_SETTING_MIN) + pwm_min;
	} else if (val > BRI_SETTING_DEF && val <= BRI_SETTING_MAX) {
			shrink_br = (val - BRI_SETTING_DEF) * (pwm_max - pwm_default) /
		(BRI_SETTING_MAX - BRI_SETTING_DEF) + pwm_default;
	} else if (val > BRI_SETTING_MAX)
			shrink_br = pwm_max;

	pr_info("brightness orig=%d, transformed=%d\n", val, shrink_br);

	return shrink_br;
}

static void zara_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	bool clk_ctrl = (mfd && mfd->panel_info.type == MIPI_VIDEO_PANEL);

	led_pwm[1] = zara_shrink_pwm((unsigned char)(mfd->bl_level));


	mipi  = &mfd->panel_info.mipi;
	pr_debug("%s+:bl=%d \n", __func__, mfd->bl_level);

	if (atomic_read(&lcd_power_state) == 0) {
		pr_debug("%s: LCD is off. Skip backlight setting\n", __func__);
		return;
	}

	if (mipi->mode == DSI_VIDEO_MODE) {
		mipi_dsi_op_mode_config(DSI_VIDEO_MODE);
	}

	if (mipi->mode == DSI_CMD_MODE) {
		return;
	}

	zara_send_display_cmds(backlight_cmds, backlight_cmds_count, clk_ctrl);

	return;
}

static int __devinit mipi_zara_lcd_probe(struct platform_device *pdev)
{
	/* Common parts */
	init_on_cmds = lg_novatek_cmd_on_cmds;
	init_on_cmds_count = ARRAY_SIZE(lg_novatek_cmd_on_cmds);

	display_on_cmds = lg_novatek_display_on_cmds;
	display_on_cmds_count = ARRAY_SIZE(lg_novatek_display_on_cmds);
	display_off_cmds = lg_novatek_display_off_cmds;
	display_off_cmds_count = ARRAY_SIZE(lg_novatek_display_off_cmds);
	backlight_cmds = lg_novatek_cmd_backlight_cmds;
	backlight_cmds_count = ARRAY_SIZE(lg_novatek_cmd_backlight_cmds);

	if (pdev->id == 0) {
		mipi_zara_pdata = pdev->dev.platform_data;
	}
	msm_fb_add_device(pdev);

	return 0;
}

/* HTC specific functions */

static struct platform_driver this_driver = {
	.probe  = mipi_zara_lcd_probe,
	.driver = {
		.name   = "mipi_zara",
	},
};

static struct msm_fb_panel_data zara_panel_data = {
	.on		= mipi_zara_lcd_on,
	.off		= mipi_zara_lcd_off,
	.set_backlight  = mipi_zara_set_backlight,
	.late_init = mipi_zara_display_on,
	.early_off = mipi_zara_display_off,
};

static int ch_used[3];

int mipi_zara_device_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;
	atomic_set(&lcd_power_state, 1);

	ret = platform_driver_register(&this_driver);
	if (ret) {
		pr_err("platform_driver_register() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_zara", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	zara_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &zara_panel_data,
		sizeof(zara_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}
