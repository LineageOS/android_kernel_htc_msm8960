/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <mach/panel_id.h>
#include "../../../drivers/video/msm/msm_fb.h"
#include "../../../drivers/video/msm/mipi_dsi.h"
#include "mipi_m7.h"

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	{0x03, 0x08, 0x05, 0x00, 0x20},
	{0xDD, 0x51, 0x27, 0x00, 0x6E, 0x74, 0x2C,
	0x55, 0x3E, 0x3, 0x4, 0xA0},
	{0x5F, 0x00, 0x00, 0x10},
	{0xFF, 0x00, 0x06, 0x00},
	{0x00, 0x38, 0x32, 0xDA, 0x00, 0x10, 0x0F, 0x61, 0x41, 0x0F, 0x01, 0x00, 0x1A, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x02},
};

static struct mipi_dsi_phy_ctrl dsi_jdi_cmd_mode_phy_db = {
	{0x03, 0x08, 0x05, 0x00, 0x20},
	{0xD7, 0x34, 0x23, 0x00, 0x63, 0x6A, 0x28, 0x37, 0x3C, 0x03, 0x04},
	{0x5F, 0x00, 0x00, 0x10},
	{0xFF, 0x00, 0x06, 0x00},
	{0x00, 0xA8, 0x30, 0xCA, 0x00, 0x20, 0x0F, 0x62, 0x70, 0x88, 0x99, 0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01},
};
static int __init mipi_video_sharp_init(void)
{
	int ret;

	pinfo.xres = 1080;
	pinfo.yres = 1920;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.width = 61;
	pinfo.height = 110;
	pinfo.camera_backlight = 176;

	pinfo.lcdc.h_back_porch = 50;
	pinfo.lcdc.h_front_porch = 100;
	pinfo.lcdc.h_pulse_width = 10;
	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_front_porch = 4;
	pinfo.lcdc.v_pulse_width = 2;

	pinfo.lcd.v_back_porch = 4;
	pinfo.lcd.v_front_porch = 4;
	pinfo.lcd.v_pulse_width = 2;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 848000000;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = FALSE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = TRUE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;

	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;

	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x02;
	pinfo.mipi.t_clk_pre = 0x2C;
	pinfo.mipi.stream = 0;	/* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.esc_byte_ratio = 6;

	ret = mipi_dlxp_device_register(&pinfo, MIPI_DSI_PRIM,
			MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	video_on_cmds = sharp_video_on_cmds;
	video_on_cmds_count = ARRAY_SIZE(sharp_video_on_cmds);
	display_on_cmds = renesas_display_on_cmds;
	display_on_cmds_count = ARRAY_SIZE(renesas_display_on_cmds);
	display_off_cmds = sharp_display_off_cmds;
	display_off_cmds_count = ARRAY_SIZE(sharp_display_off_cmds);

	mdp_gamma = mdp_gamma_sharp;
	mdp_gamma_count = ARRAY_SIZE(mdp_gamma_sharp);


	return ret;
}

static int __init mipi_video_sony_init(void)
{
	int ret;

	pinfo.xres = 1080;
	pinfo.yres = 1920;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.width = 61;
	pinfo.height = 110;
	pinfo.camera_backlight = 176;

	pinfo.lcdc.h_back_porch = 50;
	pinfo.lcdc.h_front_porch = 100;
	pinfo.lcdc.h_pulse_width = 10;
	pinfo.lcdc.v_back_porch = 3;
	pinfo.lcdc.v_front_porch = 3;
	pinfo.lcdc.v_pulse_width = 2;

	pinfo.lcd.v_back_porch = 3;
	pinfo.lcd.v_front_porch = 3;
	pinfo.lcd.v_pulse_width = 2;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 848000000;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = FALSE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = TRUE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;

	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x02;
	pinfo.mipi.t_clk_pre = 0x2C;
	pinfo.mipi.stream = 0;	/* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;

	ret = mipi_dlxp_device_register(&pinfo, MIPI_DSI_PRIM,
			MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	video_on_cmds = sony_video_on_cmds;
	video_on_cmds_count = ARRAY_SIZE(sony_video_on_cmds);
	display_on_cmds = renesas_display_on_cmds;
	display_on_cmds_count = ARRAY_SIZE(renesas_display_on_cmds);
	display_off_cmds = sony_display_off_cmds;
	display_off_cmds_count = ARRAY_SIZE(sony_display_off_cmds);

	mdp_gamma = mdp_gamma_sony;
	mdp_gamma_count = ARRAY_SIZE(mdp_gamma_sony);

	PR_DISP_INFO("%s\n", __func__);
	return ret;
}


static int __init mipi_m7_1080p_pt_init(void)
{
	switch (panel_type) {
	case PANEL_ID_DLXJ_SHARP_RENESAS:
	case PANEL_ID_M7_SHARP_RENESAS:
		sharp_renesas_panel_init();
		break;
	case PANEL_ID_DLXJ_SONY_RENESAS:
		sony_panel_init();
		break;
	case PANEL_ID_M7_JDI_SAMSUNG:
	case PANEL_ID_M7_JDI_SAMSUNG_C2:
	case PANEL_ID_M7_JDI_SAMSUNG_C2_1:
	case PANEL_ID_M7_JDI_SAMSUNG_C2_2:
		samsung_panel_init();
		break;
	case PANEL_ID_M7_SHARP_RENESAS_C1:
		sharp_panel_init();
		break;
	case PANEL_ID_M7_JDI_RENESAS:
		jdi_renesas_panel_init();
		break;
	default:
		pr_err("%s: Unsupported panel type: %d",
				__func__, panel_type);
		return -ENODEV;
	}
	return 0;
}
late_initcall(mipi_m7_1080p_pt_init);
