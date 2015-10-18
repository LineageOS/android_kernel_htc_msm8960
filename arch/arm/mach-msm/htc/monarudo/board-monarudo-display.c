/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/ion.h>
#include <asm/mach-types.h>
#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/ion.h>
#include <mach/msm_bus_board.h>
#include <mach/panel_id.h>
#include <mach/debug_display.h>
#include "devices.h"
#include "board-monarudo.h"
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include "../../../../drivers/video/msm/msm_fb.h"
#include "../../../../drivers/video/msm/mipi_dsi.h"
#include "../../../../drivers/video/msm/mdp4.h"
#include <linux/i2c.h>
#include <mach/msm_xo.h>

#define hr_msleep(x) msleep(x)

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
/* prim = 1366 x 768 x 3(bpp) x 3(pages) */
#define MSM_FB_PRIM_BUF_SIZE (1920 * ALIGN(1080, 32) * 4 * 3)
#else
/* prim = 1366 x 768 x 3(bpp) x 2(pages) */
#define MSM_FB_PRIM_BUF_SIZE (1920 * ALIGN(1080, 32) * 4 * 2)
#endif

#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE, 4096)

#ifdef CONFIG_FB_MSM_OVERLAY0_WRITEBACK
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE roundup((1920 * 1080 * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY0_WRITEBACK */

#ifdef CONFIG_FB_MSM_OVERLAY1_WRITEBACK
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE roundup((1920 * 1088 * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY1_WRITEBACK */

static struct resource msm_fb_resources[] = {
	{
		.flags = IORESOURCE_DMA,
	}
};
struct msm_xo_voter *wa_xo;
static char ptype[60] = "PANEL type = ";
const size_t ptype_len = ( 60 - sizeof("PANEL type = "));

#define MIPI_NOVATEK_PANEL_NAME "mipi_cmd_novatek_qhd"
#define MIPI_RENESAS_PANEL_NAME "mipi_video_renesas_fiwvga"
#define MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME "mipi_video_toshiba_wsvga"
#define MIPI_VIDEO_CHIMEI_WXGA_PANEL_NAME "mipi_video_chimei_wxga"
#define HDMI_PANEL_NAME "hdmi_msm"
#define TVOUT_PANEL_NAME "tvout_msm"

static int monarudo_detect_panel(const char *name)
{
	if (!strncmp(name, HDMI_PANEL_NAME,
		strnlen(HDMI_PANEL_NAME,
			PANEL_NAME_MAX_LEN)))
		return 0;

	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = monarudo_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name	      = "msm_fb",
	.id		= 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource	  = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};

void __init monarudo_allocate_fb_region(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));
}

#define MDP_VSYNC_GPIO 0

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_ui_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 216000000 * 2,
		.ib = 270000000 * 2,
	},
};

static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 216000000 * 2,
		.ib = 270000000 * 2,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 230400000 * 2,
		.ib = 288000000 * 2,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 334080000 * 2,
		.ib = 417600000 * 2,
	},
};

static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};

static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 566092800 * 2,
		.ib = 707616000 * 2,
	},
};

static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};

static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = MDP_VSYNC_GPIO,
	.mdp_max_clk = 266667000,
	.mdp_max_bw = 2000000000,
	.mdp_bw_ab_factor = 140,
	.mdp_bw_ib_factor = 210,
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
	.mdp_rev = MDP_REV_44,
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	.mem_hid = BIT(ION_CP_MM_HEAP_ID),
#else
	.mem_hid = MEMTYPE_EBI1,
#endif
	.cont_splash_enabled = 0x01,
	.splash_screen_size = 0x3f4800,
	.mdp_iommu_split_domain = 1,
};

static char wfd_check_mdp_iommu_split_domain(void)
{
	return mdp_pdata.mdp_iommu_split_domain;
}

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
static struct msm_wfd_platform_data wfd_pdata = {
	.wfd_check_mdp_iommu_split = wfd_check_mdp_iommu_split_domain,
};

static struct platform_device wfd_panel_device = {
	.name = "wfd_panel",
	.id = 0,
	.dev.platform_data = NULL,
};

static struct platform_device wfd_device = {
	.name = "msm_wfd",
	.id = -1,
	.dev.platform_data = &wfd_pdata,
};
#endif

void __init monarudo_mdp_writeback(struct memtype_reserve* reserve_table)
{
	mdp_pdata.ov0_wb_size = MSM_FB_OVERLAY0_WRITEBACK_SIZE;
	mdp_pdata.ov1_wb_size = MSM_FB_OVERLAY1_WRITEBACK_SIZE;
#if defined(CONFIG_ANDROID_PMEM) && !defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov0_wb_size;
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov1_wb_size;

	pr_info("mem_map: mdp reserved with size 0x%lx in pool\n",
			mdp_pdata.ov0_wb_size + mdp_pdata.ov1_wb_size);
#endif
}

static bool dsi_power_on = false;
static bool resume_blk = false;
static bool first_init = true;
static bool backlight_gpio_is_on = true;

static void 
backlight_gpio_enable(bool on)
{
	PR_DISP_DEBUG("monarudo's %s: request on=%d currently=%d\n", __func__, on, backlight_gpio_is_on);

	if (on == backlight_gpio_is_on)
		return;

	if (system_rev == XB) {
		gpio_tlmm_config(GPIO_CFG(MBAT_IN_XA_XB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(MBAT_IN_XA_XB, on ? 1 : 0);
	} else if (system_rev >= XC) {
		PR_DISP_DEBUG("monarudo's %s: turning %s backlight for >= XC\n", __func__, on ? "ON" : "OFF");
		gpio_tlmm_config(GPIO_CFG(BL_HW_EN_XC_XD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(BL_HW_EN_XC_XD, on ? 1 : 0);
		msleep(1);
	}

	backlight_gpio_is_on = on;
}

static void
backlight_gpio_off(void)
{
	backlight_gpio_enable(false);
}

static void
backlight_gpio_on(void)
{
	backlight_gpio_enable(true);
}

static int mipi_dsi_panel_power(int on)
{
	static struct regulator *reg_lvs5, *reg_l2;
	static int gpio36, gpio37;
	int rc;

	pr_debug("%s: on=%d\n", __func__, on);

	if (!dsi_power_on) {
		PR_DISP_DEBUG("monarudo's %s: powering on.\n", __func__);
		reg_lvs5 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi1_vddio");
		if (IS_ERR_OR_NULL(reg_lvs5)) {
			pr_err("could not get 8921_lvs5, rc = %ld\n",
				PTR_ERR(reg_lvs5));
			return -ENODEV;
		}

		reg_l2 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi1_pll_vdda");
		if (IS_ERR_OR_NULL(reg_l2)) {
			pr_err("could not get 8921_l2, rc = %ld\n",
				PTR_ERR(reg_l2));
			return -ENODEV;
		}

		rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
		if (rc) {
			pr_err("set_voltage l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		gpio36 = PM8921_GPIO_PM_TO_SYS(V_LCM_N5V_EN); /* lcd1_pwr_en_n */
		rc = gpio_request(gpio36, "lcd_5v-");
		if (rc) {
			pr_err("request lcd_5v- failed, rc=%d\n", rc);
			return -ENODEV;
		}
		gpio37 = PM8921_GPIO_PM_TO_SYS(V_LCM_P5V_EN); /* pwm_en */
		rc = gpio_request(gpio37, "lcd_5v+");
		if (rc) {
			pr_err("request lcd_5v+ failed, rc=%d\n", rc);
			return -ENODEV;
		}
		gpio_tlmm_config(GPIO_CFG(LCD_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

		dsi_power_on = true;
	}

	if (on) {
		if (!first_init) {
			PR_DISP_DEBUG("monarudo's %s: turning on, previously initialized\n", __func__);
			rc = regulator_set_optimum_mode(reg_l2, 100000);
			if (rc < 0) {
				pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
				return -EINVAL;
			}
			rc = regulator_enable(reg_l2);
			if (rc) {
				pr_err("enable l2 failed, rc=%d\n", rc);
				return -ENODEV;
			}
			rc = regulator_enable(reg_lvs5);
			if (rc) {
				pr_err("enable lvs5 failed, rc=%d\n", rc);
				return -ENODEV;
			}
			hr_msleep(1);
			gpio_set_value_cansleep(gpio37, 1);
			hr_msleep(2);
			gpio_set_value_cansleep(gpio36, 1);
			hr_msleep(7);
			gpio_set_value(LCD_RST, 1);

			/* Workaround for 1mA */
			msm_xo_mode_vote(wa_xo, MSM_XO_MODE_ON);
			hr_msleep(10);

			msm_xo_mode_vote(wa_xo, MSM_XO_MODE_OFF);
		} else {
			PR_DISP_DEBUG("monarudo's %s: turning on, initializing\n", __func__);
			/*Regulator needs enable first time*/
			rc = regulator_enable(reg_lvs5);
			if (rc) {
				pr_err("enable lvs5 failed, rc=%d\n", rc);
				return -ENODEV;
			}
			rc = regulator_set_optimum_mode(reg_l2, 100000);
			if (rc < 0) {
				pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
				return -EINVAL;
			}
			rc = regulator_enable(reg_l2);
			if (rc) {
				pr_err("enable l2 failed, rc=%d\n", rc);
				return -ENODEV;
			}
			/* Workaround for 1mA */
			msm_xo_mode_vote(wa_xo, MSM_XO_MODE_ON);
			msleep(10);
			msm_xo_mode_vote(wa_xo, MSM_XO_MODE_OFF);
		}
	} else {
		PR_DISP_DEBUG("monarudo's %s: turning off\n", __func__);
		backlight_gpio_off();

		gpio_set_value(LCD_RST, 0);
		hr_msleep(3);

		gpio_set_value_cansleep(gpio36, 0);
		hr_msleep(2);
		gpio_set_value_cansleep(gpio37, 0);

		hr_msleep(8);
		rc = regulator_disable(reg_lvs5);
		if (rc) {
			pr_err("disable reg_lvs5 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_l2);
		if (rc) {
			pr_err("disable reg_l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
	}

	return 0;
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.dsi_power_save = mipi_dsi_panel_power,
};

static struct mipi_dsi_panel_platform_data *mipi_monarudo_pdata;

static struct dsi_cmd_desc *video_on_cmds = NULL;
static struct dsi_cmd_desc *display_on_cmds = NULL;
static struct dsi_cmd_desc *display_off_cmds = NULL;

static int video_on_cmds_count = 0;
static int display_on_cmds_count = 0;
static int display_off_cmds_count = 0;

static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */

static char write_display_brightness[3]= {0x51, 0x0F, 0xFF};
static char write_control_display[2] = {0x53, 0x24}; /* DTYPE_DCS_WRITE1 */

static struct dsi_cmd_desc renesas_cmd_backlight_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(write_display_brightness), write_display_brightness},
};

static struct dsi_cmd_desc renesas_display_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_on), display_on},
};

static char interface_setting_0[2] = {0xB0, 0x04};

static char Color_enhancement[33]= {
	0xCA, 0x01, 0x02, 0xA4,
	0xA4, 0xB8, 0xB4, 0xB0,
	0xA4, 0x3F, 0x28, 0x05,
	0xB9, 0x90, 0x70, 0x01,
	0xFF, 0x05, 0xF8, 0x0C,
	0x0C, 0x0C, 0x0C, 0x13,
	0x13, 0xF0, 0x20, 0x10,
	0x10, 0x10, 0x10, 0x10,
	0x10};
static char Outline_Sharpening_Control[3] = {
	0xDD, 0x11, 0xA1};
static char BackLight_Control_6[8]= {
	0xCE, 0x00, 0x01, 0x00,
	0xC1, 0xF4, 0xB2, 0x02};
static char Manufacture_Command_setting[4] = {0xD6, 0x01};
static char nop[4] = {0x00, 0x00};
static char CABC[2] = {0x55, 0x01};
static char hsync_output[4] = {0xC3, 0x01, 0x00, 0x10};
static char protect_on[4] = {0xB0, 0x03};
static char TE_OUT[4] = {0x35, 0x00};
static char deep_standby_off[2] = {0xB1, 0x01};


static struct dsi_cmd_desc sharp_video_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(interface_setting_0), interface_setting_0},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nop), nop},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nop), nop},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(Manufacture_Command_setting), Manufacture_Command_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(Color_enhancement), Color_enhancement},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(Outline_Sharpening_Control), Outline_Sharpening_Control},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(BackLight_Control_6), BackLight_Control_6},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(write_control_display), write_control_display},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(CABC), CABC},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(TE_OUT), TE_OUT},
//	{DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(display_on), display_on},
	{DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(exit_sleep), exit_sleep},
};

static struct dsi_cmd_desc sony_video_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(interface_setting_0), interface_setting_0},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nop), nop},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nop), nop},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(hsync_output), hsync_output},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(Color_enhancement), Color_enhancement},
	//{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(Outline_Sharpening_Control), Outline_Sharpening_Control},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(BackLight_Control_6), BackLight_Control_6},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(Manufacture_Command_setting), Manufacture_Command_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(protect_on), protect_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nop), nop},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nop), nop},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(CABC), CABC},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(write_control_display), write_control_display},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(TE_OUT), TE_OUT},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(exit_sleep), exit_sleep},
};

static struct dsi_cmd_desc sharp_display_off_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 20, sizeof(display_off), display_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 50, sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc sony_display_off_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_off), display_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 48, sizeof(enter_sleep), enter_sleep},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(interface_setting_0), interface_setting_0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(nop), nop},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(nop), nop},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(deep_standby_off), deep_standby_off},
};

static struct i2c_client *blk_pwm_client;
static struct dcs_cmd_req cmdreq;

static int monarudo_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if(! first_init) {
		struct mipi_panel_info *mipi = &mfd->panel_info.mipi;

		PR_DISP_DEBUG("%s: turning on the display.\n", __func__);

		if (mipi->mode == DSI_VIDEO_MODE) {
			cmdreq.cmds = video_on_cmds;
			cmdreq.cmds_cnt = video_on_cmds_count;
			cmdreq.flags = CMD_REQ_COMMIT;
			cmdreq.rlen = 0;
			cmdreq.cb = NULL;

			mipi_dsi_cmdlist_put(&cmdreq);

			PR_DISP_INFO("%s\n", __func__);
		}
	}
	first_init = false;

	return 0;
}

static int monarudo_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	resume_blk = true;

	PR_DISP_INFO("%s\n", __func__);

	return 0;
}


static int __devinit monarudo_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_monarudo_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	PR_DISP_INFO("%s\n", __func__);
	return 0;
}

static int monarudo_display_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	/* It needs 120ms when LP to HS for renesas */
	msleep(120);

	PR_DISP_DEBUG("%s: turning on the display.\n", __func__);

	cmdreq.cmds = display_on_cmds;
	cmdreq.cmds_cnt = display_on_cmds_count;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

	PR_DISP_INFO("%s\n", __func__);
	return 0;
}

static int monarudo_display_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	cmdreq.cmds = display_off_cmds;
	cmdreq.cmds_cnt = display_off_cmds_count;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

	PR_DISP_INFO("%s\n", __func__);
	return 0;
}

#define PWM_MIN		   13
#define PWM_DEFAULT	       82
#define PWM_MAX		   255

#define BRI_SETTING_MIN		 30
#define BRI_SETTING_DEF		 142
#define BRI_SETTING_MAX		 255

static unsigned char monarudo_shrink_pwm(int val)
{
	unsigned char shrink_br = BRI_SETTING_MAX;

	if (val <= 0) {
		shrink_br = 0;
	} else if (val > 0 && (val < BRI_SETTING_MIN)) {
		shrink_br = PWM_MIN;
	} else if ((val >= BRI_SETTING_MIN) && (val <= BRI_SETTING_DEF)) {
		shrink_br = (val - BRI_SETTING_MIN) * (PWM_DEFAULT - PWM_MIN) /
		(BRI_SETTING_DEF - BRI_SETTING_MIN) + PWM_MIN;
	} else if (val > BRI_SETTING_DEF && val <= BRI_SETTING_MAX) {
		shrink_br = (val - BRI_SETTING_DEF) * (PWM_MAX - PWM_DEFAULT) /
		(BRI_SETTING_MAX - BRI_SETTING_DEF) + PWM_DEFAULT;
	} else if (val > BRI_SETTING_MAX)
		shrink_br = PWM_MAX;

	PR_DISP_DEBUG("brightness orig=%d, transformed=%d\n", val, shrink_br);

	return shrink_br;
}

static void monarudo_set_backlight(struct msm_fb_data_type *mfd)
{
	int rc;

	write_display_brightness[2] = monarudo_shrink_pwm((unsigned char)(mfd->bl_level));

	if (resume_blk) {
		resume_blk = false;

		PR_DISP_DEBUG("%s: resuming backlight\n", __func__);

		backlight_gpio_on();

		rc = i2c_smbus_write_byte_data(blk_pwm_client, 0x10, 0xC5);
		if (rc)
			pr_err("i2c write fail\n");
		rc = i2c_smbus_write_byte_data(blk_pwm_client, 0x19, 0x13);
		if (rc)
			pr_err("i2c write fail\n");
		rc = i2c_smbus_write_byte_data(blk_pwm_client, 0x14, 0xC2);
		if (rc)
			pr_err("i2c write fail\n");
		rc = i2c_smbus_write_byte_data(blk_pwm_client, 0x79, 0xFF);
		if (rc)
			pr_err("i2c write fail\n");
		rc = i2c_smbus_write_byte_data(blk_pwm_client, 0x1D, 0xFA);
		if (rc)
			pr_err("i2c write fail\n");
	}

	cmdreq.cmds = (struct dsi_cmd_desc*)&renesas_cmd_backlight_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

	if((mfd->bl_level) == 0) {
		PR_DISP_DEBUG("%s: disabling backlight\n", __func__);
		backlight_gpio_off();
		resume_blk = true;
	}

	return;
}

static struct platform_driver this_driver = {
	.probe  = monarudo_lcd_probe,
	.driver = {
		.name   = "mipi_monarudo",
	},
};

static struct msm_fb_panel_data monarudo_panel_data = {
	.on	= monarudo_lcd_on,
	.off	= monarudo_lcd_off,
	.set_backlight = monarudo_set_backlight,
	.late_init = monarudo_display_on,
	.early_off = monarudo_display_off,
};

static struct msm_panel_info pinfo;
static int ch_used[3] = {0};

int mipi_monarudo_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_monarudo", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	monarudo_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &monarudo_panel_data,
		sizeof(monarudo_panel_data));
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

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	/* DSI_BIT_CLK at 848MHz, 4 lane, RGB888 */
	/* regulator *//* off=0x0500 */
	{0x03, 0x08, 0x05, 0x00, 0x20},
	/* timing *//* off=0x0440 */
	{0xDB, 0x35, 0x24, 0x00, 0x65, 0x68, 0x29,
	0x38, 0x3D, 0x3, 0x4, 0xA0},
	/* phy ctrl *//* off=0x0470 */
	{0x5F, 0x00, 0x00, 0x10},
	/* strength *//* off=0x0480 */
	{0xFF, 0x00, 0x06, 0x00},
	/* pll control *//* off=0x0204 */
	{0x00, 0x38, 0x32, 0xDA, 0x00, 0x10, 0x0F, 0x61,
	0x41, 0x0F, 0x01,
	0x00, 0x1A, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x02 },
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
	pinfo.width = 58;
	pinfo.height = 103;
	pinfo.camera_backlight = 183;

	pinfo.lcdc.h_back_porch = 50;
	pinfo.lcdc.h_front_porch = 100;
	pinfo.lcdc.h_pulse_width = 10;
	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_front_porch = 4;
	pinfo.lcdc.v_pulse_width = 2;

	pinfo.lcd.v_back_porch = 4;
	pinfo.lcd.v_front_porch = 4;
	pinfo.lcd.v_pulse_width = 2;

	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;

	pinfo.lcd.primary_vsync_init = pinfo.yres;
	pinfo.lcd.primary_rdptr_irq = 0;
	pinfo.lcd.primary_start_pos = pinfo.yres +
		pinfo.lcd.v_back_porch + pinfo.lcd.v_front_porch - 1;

	pinfo.lcdc.border_clr = 0;
	pinfo.lcdc.underflow_clr = 0xff;
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
	pinfo.mipi.stream = 0;
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.esc_byte_ratio = 2;

	ret = mipi_monarudo_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	strncat(ptype, "PANEL_ID_DLX_SHARP_RENESAS", ptype_len);

	video_on_cmds = sharp_video_on_cmds;
	video_on_cmds_count = ARRAY_SIZE(sharp_video_on_cmds);
	display_on_cmds = renesas_display_on_cmds;
	display_on_cmds_count = ARRAY_SIZE(renesas_display_on_cmds);
	display_off_cmds = sharp_display_off_cmds;
	display_off_cmds_count = ARRAY_SIZE(sharp_display_off_cmds);

	PR_DISP_INFO("%s\n", __func__);
	return ret;
}

static const struct i2c_device_id pwm_i2c_id[] = {
	{ "pwm_i2c", 0 },
	{ }
};

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

	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;

	pinfo.lcd.primary_vsync_init = pinfo.yres;
	pinfo.lcd.primary_rdptr_irq = 0;
	pinfo.lcd.primary_start_pos = pinfo.yres +
		pinfo.lcd.v_back_porch + pinfo.lcd.v_front_porch - 1;

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
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.esc_byte_ratio = 2;

	ret = mipi_monarudo_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	strncat(ptype, "PANEL_ID_DLX_SONY_RENESAS", ptype_len);

	video_on_cmds = sony_video_on_cmds;
	video_on_cmds_count = ARRAY_SIZE(sony_video_on_cmds);
	display_on_cmds = renesas_display_on_cmds;
	display_on_cmds_count = ARRAY_SIZE(renesas_display_on_cmds);
	display_off_cmds = sony_display_off_cmds;
	display_off_cmds_count = ARRAY_SIZE(sony_display_off_cmds);

	return ret;
}

static int pwm_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	blk_pwm_client = client;

	return rc;
}

static struct i2c_driver pwm_i2c_driver = {
	.driver = {
		.name = "pwm_i2c",
		.owner = THIS_MODULE,
	},
	.probe = pwm_i2c_probe,
	.remove =  __exit_p( pwm_i2c_remove),
	.id_table =  pwm_i2c_id,
};
static void __exit pwm_i2c_remove(void)
{
	i2c_del_driver(&pwm_i2c_driver);
}

void __init monarudo_init_fb(void)
{

	platform_device_register(&msm_fb_device);

	if(panel_type != PANEL_ID_NONE) {
		msm_fb_register_device("mdp", &mdp_pdata);
		msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
		wa_xo = msm_xo_get(MSM_XO_TCXO_D0, "mipi");
	}
	msm_fb_register_device("dtv", &dtv_pdata);
#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
	platform_device_register(&wfd_panel_device);
	platform_device_register(&wfd_device);
#endif
}

static int __init monarudo_panel_init(void)
{
	int ret;

	if (panel_type == PANEL_ID_NONE) {
		PR_DISP_INFO("%s panel ID = PANEL_ID_NONE\n", __func__);
		return 0;
	}

	ret = i2c_add_driver(&pwm_i2c_driver);

	if (ret)
		pr_err(KERN_ERR "%s: failed to add i2c driver\n", __func__);

	if (panel_type == PANEL_ID_DLX_SHARP_RENESAS) {
		mipi_video_sharp_init();
		PR_DISP_INFO("match PANEL_ID_DLX_SHARP_RENESAS panel_type\n");
	} else if (panel_type == PANEL_ID_DLX_SONY_RENESAS) {
		mipi_video_sony_init();
		PR_DISP_INFO("match PANEL_ID_DLX_SONY_RENESAS panel_type\n");
	} else {
		PR_DISP_INFO("Mis-match panel_type = %x\n", panel_type);
		return -ENODEV;
	}

	PR_DISP_INFO("%s\n", __func__);

	return platform_driver_register(&this_driver);
}
device_initcall_sync(monarudo_panel_init);
