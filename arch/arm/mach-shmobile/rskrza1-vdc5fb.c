/*
 * Frame Buffer Device Driver for VDC5
 *
 * Copyright (C) 2013-2014 Renesas Solutions Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <mach/common.h>
#include <mach/irqs.h>
#include <video/vdc5fb.h>
#include <mach/r7s72100.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
/* For the LCD-KIT-B01 */
#include <linux/input/ili210x.h>
/* For the R0P772LE0011RL */
#include <linux/i2c/tsc2007.h>

/*************************************************************************/

/* COMMON MACROS */
#define	P1CLK			((13330000 * 30) / 6)
#define	PIXCLOCK(hz, div)	\
	(u32)(1000000000000 / ((double)(hz) / (double)(div)))

/* KERNEL BOOT OPTIONS */
int disable_ether;
static int disable_sdhi;
static unsigned int vdc5fb0_opts = 1;
static unsigned int vdc5fb1_opts;

int __init early_vdc5fb0(char *str)
{
	get_option(&str, &vdc5fb0_opts);
	return 0;
}
early_param("vdc5fb0", early_vdc5fb0);

int __init early_vdc5fb1(char *str)
{
	get_option(&str, &vdc5fb1_opts);
	if (vdc5fb1_opts != 0) {
		disable_ether = 1;
		disable_sdhi = 1;
	}
	return 0;
}
early_param("vdc5fb1", early_vdc5fb1);

/*************************************************************************/

/* RESOURCES */
static struct resource vdc5fb_resources_ch0[VDC5FB_NUM_RES] = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcff6000, 0x00002000, "vdc5fb.0: reg"),
	[1] = DEFINE_RES_MEM_NAMED(0x60200000, 0x00400000, "vdc5fb.0: fb"),
	[2] = DEFINE_RES_NAMED(75, 23, "vdc5fb.0: irq", IORESOURCE_IRQ),
};
static struct resource vdc5fb_resources_ch1[VDC5FB_NUM_RES] = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcff8000, 0x00002000, "vdc5fb.1: reg"),
	[1] = DEFINE_RES_MEM_NAMED(0x60600000, 0x00300000, "vdc5fb.1: fb"),
/* The last 1MB (0x60900000-0x609fffff) is reserved by librzjpeg (JCU). */
	[2] = DEFINE_RES_NAMED(99, 23, "vdc5fb.1: irq", IORESOURCE_IRQ),
};

/*************************************************************************/

static struct fb_videomode videomode_xga = {
	.name		= "XGA",
	.refresh	= 61,	/* calculated */
	.xres		= 1024,
	.yres		= 768,
	.pixclock	= PIXCLOCK(P1CLK, 1),
	.left_margin	= 32,	/* 160, */
	.right_margin	= 152,	/* 24, */
	.upper_margin	= 29,
	.lower_margin	= 3,
	.hsync_len	= 136,
	.vsync_len	= 6,
	.sync		= 0,
	.vmode		= FB_VMODE_NONINTERLACED,
	.flag		= 0,
};

static struct vdc5fb_pdata vdc5fb_pdata_ch0_vga = {
	.name			= "VESA VGA",
	.videomode		= &videomode_xga,
	.panel_icksel		= ICKSEL_P1CLK,
	.bpp			= 16,
	.panel_width		= 0,	/* unused */
	.panel_height		= 0,	/* unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB888,
	.use_lvds		= 0,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_STH,		/* HSYNC */
		[LCD_TCON1]	= TCON_SEL_STVA,	/* VSYNC */
		[LCD_TCON2]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON3]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON4]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON5]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON6]	= TCON_SEL_UNUSED,	/* NC */
	},
};

static struct vdc5fb_pdata vdc5fb_pdata_ch1_vga = {
	.name			= "VESA VGA",
	.videomode		= &videomode_xga,
	.panel_icksel		= ICKSEL_P1CLK,
	.bpp			= 16,
	.panel_width		= 0,	/* unused */
	.panel_height		= 0,	/* unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB888,
	.use_lvds		= 0,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON1]	= TCON_SEL_STVA,	/* VSYNC */
		[LCD_TCON2]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON3]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON4]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON5]	= TCON_SEL_STH,		/* HSYNC */
		[LCD_TCON6]	= TCON_SEL_UNUSED,	/* NC */
	},
};

/*************************************************************************/

static struct fb_videomode videomode_wvga_lcd_kit_b01 = {
	.name		= "WVGA",
	.refresh	= 60,	/* unsued */
	.xres		= 800,
	.yres		= 480,
	.pixclock	= PIXCLOCK(P1CLK, 2),
	.left_margin	= 0,
	.right_margin	= 64,
	.upper_margin	= 18,
	.lower_margin	= 18,
	.hsync_len	= 128,
	.vsync_len	= 4,
	.sync		= 0,	/* to be fixed */
	.vmode		= 0,	/* to be fixed */
	.flag		= 0,	/* to be fixed */
};

static struct vdc5fb_pdata vdc5fb_pdata_ch0_lcd_kit_b01 = {
	.name			= "LCD-KIT-B01",
	.videomode		= &videomode_wvga_lcd_kit_b01,
	.panel_icksel		= ICKSEL_P1CLK,
	.bpp			= 32,
	.panel_width		= 184,	/* mm, unused */
	.panel_height		= 132,	/* mm, unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB666,
	.use_lvds		= 0,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_UNUSED,	/* RESET */
		[LCD_TCON1]	= TCON_SEL_UNUSED,	/* INT */
		[LCD_TCON2]	= TCON_SEL_DE,		/* DE */
		[LCD_TCON3]	= TCON_SEL_STH,		/* HSYNC(NC) */
		[LCD_TCON4]	= TCON_SEL_STVA,	/* VSYNC(NC) */
		[LCD_TCON5]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON6]	= TCON_SEL_UNUSED,	/* NC */
	},
};

static struct vdc5fb_pdata vdc5fb_pdata_ch1_lcd_kit_b01 = {
	.name			= "LCD-KIT-B01",
	.videomode		= &videomode_wvga_lcd_kit_b01,
	.panel_icksel		= ICKSEL_P1CLK,
	.bpp			= 32,
	.panel_width		= 184,	/* mm, unused */
	.panel_height		= 132,	/* mm, unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB666,
	.use_lvds		= 0,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_STH,		/* HSYNC(NC) */
		[LCD_TCON1]	= TCON_SEL_DE,		/* DE */
		[LCD_TCON2]	= TCON_SEL_STVA,	/* VSYNC(NC) */
		[LCD_TCON3]	= TCON_SEL_UNUSED,	/* INT */
		[LCD_TCON4]	= TCON_SEL_UNUSED,	/* RESET */
		[LCD_TCON5]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON6]	= TCON_SEL_UNUSED,	/* NC */
	},
};

/*************************************************************************/

static struct fb_videomode videomode_wvga_r0p7724le0011rl = {
	.name		= "WVGA",
	.refresh	= -1,	/* not fixed */
	.xres		= 800,
	.yres		= 480,
	.pixclock	= PIXCLOCK(P1CLK, 2),
	.left_margin	= 64,
	.right_margin	= 128,
	.upper_margin	= 11,
	.lower_margin	= 22,
	.hsync_len	= 64,
	.vsync_len	= 12,
#if 1
	.sync		= 0,
	.vmode		= 0,
#else
	.sync		= (FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT),
	.vmode		= FB_VMODE_NONINTERLACED,
#endif
	.flag		= 0,
};

static struct vdc5fb_pdata vdc5fb_pdata_r0p7724le0011rl = {
	.name			= "R0P7724LE0011RL",
	.videomode		= &videomode_wvga_r0p7724le0011rl,
	.panel_icksel		= ICKSEL_P1CLK,
	.bpp			= 32,
	.panel_width		= 165,	/* mm, unused */
	.panel_height		= 106,	/* mm, unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB666,
	.use_lvds		= 0,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_UNUSED,	/* NC */
		[LCD_TCON1]	= TCON_SEL_UNUSED,	/* NC*/
		[LCD_TCON2]	= TCON_SEL_UNUSED,	/* NC*/
		[LCD_TCON3]	= TCON_SEL_STVA,	/* LCDVSYN */
		[LCD_TCON4]	= TCON_SEL_STH,		/* LCDHSYN */
		[LCD_TCON5]	= TCON_SEL_DE,		/* LCDDISP(DE) */
		[LCD_TCON6]	= TCON_SEL_UNUSED,	/* TP_IRQ# */
	},
};

/*************************************************************************/

/* SETUP */
static struct platform_device vdc5fb_devices[VDC5FB_NUM_CH] = {
	[0] = {
		.name		= "vdc5fb",
		.id		= 0,
		.num_resources	= ARRAY_SIZE(vdc5fb_resources_ch0),
		.resource	= vdc5fb_resources_ch0,
		.dev = {
			.dma_mask		= NULL,
			.coherent_dma_mask	= 0xffffffff,
			.platform_data		= &vdc5fb_pdata_ch0_lcd_kit_b01,
		},
	},
	[1] = {
		.name		= "vdc5fb",
		.id		= 1,
		.num_resources	= ARRAY_SIZE(vdc5fb_resources_ch1),
		.resource	= vdc5fb_resources_ch1,
		.dev = {
			.dma_mask		= NULL,
			.coherent_dma_mask	= 0xffffffff,
			.platform_data		= &vdc5fb_pdata_ch1_vga,
		},
	},
};

static struct platform_device *display_devices[] __initdata = {
	&vdc5fb_devices[0],
	&vdc5fb_devices[1],
};

/*************************************************************************/

static struct ili210x_platform_data i2c0_ili210x_pdata = {
	.irq_flags		= IRQF_TRIGGER_LOW,	/* unused */
	.poll_period		= 20,
	.get_pendown_state	= NULL,
};

static struct ili210x_platform_data i2c3_ili210x_pdata = {
	.irq_flags		= IRQF_TRIGGER_HIGH,
	.poll_period		= 20,
	.get_pendown_state	= NULL,
};

static struct i2c_board_info i2c0_ili210x_devices[] = {
	[0] = {
		I2C_BOARD_INFO("lcd_kit_b01", 0x42),
		.platform_data	= NULL,
	},
	[1] = {
		I2C_BOARD_INFO("ili210x", 0x41),
		.irq		= 584,	/* P11_13 */
		.platform_data	= &i2c0_ili210x_pdata,
	},
};

static struct i2c_board_info i2c3_ili210x_devices[] = {
	[0] = {
		I2C_BOARD_INFO("lcd_kit_b01", 0x42),
		.platform_data	= NULL,
	},
	[1] = {
		I2C_BOARD_INFO("ili210x", 0x41),
		.irq		= 32,	/* IRQ0 */
		.platform_data	= &i2c3_ili210x_pdata,
	},
};

static struct tsc2007_platform_data i2c_tsc2007_pdata = {
	.model			= 2007,
	.x_plate_ohms		= 180,
	.max_rt			= 0,    /* u16 */
	.poll_period		= 0,    /* unsigned long */
	.fuzzx			= 0,    /* int */
	.fuzzy			= 0,    /* int */
	.fuzzz			= 0,    /* int */
	.get_pendown_state	= NULL, /* int (*)(void) */
	.clear_penirq		= NULL, /* void (*)(void) */
	.init_platform_hw	= NULL, /* int (*)(void) */
	.exit_platform_hw	= NULL, /* void (*)(void) */
};

static struct i2c_board_info i2c3_tsc2007_devices[] = {
	[0] = {
		I2C_BOARD_INFO("tsc2007", 0x48),
		.flags		= 0,
		.archdata	= NULL,
		.of_node	= NULL,
		.irq		= 35,	/* IRQ2 */
		.platform_data	= &i2c_tsc2007_pdata,
	},
};

/*************************************************************************/

static int vdc5fb_setup(void)
{
	struct platform_device *pdev;
	int n;

	pdev = &vdc5fb_devices[0];
	if (pdev->id == 0) {		/* VDC5 CHANNEL 0 */
		switch (vdc5fb0_opts) {
		case 0:	/* Turn off */
			pr_info("vdc5fb.%d: channel 0 is turned off\n",
				pdev->id);
			pdev->name = "vdc5fb.0(hidden)";
			break;
		case 1:	/* LCD_KIT_B01 (default) */
			pdev->dev.platform_data =
				&vdc5fb_pdata_ch0_lcd_kit_b01;
			break;
		case 2:	/* LCD Monitor (VGA) */
			pdev->dev.platform_data =
				&vdc5fb_pdata_ch0_vga;
			break;
		case 3:	/* Add channel 1 first */
			break;
		default:
			break;
		}
	}
	pdev = &vdc5fb_devices[1];
	if (pdev->id == 1) {	/* VDC5 CHANNEL 1 */
		switch (vdc5fb1_opts) {
		case 0:	/* Turn off (default) */
			pr_info("vdc5fb.%d: channel 1 is turned off\n",
				pdev->id);
			pdev->name = "vdc5fb.1(hidden)";
			break;
		case 1:	/* LCD-KIT-B01 */
			pdev->dev.platform_data =
				&vdc5fb_pdata_ch1_lcd_kit_b01;
			break;
		case 2:	/* R0P7724LE0011RL */
			pdev->dev.platform_data =
				&vdc5fb_pdata_r0p7724le0011rl;
			break;
		case 3:	/* LCD monitor */
			pdev->dev.platform_data =
				&vdc5fb_pdata_ch1_vga;
			break;
		default:
			break;
		}
	}
	platform_add_devices(display_devices, ARRAY_SIZE(display_devices));

	n = 1;
	switch (vdc5fb0_opts) {
	case 9:
		n = ARRAY_SIZE(i2c0_ili210x_devices);
		/* FALL THROUGH... */
	case 1:
		i2c_register_board_info(0, i2c0_ili210x_devices, n);
		/* Setup LCD0_TCON1 (INT) as PORT */
/*		rza1_pfc_pin_assign(P11_13, PMODE, DIR_IN);	*/
		break;
	default:
		break;
	}
	n = 1;
	switch (vdc5fb1_opts) {
	case 9:
		n = ARRAY_SIZE(i2c3_ili210x_devices);
		/* FALL THROUGH... */
	case 1:
		i2c_register_board_info(3, i2c3_ili210x_devices, n);
		/* Setup LCD1_TCON3 (INT) as IRQ0 */
/*		rza1_pfc_pin_assign(P4_8, ALT8, DIIO_PBDC_DIS);	*/
		break;
	case 10:
		n = ARRAY_SIZE(i2c3_tsc2007_devices);
		/* FALL THROUGH... */
	case 2:
		/* Setup LCD1_TCON6 (TP_IRQ#) as IRQ3 */
/*		rza1_pfc_pin_assign(P4_11, ALT8, DIIO_PBDC_DIS);	*/
		i2c_register_board_info(3, i2c3_tsc2007_devices, n);
		break;

	default:
		break;
	}

	return 0;
}

/*************************************************************************/