/*
 * GRPEACH board support
 *
 * Copyright (C) 2016  Renesas Electronics
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
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/sh_eth.h>
#include <asm/mach/map.h>
#include "common.h"
#include "irqs.h"
#include "r7s72100.h"
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/spi/rspi.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/serial_sci.h>
#include <linux/i2c.h>
#include <linux/i2c-riic.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sh_mmcif.h>
#include <linux/mmc/sh_mobile_sdhi.h>
#include <linux/mfd/tmio.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/platform_data/at24.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_data/sh_adc.h>
#include <linux/usb/r8a66597.h>
#include <linux/platform_data/dma-rza1.h>
#include <linux/input/edt-ft5x06.h>
#include <linux/uio_driver.h>
#include <clocksource/sh_ostm.h>
#include <video/vdc5fb.h>
#include <sound/sh_scux.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <media/sh_mobile_ceu.h>
#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>
#include <media/ov7670.h>
#include <linux/platform_data/simplefb.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/can/platform/rza1_can.h>
#include <linux/clk.h>

/* If an XIP kernel has its memory address set to the begining
   of internal RAM, we'll assume that external SDRAM is not
   availible and we should try to reserve RAM when possible */
#define XIP_KERNEL_WITHOUT_EXTERNAL_RAM (defined(CONFIG_XIP_KERNEL) && (CONFIG_PHYS_OFFSET == 0x20000000))

/* Board Options */

/*
 * early_usbgs()
 * - This allows you to select if you want to use
 *   USB gadget or not.
 * - Optional: This was just an easy way to switch this for testing
 */
static int usbgs = -1;
static int __init early_usbgs(char *str)
{
	usbgs = 0;
	get_option(&str, &usbgs);
	return 0;
}
early_param("usbgs", early_usbgs);

/* ==========================================================
 *			IO Map Section
 *
 * Maps physical addresses to virtual addresses
 * ==========================================================*/
static struct map_desc rza1_io_desc[] __initdata = {
	/* create a 1:1 entity map for 0xe8xxxxxx
	 * used by INTC.
	 */
	{
		.virtual	= 0xe8000000,
		.pfn		= __phys_to_pfn(0xe8000000),
		.length		= SZ_256M,
		.type		= MT_DEVICE_NONSHARED
	},
	/* create a 1:1 entity map for 0xfcfexxxx
	 * used by MSTP, CPG.
	 */
	{
		.virtual	= 0xfcfe0000,
		.pfn		= __phys_to_pfn(0xfcfe0000),
		.length		= SZ_64K,
		.type		= MT_DEVICE_NONSHARED
	},
#ifdef CONFIG_CACHE_L2X0
	/* create a 1:1 entity map for 0x3ffffxxx
	 * used by L2CC (PL310).
	 */
	{
		.virtual	= 0xfffee000,
		.pfn		= __phys_to_pfn(0x3ffff000),
		.length		= SZ_4K,
		.type		= MT_DEVICE_NONSHARED
	},
#endif
};

/*
 * rza1_map_io()
 * - Called early to set up mappings
 * - Call twice during boot (before and after MMU setup)
 */
void __init r7s72100_map_io(void)
{
#ifdef CONFIG_DEBUG_LL
	/* Note: Becase we defined a .map_io handler, we must manually set our
	   SCIF2 memory mapping here. see arch/arm/mm/mmu.c */
	debug_ll_io_init();
#endif
	iotable_init(rza1_io_desc, ARRAY_SIZE(rza1_io_desc));
}

/* ==========================================================
 *			DMA Section
 *
 * Defines RZ/A1 DMA channels
 * ==========================================================*/
#define CHCFG(reqd_v, loen_v, hien_v, lvl_v, am_v, sds_v, dds_v, tm_v)\
	{								\
		.reqd	=	reqd_v,					\
		.loen	=	loen_v,					\
		.hien	=	hien_v,					\
		.lvl	=	lvl_v,					\
		.am	=	am_v,					\
		.sds	=	sds_v,					\
		.dds	=	dds_v,					\
		.tm	=	tm_v,					\
	}
#define DMARS(rid_v, mid_v)	\
	{								\
		.rid	= rid_v,					\
		.mid	= mid_v,					\
	}

static const struct rza1_dma_slave_config rza1_dma_slaves[] = {
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI0_TX,
		.addr		= 0xe804e030,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x30),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI0_RX,
		.addr		= 0xe804e030,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x30),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI1_TX,
		.addr		= 0xe804e830,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x31),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI1_RX,
		.addr		= 0xe804e830,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x31),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_MMCIF_TX,
		.addr		= 0xe804c834,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x32),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_MMCIF_RX,
		.addr		= 0xe804c834,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x32),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_PCM_MEM_SSI0,
		.addr		= 0xe820b018,		/* SSIFTDR_0 */
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x38),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_MEM_SRC1,
		.addr		= 0xe820970c,		/* DMATD1_CIM */
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0),
		.dmars		= DMARS(0x1, 0x41),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_SSI0_MEM,
		.addr		= 0xe820b01c,		/* SSIFRDR_0 */
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x38),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_SRC0_MEM,
		.addr		= 0xe8209718,		/* DMATU0_CIM */
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0),
		.dmars		= DMARS(0x2, 0x40),
	},
};

static const struct rza1_dma_pdata dma_pdata __initconst = {
	.slave		= rza1_dma_slaves,
	.slave_num	= ARRAY_SIZE(rza1_dma_slaves),
#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
	.channel_num	= 6,	/* Less channels means less RAM (2 for SDHI, 4 for Audio) */
#else
	.channel_num	= 16,	/* 16 MAX channels */
#endif
};

static const struct resource rza1_dma_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8200000, 0x1000),
	DEFINE_RES_MEM(0xfcfe1000, 0x1000),
	DEFINE_RES_NAMED(gic_iid(41), 16, NULL, IORESOURCE_IRQ),
	DEFINE_RES_IRQ(gic_iid(57)),	/* DMAERR */
};

static const struct platform_device_info dma_info  __initconst = {
	.name		= "rza1-dma",
	.id		= -1,
	.res		= rza1_dma_resources,
	.num_res	= ARRAY_SIZE(rza1_dma_resources),
	.data		= &dma_pdata,
	.size_data	= sizeof(dma_pdata),
};
/* #define use_LCD0 */
#ifdef use_LCD0
/* ==========================================================
 *			LCD 0 Video Section
 * Defines VDC5 ch0 LCD controller
 * Board Specific Portions:
 * 	- main clock speed
 * 	- Pin mux setup for Parallel LCD
 *	- LCD panel description
 *	- Frame buffer layers
 * ==========================================================*/

/* BOARD: Change if not using 13.33MHz main crystal */
#define	P1CLK			((13330000 * 30) / 6)
/* pixclock is expressed in ps (pico seconds) */
#define	PIXCLOCK(hz, div)	\
	(u32)(1000000000000 / ((double)(hz) / (double)(div)))

struct pfc_pinmux_assign {
	int port;	/* enum */
	int mode;	/* enum */
	int opts;
};

/* BOARD: Change if your [LCD0] is on different pins */
static struct pfc_pinmux_assign lcd0_common[] = {
	{ P11_15, ALT5, },	/* LCD0_CLK */
	{ P11_7,  ALT5, },	/* LCD0_DATA0 */
	{ P11_6,  ALT5, },	/* LCD0_DATA1 */
	{ P11_5,  ALT5, },	/* LCD0_DATA2 */
	{ P11_4,  ALT5, },	/* LCD0_DATA3 */
	{ P11_3,  ALT5, },	/* LCD0_DATA4 */
	{ P11_2,  ALT5, },	/* LCD0_DATA5 */
	{ P11_1,  ALT5, },	/* LCD0_DATA6 */
	{ P11_0,  ALT5, },	/* LCD0_DATA7 */
	{ P10_15, ALT5, },	/* LCD0_DATA8 */
	{ P10_14, ALT5, },	/* LCD0_DATA9 */
	{ P10_13, ALT5, },	/* LCD0_DATA10 */
	{ P10_12, ALT5, },	/* LCD0_DATA11 */
	{ P10_11, ALT5, },	/* LCD0_DATA12 */
	{ P10_10, ALT5, },	/* LCD0_DATA13 */
	{ P10_9,  ALT5, },	/* LCD0_DATA14 */
	{ P10_8,  ALT5, },	/* LCD0_DATA15 */
	{ P10_7,  ALT5, },	/* LCD0_DATA16 */
	{ P10_6,  ALT5, },	/* LCD0_DATA17 */
	{ P10_5,  ALT5, },	/* LCD0_DATA18 */
	{ P10_4,  ALT5, },	/* LCD0_DATA19 */
	{ P10_3,  ALT5, },	/* LCD0_DATA20 */
	{ P10_2,  ALT5, },	/* LCD0_DATA21 */
	{ P10_1,  ALT5, },	/* LCD0_DATA22 */
	{ P10_0,  ALT5, },	/* LCD0_DATA23 */
};

/* BOARD: Change if your [LCD0] is on different pins */
/* NOTE: Unused TCONS will be left as GPIO */
static struct pfc_pinmux_assign lcd0_tcon[] = {
	{ P11_14, ALT5, },	/* LCD0_TCON0 */
	{ P11_13, ALT5, },	/* LCD0_TCON1 */
	{ P11_12, ALT5, },	/* LCD0_TCON2 */
	{ P11_11, ALT5, },	/* LCD0_TCON3 */
	{ P11_10, ALT5, },	/* LCD0_TCON4 */
	{ P11_9,  ALT5, },	/* LCD0_TCON5 */
	{ P11_8,  ALT5, },	/* LCD0_TCON6 */
};

/* LCD Frame buffer Declaration
VDC5_0_BPP:
	This is the Frame buffer size your application will access (/dev/fb0)
	Set to either '16' or '32'
	Note that the color depth of your frame buffer does not have to be the same
	color depth of your panel. For example, even if you have a full 24-bit LCD panel,
	you can still set this to 16 or lower.

VDC5_0_FBSIZE:
	Set to the total BYTE size of your frame buffer.

VDC5_0_FB_ADDR:
	Set this to the location of your frame buffer in INTERNAL RAM.
	You can either hard code this address, or set it to 0 and it will
	be allocated for you during 'probe' (but only if using internal RAM
	only and your frame buffer is under 1MB).
*/

/* BOARD: Define your [LCD0] frame buffer location and size. */
#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
  #define VDC5_0_BPP 16 /* 16bpp or 32bpp */
  #define VDC5_0_FBSIZE (800*480*VDC5_0_BPP/8)
  #define VDC5_0_FB_ADDR 0	/* allcoate at probe */
#else
  #define VDC5_0_BPP 32 /* 16bpp or 32bpp */
  #define VDC5_0_FBSIZE (800*480*VDC5_0_BPP/8)

  /* Assume we are using external SDRAM for system memory so we have all
     the internal memory just for our LCD frame buffer */
  /* Place at end of internal RAM, but on a PAGE boundry */
  //#define VDC5_0_FB_ADDR ((0x20A00000 - VDC5_0_FBSIZE) & PAGE_MASK)

  /* Fixed allocation */
  #define VDC5_0_FB_ADDR (0x60000000) /* PAGE 0 (2MB) */
#endif

static const struct resource vdc5fb0_resources[VDC5FB_NUM_RES] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcff6000, 0x00002000, "vdc5fb.0: reg"),
	[1] = DEFINE_RES_MEM_NAMED(VDC5_0_FB_ADDR, VDC5_0_FBSIZE, "vdc5fb.0: fb"),
	[2] = DEFINE_RES_NAMED(75, 23, "vdc5fb.0: irq", IORESOURCE_IRQ),
};

static int vdc5fb_0_pinmux(struct platform_device *pdev)
{
	size_t n, total;
	struct pfc_pinmux_assign *pf;
	struct vdc5fb_pdata *pdata
	    = (struct vdc5fb_pdata *)pdev->dev.platform_data;

	/* Assign the LCD_0 pins */
	pf = lcd0_common;
	total = ARRAY_SIZE(lcd0_common);
	for (n = 0; n < total; pf++, n++)
		r7s72100_pfc_pin_assign(pf->port, pf->mode, DIIO_PBDC_DIS);

	/* Assing only the TCON_0 pins that will be used */
	pf = lcd0_tcon;
	total = ARRAY_SIZE(lcd0_tcon);
	for (n = 0; n < total; pf++, n++)
		if (pdata->tcon_sel[n] != TCON_SEL_UNUSED)
			r7s72100_pfc_pin_assign(pf->port, pf->mode, DIIO_PBDC_DIS);
	return 0;
}

/* This structure defines the panel timings */
/* BOARD: You should rename this strucutre to match your LCD panel */
static struct fb_videomode videomode_gwp0700cnwv04 = {
	.name		= "gwp0700cnwv04",
	.refresh	= 60,
	.xres		= 800,
	.yres		= 480,
	.pixclock	= PIXCLOCK(P1CLK, 2),
	.left_margin	= 210,	/* horizontal front porch */
	.right_margin	= 46,	/* horizontal back porch */
	.upper_margin	= 23,	/* vertical back porch */
	.lower_margin	= 22,	/* vertical front porch */
	.hsync_len	= 40,	/* max */
	.vsync_len	= 20,	/* max */
	.sync		= 0,
	.vmode		= 0,
	.flag		= 0,
};

#if 0 /* FLOATING LAYER SAMPLE */
/* Graphics 3 - Image Synthesizer */
/* Creates a 200x200 32-bit ARGB floating layer */

#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
uint8_t gr3_fb[ 200*200*4 ] __attribute__ ((aligned (PAGE_SIZE)));
#else
#define gr3_fb 0x60600000 /* hard coded */
#endif
#endif

/* BOARD: You should rename this strucutre to match your LCD panel */
/* This structure passing info to the VDC5 driver */
static const struct vdc5fb_pdata vdc5fb_gwp0700cnwv04_pdata = {
	.name			= "gwp0700cnwv04",
	.videomode		= &videomode_gwp0700cnwv04,
	.panel_icksel		= ICKSEL_P1CLK,	/* see include/video/vdc5fb.h */
	.bpp			= VDC5_0_BPP,
	.panel_width		= 154,	/* mm, unused */
	.panel_height		= 86,	/* mm, unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB888,
	.use_lvds		= 0,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_UNUSED,	/* MODE */
		[LCD_TCON1]	= TCON_SEL_UNUSED,	/* DITH */
		[LCD_TCON2]	= TCON_SEL_DE,		/* RGB_EN */
		[LCD_TCON3]	= TCON_SEL_STH,		/* RGB_HSYNC */
		[LCD_TCON4]	= TCON_SEL_STVA,	/* RGB_VSYNC */
		[LCD_TCON5]	= TCON_SEL_UNUSED,	/* LR_INV */
		[LCD_TCON6]	= TCON_SEL_UNUSED,	/* UD_INV */
	},
	.pinmux			= vdc5fb_0_pinmux,
	.layers			= {
		/* Graphics 2 - Image Synthesizer */
		/* Full LCD Panel - will be /dev/fb0 */
		[2].xres	= 800,
		[2].yres	= 480,
		[2].x_offset	= 0,
		[2].y_offset	= 0,
#if VDC5_0_BPP == 16
		[2].format	= GR_FORMAT(GR_FORMAT_RGB565) | GR_RDSWA(6),
#else
		[2].format	= GR_FORMAT(GR_FORMAT_ARGB8888) | GR_RDSWA(4),
#endif
		[2].bpp		= VDC5_0_BPP,
		[2].base	= VDC5_0_FB_ADDR,
		[2].blend	 = 0,

#if 0 /* FLOATING LAYER SAMPLE */
		/* Graphics 3 - Image Synthesizer */
		/* Creates a 200x200 32-bit ARGB floating layer */
		[3].xres	= 200,
		[3].yres	= 200,
		[3].x_offset	= 100,
		[3].y_offset	= 50,
		[3].format	= GR_FORMAT(GR_FORMAT_ARGB8888) | GR_RDSWA(4),
		[3].bpp		= 32,
		[3].base	= (u32)gr3_fb,
		[3].blend	= 1,
#endif
	},
};

static const struct platform_device_info vdc5fb0_info __initconst = {
	.name		= "vdc5fb",
	.id		= 0,	/* ch 0 */
	.res		= vdc5fb0_resources,
	.num_res	= ARRAY_SIZE(vdc5fb0_resources),
	.data		= &vdc5fb_gwp0700cnwv04_pdata,
	.size_data	= sizeof(vdc5fb_gwp0700cnwv04_pdata),
	.dma_mask	= DMA_BIT_MASK(32),	/* only needed if not hardcoding fb */
};

#if 0 /* FLOATING LAYER SAMPLE */
	/* Graphics 3 - Image Synthesizer */
	/* Map our floating layer as /dev/fb1 */
  #ifndef CONFIG_FB_SIMPLE
    #error Requires CONFIG_FB_SIMPLE=y
  #endif

/* simple-framebuffer */
static const struct resource simplefb_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(gr3_fb, 200*200*4, "vdc5_alpha.0: fb"),
};
static const struct simplefb_platform_data simplefb_pdata = {
	.width = 200,
	.height = 200,
	.stride = 200 * 4,
	.format = "a8r8g8b8",
};
static const struct platform_device_info simplefb_info __initconst = {
	.parent		= &platform_bus,
	.name		= "simple-framebuffer",
	.id		= -1,
	.res		= simplefb_resources,
	.num_res	= ARRAY_SIZE(simplefb_resources),
	.data		= &simplefb_pdata,
	.size_data	= sizeof(simplefb_pdata),
};
#endif /* FLOATING LAYER SAMPLE */
#endif

#define USE_LVDS
#ifdef USE_LVDS 
/* ==========================================================
 *			LCD 1 Video Section
 * Defines VDC5 ch1 LCD controller (RZ/A1M and RZ/A1H only)
 * Board Specific Portions:
 *	- Assumes LVDS
 * 	- main clock speed
 * 	- Pin mux setup for Parallel LCD
 *	- LCD panel description
 *	- Frame buffer layers
 * ==========================================================*/

/* BOARD: Change if not using 13.33MHz main crystal */
#define	P1CLK			((13330000 * 30) / 6)
/* pixclock is expressed in ps (pico seconds) */
#define	PIXCLOCK(hz, div)	\
	(u32)(1000000000000 / ((double)(hz) / (double)(div)))

/* LCD 1 Frame buffer Declaration */
/* BOARD: Define your [LCD1 LVDS] frame buffer location and size.
VDC5_1_BPP, VDC5_1_FBSIZE, VDC5_1_FB_ADDR:
	See VDC5_1_XXX for more info
*/

/* NOTE: If you want to use VDC5 ch0 and ch1 at the same time, you
   need to put their frame buffers in different RAM Pages */
#define VDC5_1_BPP 16
#define VDC5_1_FBSIZE (800*480*VDC5_1_BPP/8)
#define VDC5_1_FB_ADDR 0 
static const struct resource vdc5fb1_resources[VDC5FB_NUM_RES] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcff8000, 0x00002000, "vdc5fb.1: reg"),
	[1] = DEFINE_RES_MEM_NAMED(VDC5_1_FB_ADDR, VDC5_1_FBSIZE, "vdc5fb.1: fb"),
	[2] = DEFINE_RES_NAMED(99, 23, "vdc5fb.1: irq", IORESOURCE_IRQ),
};

static int vdc5fb_pinmux_lvds(struct platform_device *pdev)
{
	/* When using the LVDS pins, PIPCn.PIPCnm bits should be Set to 0
	   and the port direction should be set as input. See Table 54.7 */
	r7s72100_pfc_pin_assign(P5_0, ALT1, DIR_IN); /* TXCLKOUTP */
	r7s72100_pfc_pin_assign(P5_1, ALT1, DIR_IN); /* TXCLKOUTM */
	r7s72100_pfc_pin_assign(P5_2, ALT1, DIR_IN); /* TXOUT2P */
	r7s72100_pfc_pin_assign(P5_3, ALT1, DIR_IN); /* TXOUT2M */
	r7s72100_pfc_pin_assign(P5_4, ALT1, DIR_IN); /* TXOUT1P */
	r7s72100_pfc_pin_assign(P5_5, ALT1, DIR_IN); /* TXOUT1M */
	r7s72100_pfc_pin_assign(P5_6, ALT1, DIR_IN); /* TXOUT0P */
	r7s72100_pfc_pin_assign(P5_7, ALT1, DIR_IN); /* TXOUT0M */

	return 0;
}

static int vdc5fb_1_pinmux(struct platform_device *pdev)
{
	vdc5fb_pinmux_lvds( pdev );
	return 0;
}

/* LVDS Panel: AM800480RBTMQW-TA3H| 800x480 (WVGA), 7", LVDS, 16bpp */
static struct fb_videomode videomode_AM800480RBTMQW = {
	.name		= "AM800480RBTMQW",
	.refresh	= 60,			/* not fixed */
	.xres		= 800,
	.yres		= 480,
	.pixclock	= PIXCLOCK(71100000,1),	/* min:64.3MHz, typ:71.1MHz, max:82MHz */
	.left_margin	= 128,		/* horizontal back porch */
	.right_margin	= 92, 		/* horizontal front porch */
	.upper_margin	= 35, 		/* vertical back porch */
	.lower_margin	= 5, 		/* vertical front porch */
	.hsync_len	= 36, 		/* Horizontal Blanking Time */
	.vsync_len	= 5, 		/* Vertical Blanking Time */
	.sync		= FB_SYNC_HOR_HIGH_ACT, /* 0 Polarity Inversion Control of STH Signal*/
	.vmode		= 0,
	.flag		= 0,
};

static const struct vdc5fb_pdata vdc5fb_AM800480RBTMQW_pdata = {
	.name			= "AM800480RBTMQW",
	.videomode		= &videomode_AM800480RBTMQW,
	.panel_ocksel		= OCKSEL_PLL_DIV7,	/* see include/video/vdc5fb.h */
	.bpp			= VDC5_1_BPP,
	.panel_width		= 0,			/* mm, unused */
	.panel_height		= 0,			/* mm, unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB888,
	.use_lvds		= 1,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_STVA, 	/* VSYNC (for LVDS, VS must be TCON0) */
		[LCD_TCON1]	= TCON_SEL_UNUSED,	/* (not connected to LVDS circuit) */
		[LCD_TCON2]	= TCON_SEL_STH, 	/* HSYNC (for LVDS, HS must be TCON2) */
		[LCD_TCON3]	= TCON_SEL_DE,		/* DE (for LVDS, DE must be TCON3) */
		[LCD_TCON4]	= TCON_SEL_UNUSED,	/* (not connected to LVDS circuit) */
		[LCD_TCON5]	= TCON_SEL_UNUSED,	/* (not connected to LVDS circuit) */
		[LCD_TCON6]	= TCON_SEL_UNUSED,	/* (not connected to LVDS circuit) */
	},
	.pinmux			= vdc5fb_1_pinmux,
	.layers			= {
		/* Graphics 2 - Image Synthesizer */
		/* Full LCD Panel - will be /dev/fb0 */
		[2].xres	= 800, 
		[2].yres	= 480, 
		[2].x_offset	= 0,
		[2].y_offset	= 0,
#if VDC5_1_BPP == 16
		[2].format	= GR_FORMAT(GR_FORMAT_RGB565) | GR_RDSWA(6),
#else
		[2].format	= GR_FORMAT(GR_FORMAT_ARGB8888) | GR_RDSWA(4),
#endif
		[2].bpp		= VDC5_1_BPP,
		[2].base	= VDC5_1_FB_ADDR,
		[2].blend	 = 0,
	},
};

static const struct platform_device_info vdc5fb1_info __initconst = {
	.name		= "vdc5fb",
	.id		= 1,			/* ch 1 */
	.res		= vdc5fb1_resources,
	.num_res	= ARRAY_SIZE(vdc5fb1_resources),
	.data		= &vdc5fb_AM800480RBTMQW_pdata,
	.size_data	= sizeof(vdc5fb_AM800480RBTMQW_pdata),
	.dma_mask	= DMA_BIT_MASK(32),	/* only needed if not hardcoding fb */
};

#endif /* USE_LVDS */

/* ==========================================================
 *                Ethernet Section
 * ==========================================================*/
static const struct sh_eth_plat_data ether_pdata __initconst = {
	.phy			= 0x00, /* PD60610 */
	.edmac_endian		= EDMAC_LITTLE_ENDIAN,
	.phy_interface		= PHY_INTERFACE_MODE_MII,
	.no_ether_link		= 1
};

static const struct resource ether_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8203000, 0x800),
	DEFINE_RES_MEM(0xe8204800, 0x200),
	DEFINE_RES_IRQ(gic_iid(359)),
};

static const struct platform_device_info ether_info __initconst = {
	.parent		= &platform_bus,
	.name		= "r7s72100-ether",
	.id		= -1,
	.res		= ether_resources,
	.num_res	= ARRAY_SIZE(ether_resources),
	.data		= &ether_pdata,
	.size_data	= sizeof(ether_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* ==========================================================
 *                     I2C Section
 * ==========================================================*/
/* I2C0*/
static const struct resource riic0_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcfee000, 0x400),
	DEFINE_RES_IRQ(gic_iid(189)),
	DEFINE_RES_NAMED(gic_iid(190),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_NAMED(gic_iid(191),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_IRQ(gic_iid(192)),
	DEFINE_RES_IRQ(gic_iid(193)),
	DEFINE_RES_IRQ(gic_iid(194)),
	DEFINE_RES_IRQ(gic_iid(195)),
	DEFINE_RES_IRQ(gic_iid(196)),
};

static const struct riic_platform_data riic0_pdata __initconst = {
	.bus_rate = 100,
};

static const struct platform_device_info riic0_info __initconst = {
	.parent		= &platform_bus,
	.name		= "i2c-riic",
	.id		= 0,
	.res		= riic0_resources,
	.num_res	= ARRAY_SIZE(riic0_resources),
	.data		= &riic0_pdata,
	.size_data	= sizeof(riic0_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* I2C1*/
static const struct resource riic1_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcfee400, 0x400),
	DEFINE_RES_IRQ(gic_iid(197)),
	DEFINE_RES_NAMED(gic_iid(198),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_NAMED(gic_iid(199),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_IRQ(gic_iid(200)),
	DEFINE_RES_IRQ(gic_iid(201)),
	DEFINE_RES_IRQ(gic_iid(202)),
	DEFINE_RES_IRQ(gic_iid(203)),
	DEFINE_RES_IRQ(gic_iid(204)),
};

static const struct riic_platform_data riic1_pdata __initconst = {
	.bus_rate = 100,
};

static const struct platform_device_info riic1_info __initconst = {
	.parent		= &platform_bus,
	.name		= "i2c-riic",
	.id		= 1,
	.res		= riic1_resources,
	.num_res	= ARRAY_SIZE(riic1_resources),
	.data		= &riic1_pdata,
	.size_data	= sizeof(riic1_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

static const struct resource riic2_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcfee800, 0x400),
	DEFINE_RES_IRQ(gic_iid(205)),
	DEFINE_RES_NAMED(gic_iid(206),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_NAMED(gic_iid(207),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_IRQ(gic_iid(208)),
	DEFINE_RES_IRQ(gic_iid(209)),
	DEFINE_RES_IRQ(gic_iid(210)),
	DEFINE_RES_IRQ(gic_iid(211)),
	DEFINE_RES_IRQ(gic_iid(212)),
};

static const struct riic_platform_data riic2_pdata __initconst = {
	.bus_rate = 100,
};

static const struct platform_device_info riic2_info __initconst = {
	.parent		= &platform_bus,
	.name		= "i2c-riic",
	.id		= 2,
	.res		= riic2_resources,
	.num_res	= ARRAY_SIZE(riic2_resources),
	.data		= &riic2_pdata,
	.size_data	= sizeof(riic2_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

static const struct resource riic3_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcfeec00, 0x400),
	DEFINE_RES_IRQ(gic_iid(213)),
	DEFINE_RES_NAMED(gic_iid(214),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_NAMED(gic_iid(215),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_IRQ(gic_iid(216)),
	DEFINE_RES_IRQ(gic_iid(217)),
	DEFINE_RES_IRQ(gic_iid(218)),
	DEFINE_RES_IRQ(gic_iid(219)),
	DEFINE_RES_IRQ(gic_iid(220)),
};

static const struct riic_platform_data riic3_pdata __initconst = {
	.bus_rate = 100,
};

static const struct platform_device_info riic3_info __initconst = {
	.parent		= &platform_bus,
	.name		= "i2c-riic",
	.id		= 3,
	.res		= riic3_resources,
	.num_res	= ARRAY_SIZE(riic3_resources),
	.data		= &riic3_pdata,
	.size_data	= sizeof(riic3_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* ==========================================================
 *		I2C Device Section
 *
 * Define devices that reside on I2C busses that
 * need to be discovered
 * ==========================================================*/

/* BOARD: This is the Capacitive touchscreen controller that
	  is on the GRPEACH board */
static struct edt_ft5x06_platform_data ft5216_pdata __initdata = {
	.irq_pin	= -1,
	.reset_pin	= -1,
};


static const struct i2c_board_info i2c0_devices[] __initconst = {
	{
		I2C_BOARD_INFO("ft5x06-ts", 0x38),
		.platform_data = &ft5216_pdata,
		.irq		= 33,
	},
};

static struct at24_platform_data eeprom_pdata = {
	.byte_len = 2048,
	.page_size = 16,
};

static const struct i2c_board_info i2c3_devices[] __initconst = {
	{
		I2C_BOARD_INFO("max9856", 0x10),
	},
	{
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &eeprom_pdata,
	},
};

/* ==========================================================
 *			OS Timer Section
 *
 * High precision OS Timer
 * When using this timer as a system time, you can get better
 * accuracy then the default MTU2 timer.
 * ==========================================================*/
static struct rza1_ostm_pdata ostm_pdata = {
	.clksrc.name = "ostm.0",
	.clksrc.rating = 300,
	.clkevt.name = "ostm.1",
	.clkevt.rating = 300,
};

static const struct resource ostm_resources[] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcfec000, 0x030, "ostm.0"),
	[1] = DEFINE_RES_MEM_NAMED(0xfcfec400, 0x030, "ostm.1"),
	[2] = DEFINE_RES_IRQ_NAMED(134, "ostm.0"),
	[3] = DEFINE_RES_IRQ_NAMED(135, "ostm.1"),
};

static const struct platform_device_info ostm_info __initconst = {
	.name		= "ostm",
	.id		= 0,
	.data 		= &ostm_pdata,
	.size_data	= sizeof(ostm_pdata),
	.res		= ostm_resources,
	.num_res	= ARRAY_SIZE(ostm_resources),
};

/* ==========================================================
 *			RTC Section
 * ==========================================================*/
static const struct resource rtc_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcff1000, 0x2e),
	DEFINE_RES_IRQ(gic_iid(309)),	/* Period IRQ */
	DEFINE_RES_IRQ(gic_iid(310)),	/* Carry IRQ */
	DEFINE_RES_IRQ(gic_iid(308)),	/* Alarm IRQ */
};

static const struct platform_device_info rtc_info __initconst = {
	.parent		= &platform_bus,
	.name		= "sh-rtc",
	.id		= -1,
	.res		= rtc_resources,
	.num_res	= ARRAY_SIZE(rtc_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* ==========================================================
 *		RSPI Section
 *
 * Standard SPI controller
 * ==========================================================*/
#define RSPI_RESOURCE(idx, baseaddr, irq)				\
static const struct resource rspi##idx##_resources[] __initconst = {	\
	DEFINE_RES_MEM(baseaddr, 0x24),					\
	DEFINE_RES_IRQ_NAMED(irq, "error"),				\
	DEFINE_RES_IRQ_NAMED(irq + 1, "rx"),				\
	DEFINE_RES_IRQ_NAMED(irq + 2, "tx"),				\
}

RSPI_RESOURCE(0, 0xe800c800, gic_iid(270));
RSPI_RESOURCE(1, 0xe800d000, gic_iid(273));
RSPI_RESOURCE(2, 0xe800d800, gic_iid(276));
RSPI_RESOURCE(3, 0xe800e000, gic_iid(279));
RSPI_RESOURCE(4, 0xe800e800, gic_iid(282));

static const struct rspi_plat_data rspi_pdata __initconst = {
	.num_chipselect	= 1,
};

#define r7s72100_register_rspi(idx)					   \
	platform_device_register_resndata(&platform_bus, "rspi-rz", idx,   \
					rspi##idx##_resources,		   \
					ARRAY_SIZE(rspi##idx##_resources), \
					&rspi_pdata, sizeof(rspi_pdata))

/* ==========================================================
 *		MTD ROM Section
 *		(Memory mapped QSPI)
 *
 * This assume u-boot has already memory mapped the QSPI
 * Flash in XIP mode. This a generic kernel driver to map
 * that linear physical memory space to a MTD device so
 * you can mount a root file system.
 *
 * Memory space is read only.
 * ==========================================================*/

/* BOARD: Since the QSPI memory mapping starts at 0x18000000,
          this mapping for a rootfs image at 0x18800000 (as in
	  a 0x00800000 offset). */
static struct mtd_partition qspi_flash_partitions[] __initdata = {
	{
		.name		= "qspi_rootfs",
		.offset		= 0x00800000,
		.size		= 64 * SZ_1M - 0x00800000,
	},
};

static const struct physmap_flash_data qspi_flash_data __initconst = {
	.width		= 4,
	.probe_type	= "map_rom",
	.parts		= qspi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(qspi_flash_partitions),
};

static const struct resource qspi_flash_resources[] __initconst = {
	DEFINE_RES_MEM(0x18000000, SZ_64M),
};

static const struct platform_device_info qspi_flash_info __initconst = {
	.parent		= &platform_bus,
	.name		= "physmap-flash",
	.id		= 0,
	.res		= qspi_flash_resources,
	.num_res	= ARRAY_SIZE(qspi_flash_resources),
	.data		= &qspi_flash_data,
	.size_data	= sizeof(qspi_flash_data),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* ==========================================================
 *		SCIF Section
 * UART
 * ==========================================================*/
#define R7S72100_SCIF(index, baseaddr, irq)				\
static const struct plat_sci_port scif##index##_platform_data = {	\
	.type		= PORT_SCIF,					\
	.regtype	= SCIx_SH2_SCIF_FIFODATA_REGTYPE,		\
	.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,		\
	.capabilities	= SCIx_HAVE_RTSCTS,				\
	.scscr		= SCSCR_RIE | SCSCR_TIE | SCSCR_RE | SCSCR_TE |	\
			  SCSCR_REIE,					\
};									\
									\
static struct resource scif##index##_resources[] = {			\
	DEFINE_RES_MEM(baseaddr, 0x100),				\
	DEFINE_RES_IRQ(irq + 1),					\
	DEFINE_RES_IRQ(irq + 2),					\
	DEFINE_RES_IRQ(irq + 3),					\
	DEFINE_RES_IRQ(irq),						\
}									\

/* BOARD: To save on RAM usage, we are only declaring the SCIF
	  channels we plan on using. Modify if you plan on
	  using other channels */
//R7S72100_SCIF(0, 0xe8007000, gic_iid(221));	/* Not used */
//R7S72100_SCIF(1, 0xe8007800, gic_iid(225));	/* Not used */
R7S72100_SCIF(2, 0xe8008000, gic_iid(229));
//R7S72100_SCIF(3, 0xe8008800, gic_iid(233));	/* Not used */
//R7S72100_SCIF(4, 0xe8009000, gic_iid(237));	/* Not used */
//R7S72100_SCIF(5, 0xe8009800, gic_iid(241));	/* Not used */
//R7S72100_SCIF(6, 0xe800a000, gic_iid(245));	/* Not used */
//R7S72100_SCIF(7, 0xe800a800, gic_iid(249));	/* Not used */

#define r7s72100_register_scif(index)					       \
	platform_device_register_resndata(&platform_bus, "sh-sci", index,      \
					  scif##index##_resources,	       \
					  ARRAY_SIZE(scif##index##_resources), \
					  &scif##index##_platform_data,	       \
					  sizeof(scif##index##_platform_data))

/* ==========================================================
 *		ADC Section
 *
 * This is just a driver example for PWM Pin (Pin TIOC4A only)
 * GRPEACH does not have TIOC4A attached to anything.
 * This PWM driver was intended to be used as a variable LCD
 * backlight driver. It was devleoped on a different RZ/A1 board.
 * ==========================================================*/
static const struct resource adc0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8005800, 0x100),
	DEFINE_RES_MEM(0xfcff0280, 0x6),
	DEFINE_RES_MEM(0xfcff0380, 0x21),
	DEFINE_RES_IRQ(gic_iid(170)),
	DEFINE_RES_IRQ(gic_iid(171)),
	DEFINE_RES_IRQ(gic_iid(146)),
};

static const struct sh_adc_data adc0_pdata __initconst = {
	.num_channels = 8,
	.mtu2_ch = 1,
};

static const struct platform_device_info adc0_info __initconst = {
	.name	= "sh_adc",
	.id	= 0,
	.data		= &adc0_pdata,
	.size_data	= sizeof(adc0_pdata),
	.res		= adc0_resources,
	.num_res	= ARRAY_SIZE(adc0_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* ==========================================================
 *		SD Card Section
 *
 * SD Host Interface
 * ==========================================================*/
/* SDHI0 */
static struct sh_mobile_sdhi_info sdhi0_pdata = {
	.dma_slave_tx	= RZA1DMA_SLAVE_SDHI0_TX,
	.dma_slave_rx	= RZA1DMA_SLAVE_SDHI0_RX,
	.tmio_caps	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ,
	.tmio_ocr_mask	= MMC_VDD_32_33,
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT,
};

static const struct resource sdhi0_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe804e000, 0x100, "SDHI0"),
	DEFINE_RES_IRQ_NAMED(gic_iid(302), SH_MOBILE_SDHI_IRQ_CARD_DETECT),
	DEFINE_RES_IRQ_NAMED(gic_iid(303), SH_MOBILE_SDHI_IRQ_SDCARD),
	DEFINE_RES_IRQ_NAMED(gic_iid(304), SH_MOBILE_SDHI_IRQ_SDIO),
};

static const struct platform_device_info sdhi0_info __initconst = {
	.name		= "sh_mobile_sdhi",
	.id		= 0,
	.res		= sdhi0_resources,
	.num_res	= ARRAY_SIZE(sdhi0_resources),
	.data		= &sdhi0_pdata,
	.size_data	= sizeof(sdhi0_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* SDHI1 */
static struct sh_mobile_sdhi_info sdhi1_pdata = {
	.dma_slave_tx	= RZA1DMA_SLAVE_SDHI1_TX,
	.dma_slave_rx	= RZA1DMA_SLAVE_SDHI1_RX,
	.tmio_caps	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ,
	.tmio_ocr_mask	= MMC_VDD_32_33,
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT,
};

static const struct resource sdhi1_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe804e800, 0x100, "SDHI1"),
	DEFINE_RES_IRQ_NAMED(gic_iid(305), SH_MOBILE_SDHI_IRQ_CARD_DETECT),
	DEFINE_RES_IRQ_NAMED(gic_iid(306), SH_MOBILE_SDHI_IRQ_SDCARD),
	DEFINE_RES_IRQ_NAMED(gic_iid(307), SH_MOBILE_SDHI_IRQ_SDIO),
};

static const struct platform_device_info sdhi1_info __initconst = {
	.name		= "sh_mobile_sdhi",
	.id		= 1,
	.res		= sdhi1_resources,
	.num_res	= ARRAY_SIZE(sdhi1_resources),
	.data		= &sdhi1_pdata,
	.size_data	= sizeof(sdhi1_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* ==========================================================
 *		USB Host Section
 * ==========================================================*/
static const struct r8a66597_platdata r8a66597_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_host0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8010000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(73)),
};

static const struct platform_device_info r8a66597_usb_host0_info __initconst= {
	.name		= "r8a66597_hcd",
	.id		= 0,
	.data		= &r8a66597_pdata,
	.size_data	= sizeof(r8a66597_pdata),
	.res		= r8a66597_usb_host0_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_host0_resources),
};

static const struct resource r8a66597_usb_host1_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8207000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(74)),
};

static const struct platform_device_info r8a66597_usb_host1_info __initconst = {
	.name		= "r8a66597_hcd",
	.id		= 1,
	.data		= &r8a66597_pdata,
	.size_data	= sizeof(r8a66597_pdata),
	.res		= r8a66597_usb_host1_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_host1_resources),
};

/* ==========================================================
 *		USB Device (Gadget) Section
 * ==========================================================*/
static const struct r8a66597_platdata r8a66597_usb_gadget0_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_gadget0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8010000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(73)),
};

static const struct platform_device_info r8a66597_usb_gadget0_info __initconst = {
	.name		= "r8a66597_udc",
	.id		= 0,
	.data		= &r8a66597_usb_gadget0_pdata,
	.size_data	= sizeof(r8a66597_usb_gadget0_pdata),
	.res		= r8a66597_usb_gadget0_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_gadget0_resources),
};

static const struct r8a66597_platdata r8a66597_usb_gadget1_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_gadget1_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8207000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(74)),
};

static const struct platform_device_info r8a66597_usb_gadget1_info __initconst = {
	.name		= "r8a66597_udc",
	.id		= 1,
	.data		= &r8a66597_usb_gadget1_pdata,
	.size_data	= sizeof(r8a66597_usb_gadget1_pdata),
	.res		= r8a66597_usb_gadget1_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_gadget1_resources),
};

/* ==========================================================
 *		CAN Bus Section
 * ==========================================================*/
#ifdef CONFIG_CAN_RZA1
static struct resource rz_can_resources[] = {
	[0] = {
		.start	= 0xe803a000,
		.end	= 0xe803b813,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 258,
		.end	= 258,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= 260,
		.end	= 260,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		.start	= 259,
		.end	= 259,
		.flags	= IORESOURCE_IRQ,
	},
	[4] = {
		.start	= 253,
		.end	= 253,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct rz_can_platform_data rz_can_data = {
	.channel	= 1,
	.clock_select	= CLKR_CLKC,
};

static struct platform_device_info rz_can_device = {
	.name		= "rz_can",
	.id		= 1,
	.num_res	= ARRAY_SIZE(rz_can_resources),
	.res		= rz_can_resources,
	.data		= &rz_can_data,
	.size_data	= sizeof(rz_can_data),
};
#endif /* CONFIG_CAN_RZA1 */

/* ==========================================================
 *		Audio Section
 * ==========================================================*/
static const struct platform_device_info alsa_soc_info = {
	.name		= "grpeach_alsa_soc_platform",
	.id		= 0,
};

static const struct resource scux_resources[] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xe8208000, 0x00001778, "scux"),
	[1] = DEFINE_RES_MEM_NAMED(0xe820b000, 0x00002830, "ssif0"),
};

static struct scu_config ssi_ch_value[] = {
	{RP_MEM_SSI0,		SSI0},
	{RP_MEM_SRC1_SSI0,	SSI0},
	{RP_MEM_SRC1_DVC1_SSI0,	SSI0},
	{RC_SSI0_MEM,		SSI0},
	{RC_SSI0_SRC0_MEM,	SSI0},
};

static struct scu_config src_ch_value[] = {
	{RP_MEM_SSI0,		-1},
	{RP_MEM_SRC1_SSI0,	SRC1},
	{RP_MEM_SRC1_DVC1_SSI0,	SRC1},
	{RC_SSI0_MEM,		-1},
	{RC_SSI0_SRC0_MEM,	SRC0},
};

static struct scu_config dvc_ch_value[] = {
	{RP_MEM_SSI0,		-1},
	{RP_MEM_SRC1_SSI0,	-1},
	{RP_MEM_SRC1_DVC1_SSI0,	DVC1},
	{RC_SSI0_MEM,		-1},
	{RC_SSI0_SRC0_MEM,	-1},
};

static struct scu_config audma_slave_value[] = {
	{RP_MEM_SSI0,		RZA1DMA_SLAVE_PCM_MEM_SSI0},
	{RP_MEM_SRC1_SSI0,	RZA1DMA_SLAVE_PCM_MEM_SRC1},
	{RP_MEM_SRC1_DVC1_SSI0,	RZA1DMA_SLAVE_PCM_MEM_SRC1},
	{RC_SSI0_MEM,		RZA1DMA_SLAVE_PCM_SSI0_MEM},
	{RC_SSI0_SRC0_MEM,	RZA1DMA_SLAVE_PCM_SRC0_MEM},
};

static struct scu_config ssi_depend_value[] = {
	{RP_MEM_SSI0,		SSI_INDEPENDANT},
	{RP_MEM_SRC1_SSI0,	SSI_DEPENDANT},
	{RP_MEM_SRC1_DVC1_SSI0,	SSI_DEPENDANT},
	{RC_SSI0_MEM,		SSI_INDEPENDANT},
	{RC_SSI0_SRC0_MEM,	SSI_DEPENDANT},
};

static struct scu_config ssi_mode_value[] = {
	{RP_MEM_SSI0,		SSI_MASTER},
	{RP_MEM_SRC1_SSI0,	SSI_MASTER},
	{RP_MEM_SRC1_DVC1_SSI0,	SSI_MASTER},
	{RC_SSI0_MEM,		SSI_SLAVE},
	{RC_SSI0_SRC0_MEM,	SSI_SLAVE},
};

static struct scu_config src_mode_value[] = {
	{RP_MEM_SSI0,		SRC_CR_ASYNC},
	{RP_MEM_SRC1_SSI0,	SRC_CR_ASYNC},
	{RP_MEM_SRC1_DVC1_SSI0,	SRC_CR_ASYNC},
	{RC_SSI0_MEM,		SRC_CR_ASYNC},
	{RC_SSI0_SRC0_MEM,	SRC_CR_ASYNC},
};

static const struct scu_platform_data scu_pdata __initconst = {
	.ssi_master		= SSI0,
	.ssi_slave		= SSI0,
	.ssi_ch			= ssi_ch_value,
	.ssi_ch_num		= ARRAY_SIZE(ssi_ch_value),
	.src_ch			= src_ch_value,
	.src_ch_num		= ARRAY_SIZE(src_ch_value),
	.dvc_ch			= dvc_ch_value,
	.dvc_ch_num		= ARRAY_SIZE(dvc_ch_value),
	.dma_slave_maxnum	= RZA1DMA_SLAVE_PCM_MAX,
	.audma_slave		= audma_slave_value,
	.audma_slave_num	= ARRAY_SIZE(audma_slave_value),
	.ssi_depend		= ssi_depend_value,
	.ssi_depend_num		= ARRAY_SIZE(ssi_depend_value),
	.ssi_mode		= ssi_mode_value,
	.ssi_mode_num		= ARRAY_SIZE(ssi_mode_value),
	.src_mode		= src_mode_value,
	.src_mode_num		= ARRAY_SIZE(src_mode_value),
};

static const struct platform_device_info scux_info __initconst = {
	.name		= "scux-pcm-audio",
	.id		= 0,
	.data		= &scu_pdata,
	.size_data	= sizeof(scu_pdata),
	.num_res	= ARRAY_SIZE(scux_resources),
	.res		= scux_resources,
};

/* ==========================================================
 *		OV7670 Section
 *
 * OV7670 Camera Example
 * ==========================================================*/
/* BOARD: This was only for example. If you are not using a
	  OV7670, you can remove this seciton */
static struct ov7670_config ov7670_config = {
	.min_width =		320,	/* Filter out smaller sizes */
	.min_height =		160,	/* Filter out smaller sizes */
	.clock_speed =		0,	/* External clock speed (MHz) */
	.use_smbus =		0,	/* Use smbus I/O instead of I2C */
	.pll_bypass =		0,	/* Choose whether to bypass the PLL */
	.pclk_hb_disable =	0,	/* Disable toggling pixclk during horizontal blanking */
};

static struct i2c_board_info ceu_camera = {
	I2C_BOARD_INFO("ov7670", 0x21),
};

static struct soc_camera_link ceu_iclink = {
	.bus_id = 0,
	.board_info = &ceu_camera,
	.i2c_adapter_id = 0,
	.priv = &ov7670_config,
};

static const struct platform_device_info ceu_camera_info __initconst = {
	.name = "soc-camera-pdrv",
	.id = 0,
	.data = &ceu_iclink,
	.size_data = sizeof(ceu_iclink),
};

/* ==========================================================
 *		CEU Section
 *
 * Camera Engine Unit
 * ==========================================================*/
static struct sh_mobile_ceu_info sh_mobile_ceu_info = {
	.flags = SH_CEU_FLAG_USE_8BIT_BUS,
	.max_width = 1280,
	.max_height = 768,
};

static struct resource ceu_resources[] = {
	[0] = {
		.name	= "CEU",
		.start	= 0xE8210000,
		.end	= 0xE821000f,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start  = gic_iid(364),
		.flags  = IORESOURCE_IRQ,
	},
#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
	/* CEU Requires dedicated memory when in XIP mode, as there's not enough contiguous
	 * memory for the buffers. You must also specify mem=8M on the kernel command line */
	[2] = {
		.name	= "CEU Buffer",
		.start	= 0x20800000,
		.end	= 0x20A00000,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

static const struct platform_device_info ceu_info __initconst = {
	.name		= "sh_mobile_ceu",
	.id		= 0,
	.res		= ceu_resources,
	.num_res	= ARRAY_SIZE(ceu_resources),
	.data		= &sh_mobile_ceu_info,
	.size_data	= sizeof(sh_mobile_ceu_info),
	.dma_mask	= DMA_BIT_MASK(32),
};

#ifdef CONFIG_VIDEO_SH_MOBILE_CEU
static struct pfc_pinmux_assign ceu_common[] = {
	{ P11_11, ALT1, },	/* VIO_D23 */
	{ P11_10, ALT1, },	/* VIO_D22 */
	{ P11_9,  ALT1, },	/* VIO_D21 */
	{ P11_8,  ALT1, },	/* VIO_D20 */
	{ P11_7,  ALT1, },	/* VIO_D19 */
	{ P11_6,  ALT1, },	/* VIO_D18 */
	{ P11_5,  ALT1, },	/* VIO_D17 */
	{ P11_4,  ALT1, },	/* VIO_D16 */
	{ P11_3,  ALT6, },	/* VIO_D15 */
	{ P11_2,  ALT6, },	/* VIO_D14 */
	{ P11_1,  ALT6, },	/* VIO_D13 */
	{ P11_0,  ALT6, },	/* VIO_D12 */
	{ P10_15, ALT6, },	/* VIO_D11 */
	{ P10_14, ALT6, },	/* VIO_D10 */
	{ P10_13, ALT6, },	/* VIO_D9 */
	{ P10_12, ALT6, },	/* VIO_D8 */
	{ P10_11, ALT6, },	/* VIO_D7 */
	{ P10_10, ALT6, },	/* VIO_D6 */
	{ P10_9,  ALT6, },	/* VIO_D5 */
	{ P10_8,  ALT6, },	/* VIO_D4 */
	{ P10_7,  ALT6, },	/* VIO_D3 */
	{ P10_6,  ALT6, },	/* VIO_D2 */
	{ P10_5,  ALT6, },	/* VIO_D1 */
	{ P10_4,  ALT6, },	/* VIO_D0 */
	{ P10_3,  ALT6, },	/* VIO_DFLD */
	{ P10_2,  ALT6, },	/* VIO_DHD */
	{ P10_1,  ALT6, },	/* VIO_DVD */
	{ P10_0,  ALT6, },	/* VIO_CLK */
};

static void ceu_pinmux(void)
{
	size_t n;

	for (n = 0; n < ARRAY_SIZE(ceu_common); n++)
		r7s72100_pfc_pin_assign(ceu_common[n].port, ceu_common[n].mode, DIIO_PBDC_EN);
}
#endif

#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
/* By default, the Linux ARM code will pre-allocated IRQ descriptors
   based on the size (HW) of the GIC. For this device, that means
   576 possible interrutps sources. This eats up a lot of RAM at
   run-time that is pretty much wasted.
   Therefore, use this 'whitelist' below to delete any pre-allocated
   irq descriptors except those that is in this list to achive up to
   400KB of RAM savings.
*/
struct irq_res {
	int irq;	/* Starting IRQ number */
	int count;	/* The number of consecutive IRQs */
};
struct irq_res const irq_keep_list[] __initconst = {
//	{32, 1},	/* IRQ0 */
	{33, 1},	/* IRQ1 (for ft5x06-ts Touchsreen) */
//	{34, 1},	/* IRQ2 */
//	{35, 1},	/* IRQ3 */
//	{36, 1},	/* IRQ4 */
//	{37, 1},	/* IRQ5 */
//	{38, 1},	/* IRQ6 */
//	{39, 1},	/* IRQ7 */
//	{40, 1},	/* PL310ERR (L2 Cache error - not used) */
	{41, 17},	/* RZA1_DMA */
	{73, 1},	/* USB0 (host/device) */
	{74, 1},	/* USB1 (host/device) */
	{75, 23},	/* VDC0 */
	{99, 23},	/* VDC1 */
//	{126, 1},	/* JCU */
	{134, 2},	/* OSTM */
	{139, 1},	/* MTU2-TGI0A (Kernel jiffies) */
//	{170, 2}, {146, 2},	/* ADC and MTU2-TGI1A */
//	{189, 8},	/* RIIC0 */
	{197, 8},	/* RIIC1 (CN9) */
//	{205, 8},	/* RIIC2 */
	{213, 8},	/* RIIC3 (CN16) */
//	{221, 4},	/* SCIF0 */
//	{225, 4},	/* SCIF1 */
	{229, 4},	/* SCIF2 (Console) */
//	{233, 4},	/* SCIF3 */
//	{237, 4},	/* SCIF4 */
//	{241, 4},	/* SCIF5 */
//	{245, 4},	/* SCIF6 */
//	{249, 4},	/* SCIF7 */
//	{253, 2},	/* CAN GERR/GRECC */
//	{255, 3},	/* CAN0 */
//	{258, 3},	/* CAN1 */
//	{261, 3},	/* CAN2 */
//	{264, 3},	/* CAN3 */
//	{267, 3},	/* CAN4 */
//	{270, 3},	/* RSPI0 */
//	{273, 3},	/* RSPI1 */
//	{276, 3},	/* RSPI2 */
//	{279, 3},	/* RSPI3 */
//	{282, 3},	/* RSPI4 */
//	{299, 3},	/* MMC */
//	{302, 3},	/* SDHI0 */
	{305, 3},	/* SDHI1 */
	{308, 3},	/* RTC */
	{359, 1},	/* ETH */
};

static int rz_irq_trim = 0;
static int __init early_rz_irq_trim(char *str)
{
	rz_irq_trim = 1;
	return 0;
}
early_param("rz_irq_trim", early_rz_irq_trim);

/* Removes unused pre-allocated IRQ. */
/* This operation only occurs when 'rz_irq_trim' is on the boot command line */
static void remove_irqs(void)
{
	int i,j;
	int max = nr_irqs;
	int keep;

	if( !rz_irq_trim )
		return;		/* Feature disabled if 'rz_irq_trim' not set */

	/* Run through all allocated irq descriptors and delete
	   the ones we are not using */
	/* Skip 0 - 16 because they are the soft irqs */
	for (i=17; i < max; i++) {
		/* Is it one we want to keep? */
		for ( j=0, keep=0 ; j < sizeof(irq_keep_list)/sizeof(struct irq_res); j++) {
			if( i == irq_keep_list[j].irq ) {
				keep = 1;
				break;
			}
		}

		if( keep )
			i += irq_keep_list[j].count - 1; /* skip if multiple */
		else
			irq_free_descs(i, 1);		/* un-used irq */
	}
}
#endif /* XIP_KERNEL_WITHOUT_EXTERNAL_RAM */

/* Write to I2C device */
/* Based off of board-sx1.c */
int rza1_i2c_write_byte(u8 ch, u8 devaddr, u8 regoffset, u8 value)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	adap = i2c_get_adapter(ch);
	if (!adap)
		return -ENODEV;
	msg->addr = devaddr;	/* I2C address of chip */
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = regoffset;	/* register num */
	data[1] = value;		/* register data */
	err = i2c_transfer(adap, msg, 1);
	i2c_put_adapter(adap);
	if (err >= 0)
		return 0;
	return err;
}

/* Read from I2C device */
/* Based off of board-sx1.c */
int rza1_i2c_read_byte(u8 ch, u8 devaddr, u8 regoffset, u8 *value)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	adap = i2c_get_adapter(ch);
	if (!adap)
		return -ENODEV;

	msg->addr = devaddr;	/* I2C address of chip */
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	data[0] = regoffset;	/* register num */
	err = i2c_transfer(adap, msg, 1);
	if (err < 0)
		return err;

	msg->addr = devaddr;	/* I2C address */
	msg->flags = I2C_M_RD;
	msg->len = 1;
	msg->buf = data;
	err = i2c_transfer(adap, msg, 1);
	*value = data[0];
	i2c_put_adapter(adap);

	if (err >= 0)
		return 0;
	return err;
}

static void __init grpeach_add_standard_devices(void)
{
#ifdef CONFIG_CACHE_L2X0
	/* Early BRESP enable, 16K*8way(defualt) */
	/* NOTES: BRESP can be set for IP version after r2p0 */
	/*        As of linux-3.16, cache-l2x0.c handles this automatically */
	l2x0_init(IOMEM(0xfffee000), 0x00000000, 0xffffffff);	/* Leave as defaults */
#endif
#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
	remove_irqs();
#endif

	r7s72100_clock_init();
	r7s72100_pinmux_setup();
	r7s72100_add_dt_devices();

	/* ------------ Pin setup section ---------------*/

	r7s72100_pfc_pin_assign(P1_15, ALT1, DIIO_PBDC_EN);	/* AD7 */
	r7s72100_pfc_pin_assign(P1_0, ALT1, DIIO_PBDC_EN);	/* I2C SCL0 */
	r7s72100_pfc_pin_assign(P1_1, ALT1, DIIO_PBDC_EN);	/* I2C SDA0 */

	r7s72100_pfc_pin_assign(P4_4, ALT5, DIIO_PBDC_EN);	/* SSISCK0 */
	r7s72100_pfc_pin_assign(P4_5, ALT5, DIIO_PBDC_EN);	/* SSIWS0 */
	r7s72100_pfc_pin_assign(P4_6, ALT5, DIIO_PBDC_EN);	/* SSIRxD0 */
	r7s72100_pfc_pin_assign(P4_7, ALT5, SWIO_OUT_PBDCEN);	/* SSITxD0 */

	r7s72100_pfc_pin_assign(P3_8, ALT7, DIIO_PBDC_DIS);	/* SDHI1 CD */
	r7s72100_pfc_pin_assign(P3_9, ALT7, DIIO_PBDC_DIS);	/* SDHI1 WP */
	r7s72100_pfc_pin_assign(P3_10, ALT7, DIIO_PBDC_EN);	/* SDHI1 DAT1 */
	r7s72100_pfc_pin_assign(P3_11, ALT7, DIIO_PBDC_EN);	/* SDHI1 DAT0 */
	r7s72100_pfc_pin_assign(P3_12, ALT7, DIIO_PBDC_DIS);	/* SDHI1 CLK */
	r7s72100_pfc_pin_assign(P3_13, ALT7, DIIO_PBDC_EN);	/* SDHI1 CMD */
	r7s72100_pfc_pin_assign(P3_14, ALT7, DIIO_PBDC_EN);	/* SDHI1 DAT3*/
	r7s72100_pfc_pin_assign(P3_15, ALT7, DIIO_PBDC_EN);	/* SDHI1 DAT2 */

#ifdef CONFIG_VIDEO_SH_MOBILE_CEU
	ceu_pinmux();
#endif

	/* Set up IRQ for touchscreen */
	{
		/* Set for low edge trigger */
		void __iomem *irc1 = IOMEM(0xfcfef802);
		__raw_writew((__raw_readw(irc1) & ~(0x3 << 2)), irc1);
		r7s72100_pfc_pin_assign(P4_9, ALT8, DIIO_PBDC_DIS);  /* IRQ1 */
	}

#ifdef CONFIG_CAN_RZA1
	/* Ch 1 (conflicts with Ethernet, requires resistor change) */
	r7s72100_pfc_pin_assign(P5_9, ALT5, DIIO_PBDC_DIS);	/* CAN CAN1RX */
	r7s72100_pfc_pin_assign(P5_10, ALT5, DIIO_PBDC_DIS);	/* CAN CAN1TX */

	platform_device_register_full(&rz_can_device);
#endif

	/* ------------ Register Device and Drivers ---------------*/

	i2c_register_board_info(0, i2c0_devices, ARRAY_SIZE(i2c0_devices));
	i2c_register_board_info(3, i2c3_devices, ARRAY_SIZE(i2c3_devices));

	platform_device_register_full(&ostm_info);	/* High precision OS Timer */
	platform_device_register_full(&dma_info);	/* DMA */
	platform_device_register_full(&alsa_soc_info);	/* Sound */
	platform_device_register_full(&scux_info);	/* Sound */
	platform_device_register_full(&ether_info);	/* Ethernet */

//	platform_device_register_full(&riic0_info);	/* I2C0: Not used */
	platform_device_register_full(&riic1_info);	/* I2C1 */
//	platform_device_register_full(&riic2_info);	/* I2C2: Not used */
	platform_device_register_full(&riic3_info);	/* I2C3 */
	platform_device_register_full(&rtc_info);	/* RTC */

	platform_device_register_full(&vdc5fb1_info);	/* VDC5 ch1 */
	//platform_device_register_full(&simplefb_info);	/* Simplefb (FLOATING LAYER) */

	platform_device_register_full(&ceu_info);		/* CEU */
	platform_device_register_full(&ceu_camera_info);	/* OV7670 */


	platform_device_register_full(&qspi_flash_info);	/* Memory Mapped XIP QSPI */

	platform_device_register_full(&adc0_info);	/* ADC */

//	platform_device_register_full(&sdhi0_info);	/* SDHI ch0 */
	platform_device_register_full(&sdhi1_info);	/* SDHI ch1 */ 

	r7s72100_pfc_pin_assign(P3_8, PMODE, PORT_OUT_LOW);	/*VBUS*/
	r7s72100_pfc_pin_assign(P8_15, PMODE, PORT_OUT_HIGH);	/*contrast*/
	r7s72100_pfc_pin_assign(P8_1, PMODE, PORT_OUT_HIGH);	/*lcd_blon*/
	r7s72100_pfc_pin_assign(P7_15, PMODE, PORT_OUT_HIGH);	/*lcd_pwon*/

	if (usbgs == 0) {
		platform_device_register_full(&r8a66597_usb_gadget0_info);	/* USB ch0 as Device */
		platform_device_register_full(&r8a66597_usb_host1_info);	/* USB ch1 as Host */
	} else if (usbgs == 1) {
		platform_device_register_full(&r8a66597_usb_host0_info);	/* USB ch0 as Host */
		platform_device_register_full(&r8a66597_usb_gadget1_info);	/* USB ch1 as Device */
	} else {
		platform_device_register_full(&r8a66597_usb_host0_info);	/* USB ch0 as Host */
		platform_device_register_full(&r8a66597_usb_host1_info);	/* USB ch1 as Host */
	}

//	r7s72100_register_rspi(0);	/* RSPI ch0 */ /* Not used */
//	r7s72100_register_rspi(1);	/* RSPI ch1 */ /* Not used */
//	r7s72100_register_rspi(2);	/* RSPI ch2 */ /* Not used */
//	r7s72100_register_rspi(3);	/* RSPI ch3 */ /* Not used */
//	r7s72100_register_rspi(4);	/* RSPI ch4 */ /* Not used */

//	r7s72100_register_scif(0);	/* SCIF ch0 */ /* Not used */
//	r7s72100_register_scif(1);	/* SCIF ch1 */ /* Not used */
	r7s72100_register_scif(2);	/* SCIF ch2 */
//	r7s72100_register_scif(3);	/* SCIF ch3 */ /* Not used */
//	r7s72100_register_scif(4);	/* SCIF ch4 */ /* Not used */
//	r7s72100_register_scif(5);	/* SCIF ch5 */ /* Not used */
//	r7s72100_register_scif(6);	/* SCIF ch6 */ /* Not used */
//	r7s72100_register_scif(7);	/* SCIF ch7 */ /* Not used */

}

//#define STATS_OUT
#ifdef STATS_OUT
/* This function is intended to be called periodically in order to
print out the current CPU Idle time and RAM usage with minimal
overhead.

On the RZ/A1 GRPEACH, SCIF 1 comes out on header XXX
	TXD1 = XXX, pin 9
	RXD1 = XXX, pin 12
*/

/* You need to fill in how much RAM your system has so we can
   subtract TOTAL - FREE - USED. */
#define TOTAL_SYSTEM_RAM 0*1024 /* Total System RAM in KB */
#if TOTAL_SYSTEM_RAM == 0
  #error "Please define TOTAL_SYSTEM_RAM first "
#endif

/* NOTE: You need to remove 'static' from the function
  get_idle_time() in file fs/proc/stat.c */
u64 get_idle_time(int cpu);

void print_stats(void)
{
	static char text[17] = " idle%,memKB\r\n";
	static cputime64_t idle, last_idle = -1;
	static void *scif1_base;

	int ram_used = 3000, cpu_used = 10;
	int i;
	struct sysinfo info;
	struct clk *scif1_clk;

	/* Setup, only once */
	if( last_idle == -1 ) {
		/* SCIF 1 pinmux */
		r7s72100_pfc_pin_assign(P4_12, ALT7, DIIO_PBDC_DIS);	/* SCIF1 TX */
		r7s72100_pfc_pin_assign(P4_13, ALT7, DIIO_PBDC_DIS);	/* SCIF1 RX */

		/* SCIF 1 clock */
		scif1_clk = clk_get_sys("sh-sci.1", "sci_fck");
		clk_enable(scif1_clk);

		/* SCIF1 Registers */
		/* Map registers so we can get at them */
		scif1_base = ioremap_nocache(0xE8007800, 0x30);
		#define SCFTDR_1 *(volatile uint8_t *)(scif1_base + 0x0C)
		#define SCSCR_1 *(volatile uint16_t *)(scif1_base + 0x08)
		#define SCBRR_1 *(volatile uint8_t *)(scif1_base + 0x04)

		/* Minimum SCIF setup */
		SCBRR_1 = 0x11;		// Baud 115200
		SCSCR_1 = 0x0030;	// Enable TX an RX, but no interrupts

		/* Print out header */
		i = 0;
		while(text[i++])
		{
			SCFTDR_1 = text[i];
		}

		//idle = kcpustat_cpu(0).cpustat[CPUTIME_IDLE];
		idle = get_idle_time(0);
		last_idle = get_idle_time(0);
	}

	/* Print out current CPU idle time and RAM usage */
	si_meminfo(&info);

	/* Idle time */
	idle = get_idle_time(0);
	cpu_used = (last_idle - idle)*100;
	cpu_used = cpu_used / 132;
	if( cpu_used >= 99)
		cpu_used = 99;
	last_idle = idle;

	/* USED_RAM = TOTAL_RAM - FREE_RAM */
	ram_used = TOTAL_SYSTEM_RAM - (info.freeram << (PAGE_SHIFT - 10));

	sprintf(text,"%03d%%,%04dKB\r\n",cpu_used,ram_used);

	/* We want the string to be less than 16 characters so all fits
	   in the 16byte SCIF FIFO */
	i=0;
	while(text[i++])
	{
		SCFTDR_1 = text[i];
	}
}
#endif

static int heartbeat(void * data)
{
	u8 index = 0;

	while(1) {

		switch (index)
		{
			case 0:
			case 2:
				r7s72100_pfc_pin_assign(P6_12, PMODE, PORT_OUT_HIGH);
				break;
			default:
				r7s72100_pfc_pin_assign(P6_12, PMODE, PORT_OUT_LOW);
				break;
		}

		index = (index + 1) & 7;

		msleep_interruptible(125);

#ifdef STATS_OUT
		/* Only print stats once per second */
		if( (index & 0x03) == 0 )
			print_stats();
#endif
	}

	return 0;
}

static void __init grpeach_init_late(void)
{
	/* Start heartbeat kernel thread */
	kthread_run(heartbeat, NULL,"heartbeat");
}

#define WTCSR 0
#define WTCNT 2
#define WRCSR 4
static void r7s72100_restart(enum reboot_mode mode, const char *cmd)
{
	void *base = ioremap_nocache(0xFCFE0000, 0x10);

	/* If you have board specific stuff to do, you can do it
	   here before you reboot */

	/* NOTE: A reboot command doesn't 'sync' before this function
	   is called. See funciotn reboot() in kernel/reboot.c */

	/* Dummy read (must read WRCSR:WOVF at least once before clearing) */
	*(volatile uint8_t *)(base + WRCSR) = *(uint8_t *)(base + WRCSR);

	*(volatile uint16_t *)(base + WRCSR) = 0xA500;	/* Clear WOVF */
	*(volatile uint16_t *)(base + WRCSR) = 0x5A5F;	/* Reset Enable */
	*(volatile uint16_t *)(base + WTCNT) = 0x5A00;	/* Counter to 00 */
	*(volatile uint16_t *)(base + WTCSR) = 0xA578;	/* Start timer */

	while(1); /* Wait for WDT overflow */
}

void __init r7s72100_init_early(void)
{
	shmobile_init_delay();

#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
	/* Set the size of our pre-allocated DMA buffer pool because the
	   default is 256KB */
	init_dma_coherent_pool_size(16 * SZ_1K);
#endif
}

static const char * const grpeach_boards_compat_dt[] __initconst = {
	"renesas,grpeach",
	NULL,
};

DT_MACHINE_START(GRPEACH_DT, "grpeach")
	.init_early	= r7s72100_init_early,
	.init_machine	= grpeach_add_standard_devices,
	.init_late	= grpeach_init_late,
	.dt_compat	= grpeach_boards_compat_dt,
	.map_io		= r7s72100_map_io,
	.restart	= r7s72100_restart,
MACHINE_END
