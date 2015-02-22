/*
 * RSKRZA1 board support
 *
 * Copyright (C) 2013  Renesas Solutions Corp.
 * Copyright (C) 2013  Magnus Damm
 * Copyright (C) 2014  Chris Brandt
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
#include <mach/common.h>
#include <mach/irqs.h>
#include <mach/r7s72100.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/spi/rspi.h>
#include <linux/spi/sh_spibsc.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
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
#include <linux/irq.h>

/* Board Options */
//#define RSPI_TESTING	/* Uncomment for RSPI4 enabled on CN15 */


static int usbgs = -1;
static int __init early_usbgs(char *str)
{
	usbgs = 0;
	get_option(&str, &usbgs);
	return 0;
}
early_param("usbgs", early_usbgs);

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

void __init rza1_map_io(void)
{
	iotable_init(rza1_io_desc, ARRAY_SIZE(rza1_io_desc));
}


/* DMA */
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
#ifdef CONFIG_XIP_KERNEL
	.channel_num	= 2,	/* Less channels means less RAM */
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

/* Video */
#define	P1CLK			((13330000 * 30) / 6)
#define	PIXCLOCK(hz, div)	\
	(u32)(1000000000000 / ((double)(hz) / (double)(div)))

struct pfc_pinmux_assign {
	int port;	/* enum */
	int mode;	/* enum */
	int opts;
};

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

static struct pfc_pinmux_assign lcd0_tcon[] = {
	{ P11_14, ALT5, },	/* LCD0_TCON0 */
	{ P11_13, ALT5, },	/* LCD0_TCON1 */
	{ P11_12, ALT5, },	/* LCD0_TCON2 */
	{ P11_11, ALT5, },	/* LCD0_TCON3 */
	{ P11_10, ALT5, },	/* LCD0_TCON4 */
	{ P11_9,  ALT5, },	/* LCD0_TCON5 */
	{ P11_8,  ALT5, },	/* LCD0_TCON6 */
};

static void vdc5fb_pinmux(struct pfc_pinmux_assign *pf, size_t num)
{
	size_t n;

	for (n = 0; n < num; pf++, n++)
		r7s72100_pfc_pin_assign(pf->port, pf->mode, DIIO_PBDC_DIS);
}

static void vdc5fb_pinmux_tcon(struct pfc_pinmux_assign *pf, size_t num,
	struct vdc5fb_pdata *pdata)
{
	size_t n;

	for (n = 0; n < num; pf++, n++)
		if (pdata->tcon_sel[n] != TCON_SEL_UNUSED)
			r7s72100_pfc_pin_assign(pf->port, pf->mode, DIIO_PBDC_DIS);
}

#define VDC5_BPP 32 /* 16bpp or 32bpp */
#define VDC5_FBSIZE (800*480*VDC5_BPP/8)
/* Place at end of internal RAM, but on a PAGE boundry */
#define VDC5_FB_ADDR ((0x20A00000 - VDC5_FBSIZE) & PAGE_MASK)

static const struct resource vdc5fb_resources[VDC5FB_NUM_RES] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcff6000, 0x00002000, "vdc5fb.0: reg"),
	[1] = DEFINE_RES_MEM_NAMED(VDC5_FB_ADDR, VDC5_FBSIZE, "vdc5fb.0: fb"),
	[2] = DEFINE_RES_NAMED(75, 23, "vdc5fb.0: irq", IORESOURCE_IRQ),
};

static int vdc5fb_pinmux_gwp0700cnwv04(struct platform_device *pdev)
{
	struct vdc5fb_pdata *pdata
	    = (struct vdc5fb_pdata *)pdev->dev.platform_data;

	vdc5fb_pinmux(lcd0_common, ARRAY_SIZE(lcd0_common));
	vdc5fb_pinmux_tcon(lcd0_tcon, ARRAY_SIZE(lcd0_tcon), pdata);

	return 0;
}

static struct fb_videomode videomode_gwp0700cnwv04 = {
	.name		= "gwp0700cnwv04",
	.refresh	= 60,
	.xres		= 800,
	.yres		= 480,
	.pixclock	= PIXCLOCK(P1CLK, 2),
	.left_margin	= 210,
	.right_margin	= 46,
	.upper_margin	= 22,
	.lower_margin	= 23,
	.hsync_len	= 40,
	.vsync_len	= 20,
	.sync		= 0,
	.vmode		= 0,
	.flag		= 0,
};

static const struct vdc5fb_pdata vdc5fb_gwp0700cnwv04_pdata = {
	.name			= "gwp0700cnwv04",
	.videomode		= &videomode_gwp0700cnwv04,
	.panel_icksel		= ICKSEL_P1CLK,
	.bpp			= VDC5_BPP,
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
	.pinmux			= vdc5fb_pinmux_gwp0700cnwv04,
	.layers			= {
		/* Graphics 2 - Image Synthesizer */
		/* Full LCD Panel - will be /dev/fb0 */
		[2].xres	= 800,
		[2].yres	= 480,
		[2].x_offset	= 0,
		[2].y_offset	= 0,
#if VDC5_BPP == 16
		[2].format	= GR_FORMAT(GR_FORMAT_RGB565) | GR_RDSWA(6),
#else
		[2].format	= GR_FORMAT(GR_FORMAT_ARGB8888) | GR_RDSWA(4),
#endif
		[2].bpp		= VDC5_BPP,
		[2].base	= VDC5_FB_ADDR,
		[2].blend	 = 0,

#if 0 /* SAMPLE */
		/* Graphics 3 - Image Synthesizer */
		/* Layer 3 - floating layer (Alpha blending) */
		[3].xres	= 200,
		[3].yres	= 200,
		[3].x_offset	= 100,
		[3].y_offset	= 50,
		[3].format	= GR_FORMAT(GR_FORMAT_ARGB8888) | GR_RDSWA(4),
		[3].bpp		= 32,
		[3].base	= 0x20000000 + (10 * SZ_1M) - (200 * 200 * 4), /* End of internal RAM */
		[3].blend	= 1,
#endif
	},
};

static const struct platform_device_info vdc5fb_info __initconst = {
	.name		= "vdc5fb",
	.id		= 0,
	.res		= vdc5fb_resources,
	.num_res	= ARRAY_SIZE(vdc5fb_resources),
	.data		= &vdc5fb_gwp0700cnwv04_pdata,
	.size_data	= sizeof(vdc5fb_gwp0700cnwv04_pdata),
	.dma_mask	= DMA_BIT_MASK(32),	/* only needed if not hardcoding fb */
};

/* JCU */
static const struct uio_info jcu_platform_pdata __initconst = {
	.name = "JCU",
	.version = "0",
	.irq = 126, /* Not used */
};

static const struct resource jcu_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe8017000, 0x1000, "jcu:reg"), /* for JCU of RZ */
	DEFINE_RES_MEM_NAMED(0xfcfe0000, 0x2000, "jcu:rstreg clkreg"), /* Use STBCR6 & SWRSTCR2 */
	DEFINE_RES_MEM_NAMED(0x20200000, 0x100000, "jcu:iram"), /* (Non cacheable 1MB) */
};

static const struct platform_device_info jcu_info __initconst = {
	.name		= "uio_pdrv_genirq",
	.id		= 0,
	.data		= &jcu_platform_pdata,
	.size_data	= sizeof(jcu_platform_pdata),
	.res		= jcu_resources,
	.num_res	= ARRAY_SIZE(jcu_resources),
};

/* Ether */
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

static struct edt_ft5x06_platform_data ft5216_pdata __initdata = {
	.irq_pin	= -1,
	.reset_pin	= -1,

	/* startup defaults for operational parameters */
//	bool use_parameters;
//	u8 gain;
//	u8 threshold;
//	u8 offset;
//	u8 report_rate;
};


static const struct i2c_board_info i2c0_devices[] __initconst = {
	{
		I2C_BOARD_INFO("ft5x06-ts", 0x38),
		.platform_data = &ft5216_pdata,
		.irq		= 33,
	},
};

/* Init TC_IRQ1 as input */
static void __init gpio_irq_init(void) {
	/* Set for low edge trigger */
	void __iomem *irc1 = IOMEM(0xfcfef802);
	__raw_writew((__raw_readw(irc1) & ~(0x3 << 2)), irc1);
	r7s72100_pfc_pin_assign(P4_9, ALT8, DIIO_PBDC_DIS);  /* IRQ1 */
}

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

/* OSTM */
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

/* RTC */
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

/* NOR Flash */
static struct mtd_partition nor_flash_partitions[] __initdata = {
	{
		.name		= "nor_u-boot",
		.offset		= 0x00000000,
		.size		= SZ_512K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "nor_u-boot_env",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_256K,
	},
	{
		.name		= "nor_dtb",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_256K,
	},
	{
		.name		= "nor_kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
	},
	{
		.name		= "nor_data",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static const struct physmap_flash_data nor_flash_data __initconst = {
	.width		= 2,
	.parts		= nor_flash_partitions,
	.nr_parts	= ARRAY_SIZE(nor_flash_partitions),
};

static const struct resource nor_flash_resources[] __initconst = {
	DEFINE_RES_MEM(0x00000000, SZ_64M),
};

static const struct platform_device_info nor_flash_info __initconst = {
	.parent		= &platform_bus,
	.name		= "physmap-flash",
	.id		= 1,
	.res		= nor_flash_resources,
	.num_res	= ARRAY_SIZE(nor_flash_resources),
	.data		= &nor_flash_data,
	.size_data	= sizeof(nor_flash_data),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* SPI NOR Flash */
/* Single Flash only */
static struct mtd_partition spibsc0_flash_partitions[] = {
	{
		.name		= "spibsc0_loader",
		.offset		= 0x00000000,
		.size		= 0x00080000,
		/* .mask_flags	= MTD_WRITEABLE, */
	},
	{
		.name		= "spibsc0_bootenv",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x00040000,
	},
	{
		.name		= "spibsc0_kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x00400000,
	},
	{
		.name		= "spibsc0_rootfs",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition spibsc1_flash_partitions[] = {
	{
		.name		= "spibsc1_data",
		.offset		= 0x00000000,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data spibsc0_flash_pdata = {
	.name	= "m25p80",
	.parts	= spibsc0_flash_partitions,
	.nr_parts = ARRAY_SIZE(spibsc0_flash_partitions),
	.type = "s25fl512s",
};

static struct flash_platform_data spibsc1_flash_pdata = {
	.name	= "m25p80",
	.parts	= spibsc1_flash_partitions,
	.nr_parts = ARRAY_SIZE(spibsc1_flash_partitions),
	.type = "s25fl512s",
};

/* QSPI Flash (Memory Map Mode, read only) */
/* Dual Flash */
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

/* PWM Pin (Pin TIOC4A only) */
/* RSKRZA1 does not have TIOC4A attached to anything */
#if 0
static const struct resource pwm_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcff0200, 0x4c),	/* mtu2_3,4 */
	DEFINE_RES_MEM(0xfcff0280, 0x6),	/* mtu2 share regs */
};

static const struct platform_device_info pwm0_info __initconst = {
	.parent		= &platform_bus,
	.name		= "rza1-pwm",
	.id		= 0,
	.res		= pwm_resources,
	.num_res	= ARRAY_SIZE(pwm_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* Backlight */
static struct platform_pwm_backlight_data pwm_backlight_pdata = {
	.max_brightness = 255,
	.dft_brightness = 255,
	.pwm_period_ns = 33333, /* 30kHz */
	.enable_gpio = -1,
};

static struct pwm_lookup pwm_lookup[] = {
	PWM_LOOKUP("rza1-pwm.0", 0, "pwm-backlight.0", NULL),
};

static const struct platform_device_info pwm_backlight_info __initconst = {
	.name		= "pwm-backlight",
	.data		= &pwm_backlight_pdata,
	.size_data	= sizeof(pwm_backlight_pdata),
};
#endif

/* RSPI */
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


static struct spi_board_info rskrza1_spi_devices[] __initdata = {
#if defined(RSPI_TESTING)
	{
		/* spidev */
		.modalias		= "spidev",
		.max_speed_hz           = 5000000,
		.bus_num                = 4,
		.chip_select            = 0,
		.mode			= SPI_MODE_3,
	},
#endif
	{
		/* SPI Flash0 */
		.modalias = "m25p80",
		.bus_num = 5,
		.chip_select = 0,
		.platform_data = &spibsc0_flash_pdata,
	},
	{
		/* SPI Flash1 */
		.modalias = "m25p80",
		.bus_num = 6,
		.chip_select = 0,
		.platform_data = &spibsc1_flash_pdata,
	},
};

/* spibsc0 */
static const struct sh_spibsc_info spibsc0_pdata = {
	.bus_num	= 5,
};

static const struct resource spibsc0_resources[] __initconst = {
	DEFINE_RES_MEM(0x3fefa000, 0x100),
};

static const struct platform_device_info spibsc0_info __initconst = {
	.name		= "spibsc",
	.id		= 0,
	.data 		= &spibsc0_pdata,
	.size_data	= sizeof(spibsc0_pdata),
	.num_res	= ARRAY_SIZE(spibsc0_resources),
	.res		= spibsc0_resources,
};

/* spibsc1 */
static const struct sh_spibsc_info spibsc1_pdata = {
	.bus_num	= 6,
};

static const struct resource spibsc1_resources[] __initconst = {
	DEFINE_RES_MEM(0x3fefb000, 0x100),
};

static const struct platform_device_info spibsc1_info __initconst = {
	.name		= "spibsc",
	.id		= 1,
	.data 		= &spibsc1_pdata,
	.size_data	= sizeof(spibsc1_pdata),
	.num_res	= ARRAY_SIZE(spibsc1_resources),
	.res		= spibsc1_resources,
};

/* ADC */
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

/* MMCIF */
static const struct resource sh_mmcif_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe804c800, 0x100, "MMCIF"),
	DEFINE_RES_IRQ(gic_iid(300)),
	DEFINE_RES_IRQ(gic_iid(301)),
};

static const struct sh_mmcif_plat_data sh_mmcif_pdata __initconst = {
	.sup_pclk	= 0,
	.ccs_unsupported = true,
	.ocr		= MMC_VDD_32_33,
	.caps		= MMC_CAP_4_BIT_DATA |
			  MMC_CAP_8_BIT_DATA |
			  MMC_CAP_NONREMOVABLE,
};

static const struct platform_device_info mmc_info __initconst = {
	.name		= "sh_mmcif",
	.id		= -1,
	.res		= sh_mmcif_resources,
	.num_res	= ARRAY_SIZE(sh_mmcif_resources),
	.data		= &sh_mmcif_pdata,
	.size_data	= sizeof(sh_mmcif_pdata),
};

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

/* USB Host */
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

/* USB Gadget */
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

/* Write to I2C device */
/* stolen from board-sx1.c */
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
/* stolen from board-sx1.c */
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

/* Audio */
static const struct platform_device_info alsa_soc_info = {
	.name		= "rskrza1_alsa_soc_platform",
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
//	{99, 23},	/* VDC1 */
	{126, 1},	/* JCU */
	{134, 2},	/* OSTM */
	{139, 1},	/* MTU2-TGI0A (Kernel jiffies) */
//	{170, 2}, {146, 2},	/* ADC and MTU2-TGI1A */
	{189, 8},	/* RIIC0 (Touchscreen) */
//	{197, 8},	/* RIIC1 */
//	{205, 8},	/* RIIC2 */
	{213, 8},	/* RIIC3 (Port Expander, EEPROM (MAC Addr), Audio Codec) */
//	{221, 4},	/* SCIF0 */
//	{225, 4},	/* SCIF1 */
	{229, 4},	/* SCIF2 (Console) */
//	{233, 4},	/* SCIF3 */
//	{237, 4},	/* SCIF4 */
//	{241, 4},	/* SCIF5 */
//	{245, 4},	/* SCIF6 */
//	{249, 4},	/* SCIF7 */
//	{270, 3},	/* RSPI0 */
//	{273, 3},	/* RSPI1 */
//	{276, 3},	/* RSPI2 */
//	{279, 3},	/* RSPI3 */
//	{282, 3},	/* RSPI4 */
	{299, 3},	/* MMC */
//	{302, 3},	/* SDHI0 */
	{305, 3},	/* SDHI1 */
	{308, 3},	/* RTC */
	{359, 1},	/* ETH */
};

static int irq_keep_all = 0;
static int __init early_irq_keep_all(char *str)
{
	irq_keep_all = 1;
	return 0;
}
early_param("irq_keep_all", early_irq_keep_all);

/* Removes unused pre-allocated IRQ. */
/* To skip this operation, add 'irq_keep_all' on the comamnd line */
static void remove_irqs(void)
{
	int i,j;
	int max = nr_irqs;
	int keep;

	if( irq_keep_all )
		return;		/* Feature disabled by 'irq_keep_all */

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

static void __init rskrza1_add_standard_devices(void)
{
#ifdef CONFIG_CACHE_L2X0
	/* Early BRESP enable, 16K*8way(defualt) */
	/* NOTES: BRESP can be set for IP version after r2p0 */
	/*        As of linux-3.16, cache-l2x0.c handles this automatically */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,16,0)
	l2x0_init(IOMEM(0xfffee000), 0x40000000, 0xffffffff);	/* Early BRESP enable */
#else
	l2x0_init(IOMEM(0xfffee000), 0x00000000, 0xffffffff);	/* Leave as defaults */
#endif
#endif
#if CONFIG_XIP_KERNEL
	remove_irqs();
#endif

	r7s72100_clock_init();
	r7s72100_pinmux_setup();
	r7s72100_add_dt_devices();

	r7s72100_pfc_pin_assign(P1_15, ALT1, DIIO_PBDC_EN);	/* AD7 */
	r7s72100_pfc_pin_assign(P1_0, ALT1, DIIO_PBDC_EN);	/* I2C SCL0 */
	r7s72100_pfc_pin_assign(P1_1, ALT1, DIIO_PBDC_EN);	/* I2C SDA0 */

	r7s72100_pfc_pin_assign(P4_4, ALT5, DIIO_PBDC_EN);	/* SSISCK0 */
	r7s72100_pfc_pin_assign(P4_5, ALT5, DIIO_PBDC_EN);	/* SSIWS0 */
	r7s72100_pfc_pin_assign(P4_6, ALT5, DIIO_PBDC_EN);	/* SSIRxD0 */
	r7s72100_pfc_pin_assign(P4_7, ALT5, SWIO_OUT_PBDCEN);	/* SSITxD0 */

#ifndef CONFIG_MMC_SDHI
	r7s72100_pfc_pin_assign(P3_8, ALT8, DIIO_PBDC_DIS);	/* MMC CD */
	r7s72100_pfc_pin_assign(P3_10, ALT8, DIIO_PBDC_DIS);	/* MMC DAT1 */
	r7s72100_pfc_pin_assign(P3_11, ALT8, DIIO_PBDC_DIS);	/* MMC DAT0 */
	r7s72100_pfc_pin_assign(P3_12, ALT8, DIIO_PBDC_DIS);	/* MMC CLK */
	r7s72100_pfc_pin_assign(P3_13, ALT8, DIIO_PBDC_DIS);	/* MMC CMD */
	r7s72100_pfc_pin_assign(P3_14, ALT8, DIIO_PBDC_DIS);	/* MMC DAT3*/
	r7s72100_pfc_pin_assign(P3_15, ALT8, DIIO_PBDC_DIS);	/* MMC DAT2 */
	r7s72100_pfc_pin_assign(P4_0, ALT8, DIIO_PBDC_DIS);	/* MMC DAT4 */
	r7s72100_pfc_pin_assign(P4_1, ALT8, DIIO_PBDC_DIS);	/* MMC DAT5 */
	r7s72100_pfc_pin_assign(P4_2, ALT8, DIIO_PBDC_DIS);	/* MMC DAT6*/
	r7s72100_pfc_pin_assign(P4_3, ALT8, DIIO_PBDC_DIS);	/* MMC DAT7 */
#else
	r7s72100_pfc_pin_assign(P3_8, ALT7, DIIO_PBDC_DIS);	/* SDHI1 CD */
	r7s72100_pfc_pin_assign(P3_9, ALT7, DIIO_PBDC_DIS);	/* SDHI1 WP */
	r7s72100_pfc_pin_assign(P3_10, ALT7, DIIO_PBDC_EN);	/* SDHI1 DAT1 */
	r7s72100_pfc_pin_assign(P3_11, ALT7, DIIO_PBDC_EN);	/* SDHI1 DAT0 */
	r7s72100_pfc_pin_assign(P3_12, ALT7, DIIO_PBDC_DIS);	/* SDHI1 CLK */
	r7s72100_pfc_pin_assign(P3_13, ALT7, DIIO_PBDC_EN);	/* SDHI1 CMD */
	r7s72100_pfc_pin_assign(P3_14, ALT7, DIIO_PBDC_EN);	/* SDHI1 DAT3*/
	r7s72100_pfc_pin_assign(P3_15, ALT7, DIIO_PBDC_EN);	/* SDHI1 DAT2 */
#endif
	gpio_irq_init();

	i2c_register_board_info(0, i2c0_devices, ARRAY_SIZE(i2c0_devices));
	i2c_register_board_info(3, i2c3_devices, ARRAY_SIZE(i2c3_devices));

#ifndef CONFIG_XIP_KERNEL	/* TODO: Uses too much internal RAM */
	platform_device_register_full(&jcu_info);
#endif
	platform_device_register_full(&ostm_info);
	platform_device_register_full(&dma_info);
	platform_device_register_full(&alsa_soc_info);
	platform_device_register_full(&scux_info);
	platform_device_register_full(&ether_info);

	platform_device_register_full(&riic0_info);	/* (Touchscreen) */
//	platform_device_register_full(&riic1_info);	/* Not used */
//	platform_device_register_full(&riic2_info);	/* Not used */
	platform_device_register_full(&riic3_info);	/* Port Expander, EEPROM (MAC Addr), Audio Codec */
	platform_device_register_full(&rtc_info);

#if !defined(CONFIG_XIP_KERNEL) && defined(CONFIG_SPI_SH_SPIBSC)
	/* Need to disable both spibsc channels if using memory mapped QSPI */
	platform_device_register_full(&spibsc0_info);
	platform_device_register_full(&spibsc1_info);
#else
	platform_device_register_full(&qspi_flash_info);
#endif

	platform_device_register_full(&nor_flash_info);
#if 0 /* For refernce only */
	platform_device_register_full(&pwm0_info);
	platform_device_register_full(&pwm_backlight_info);
	pwm_add_table(pwm_lookup, ARRAY_SIZE(pwm_lookup));
#endif
	platform_device_register_full(&adc0_info);
	platform_device_register_full(&sdhi0_info);

#ifndef CONFIG_MMC_SDHI
	platform_device_register_full(&mmc_info);
#else
	platform_device_register_full(&sdhi1_info);
#endif

	platform_device_register_full(&vdc5fb_info);

	if (usbgs == 0) {
		platform_device_register_full(&r8a66597_usb_gadget0_info);
		platform_device_register_full(&r8a66597_usb_host1_info);
	} else if (usbgs == 1) {
		platform_device_register_full(&r8a66597_usb_host0_info);
		platform_device_register_full(&r8a66597_usb_gadget1_info);
	} else {
		platform_device_register_full(&r8a66597_usb_host0_info);
		platform_device_register_full(&r8a66597_usb_host1_info);
	}

//	r7s72100_register_rspi(0);	/* Not used */
//	r7s72100_register_rspi(1);	/* Not used */
//	r7s72100_register_rspi(2);	/* Not used */
//	r7s72100_register_rspi(3);	/* Not used */
#if defined(RSPI_TESTING)
	r7s72100_register_rspi(4);	/* Not used */
#endif

	/* Register SPI device information */
	spi_register_board_info(rskrza1_spi_devices,
				ARRAY_SIZE(rskrza1_spi_devices));

}

static int heartbeat(void * data)
{
	u8 index = 0;
	u8 value;
	int ret;
	static const u8 pattern[8] = {7,6,5,3,7,3,5,6};

	while(1) {

		/* Register address 1 is the Output Contorl register */
		ret = rza1_i2c_read_byte(3, 0x20, 0x01, &value);
		value &= ~0x7;
		value |= pattern[index++];
		index &= 0x7;

		if( !ret )
			rza1_i2c_write_byte(3, 0x20, 0x01, value);

		msleep_interruptible(250);
	}

	return 0;
}

static void __init rskrza1_init_late(void)
{
	/* Make RSPI4 availible. */
	/* Done here because we have to wait till i2c driver is ready */
#if defined(RSPI_TESTING) && (defined CONFIG_SPI_RSPI) && !(defined CONFIG_SH_ETH)
	{
		int i;
		u8 value;

		/* Set P2_0-P2_11 and p3_3-p3_6 to input pins */
		for (i=0;i<=11;i++)
			r7s72100_pfc_pin_assign(P2_0+i, PMODE, DIR_IN);
		for (i=3;i<=6;i++)
			r7s72100_pfc_pin_assign(P3_0+i, PMODE, DIR_IN);

		/* Set PX1_EN1 to 0 to disable Ethernet */
		/* PX1_EN1 is controled through Port Expanders on I2C3 */
		/* Register address 1 is the Output Control register */
		i = rza1_i2c_read_byte(3, 0x21, 0x01, &value);
		value &= ~0x02;		/* bit 1 = L */
		if ( !i )
			i = rza1_i2c_write_byte(3, 0x21, 0x01, value);
		if ( !i ) {
			/* Enable SPI4 pins */
			/* RSKRZA1 Board SPI4 is on CN15 (but that means you can use Ethernet) */
			r7s72100_pfc_pin_assign(P2_8, ALT8, DIIO_PBDC_EN);	/* RSPCK4 */
			r7s72100_pfc_pin_assign(P2_9, ALT8, DIIO_PBDC_EN);	/* SSL40 */
			r7s72100_pfc_pin_assign(P2_10, ALT8, DIIO_PBDC_EN);	/* MOSI4 */
			r7s72100_pfc_pin_assign(P2_11, ALT8, DIIO_PBDC_EN);	/* MISO4 */
		}
		if( !i )
			printk("%s: RSPI4 enabled on CN15\n",__func__);
		else
			printk("%s: ERROR: Failed to set pins for RSPI4 usage\n",__func__);
	}
#endif

	/* Start heardbeat kernel thread */
	kthread_run(heartbeat, NULL,"heartbeat");
}


static const char * const rskrza1_boards_compat_dt[] __initconst = {
	"renesas,rskrza1",
	NULL,
};

DT_MACHINE_START(RSKRZA1_DT, "rskrza1")
	.init_early	= r7s72100_init_early,
	.init_machine	= rskrza1_add_standard_devices,
	.init_late	= rskrza1_init_late,
	.dt_compat	= rskrza1_boards_compat_dt,
	.map_io		= rza1_map_io,
MACHINE_END
