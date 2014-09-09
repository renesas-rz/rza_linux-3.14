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
#include <linux/i2c-riic.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_data/sh_adc.h>
#include <linux/usb/r8a66597.h>
#include <linux/platform_data/dma-rza1.h>
#include <linux/uio_driver.h>
#include <video/vdc5fb.h>

static int usbgs = -1;
static int __init early_usbgs(char *str)
{
	usbgs = 0;
	get_option(&str, &usbgs);
	return 0;
}
early_param("usbgs", early_usbgs);

static struct map_desc rza1_io_desc[] __initdata = {
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

static const struct rza1_dma_slave_config rza1_dma_slaves[] __initconst = {
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
	.channel_num	= 16,
};

static const struct resource rza1_dma_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8200000, 0x1000),
	DEFINE_RES_MEM(0xfcfe1000, 0x1000),
	DEFINE_RES_IRQ(gic_iid(41)),
	DEFINE_RES_IRQ(gic_iid(42)),
	DEFINE_RES_IRQ(gic_iid(43)),
	DEFINE_RES_IRQ(gic_iid(44)),
	DEFINE_RES_IRQ(gic_iid(45)),
	DEFINE_RES_IRQ(gic_iid(46)),
	DEFINE_RES_IRQ(gic_iid(47)),
	DEFINE_RES_IRQ(gic_iid(48)),
	DEFINE_RES_IRQ(gic_iid(49)),
	DEFINE_RES_IRQ(gic_iid(50)),
	DEFINE_RES_IRQ(gic_iid(51)),
	DEFINE_RES_IRQ(gic_iid(52)),
	DEFINE_RES_IRQ(gic_iid(53)),
	DEFINE_RES_IRQ(gic_iid(54)),
	DEFINE_RES_IRQ(gic_iid(55)),
	DEFINE_RES_IRQ(gic_iid(56)),
	DEFINE_RES_IRQ(gic_iid(57)),
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

static const struct resource vdc5fb_resources[VDC5FB_NUM_RES] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcff6000, 0x00002000, "vdc5fb.0: reg"),
	[1] = DEFINE_RES_MEM_NAMED(0x60200000, 0x00400000, "vdc5fb.0: fb"),
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
	.bpp			= 32,
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
};

static const struct platform_device_info vdc5fb_info __initconst = {
	.name		= "vdc5fb",
	.id		= 0,
	.res		= vdc5fb_resources,
	.num_res	= ARRAY_SIZE(vdc5fb_resources),
	.data		= &vdc5fb_gwp0700cnwv04_pdata,
	.size_data	= sizeof(vdc5fb_gwp0700cnwv04_pdata),
};

/* JCU */
static const struct uio_info jcu_platform_pdata __initconst = {
	.name = "JCU",
	.version = "0",
	.irq = 126, /* Not used */
};

static const struct resource jcu_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe8017000, 0x3d2, "jcu:reg"), /* for JCU of RZ */
	DEFINE_RES_MEM_NAMED(0xfcfe0000, 0x2000, "jcu:rstreg clkreg"), /* Use STBCR6 & SWRSTCR2 */
	DEFINE_RES_MEM_NAMED(0x60900000, 0x100000, "jcu:iram"), /* (Non cacheable 1MB) */
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

/* RTC */
static const struct resource rtc_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcff1000, 0x2d),
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
		.name		= "u-boot",
		.offset		= 0x00000000,
		.size		= SZ_512K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "u-boot_env",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_256K,
	},
	{
		.name		= "dtb",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_256K,
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
	},
	{
		.name		= "data",
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
	.id		= -1,
	.res		= nor_flash_resources,
	.num_res	= ARRAY_SIZE(nor_flash_resources),
	.data		= &nor_flash_data,
	.size_data	= sizeof(nor_flash_data),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* PWM */
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
	platform_device_register_resndata(&platform_bus, "rspi", idx,   \
					rspi##idx##_resources,		   \
					ARRAY_SIZE(rspi##idx##_resources), \
					&rspi_pdata, sizeof(rspi_pdata))

/* spibsc0 */
static const struct sh_spibsc_info spibsc0_pdata __initconst = {
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
static const struct sh_spibsc_info spibsc1_pdata __initconst = {
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
	r7s72100_clock_init();
	r7s72100_pinmux_setup();
	r7s72100_add_dt_devices();

	r7s72100_pfc_pin_assign(P1_15, ALT1, DIIO_PBDC_EN);	/* AD7 */

	platform_device_register_full(&jcu_info);
	platform_device_register_full(&dma_info);
	platform_device_register_full(&ether_info);
	platform_device_register_full(&riic0_info);
	platform_device_register_full(&riic1_info);
	platform_device_register_full(&riic2_info);
	platform_device_register_full(&riic3_info);
	platform_device_register_full(&rtc_info);
	platform_device_register_full(&nor_flash_info);
	platform_device_register_full(&pwm0_info);
	platform_device_register_full(&spibsc0_info);
	platform_device_register_full(&spibsc1_info);
	platform_device_register_full(&adc0_info);
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

	r7s72100_register_rspi(0);
	r7s72100_register_rspi(1);
	r7s72100_register_rspi(2);
	r7s72100_register_rspi(3);
	r7s72100_register_rspi(4);
}

static const char * const rskrza1_boards_compat_dt[] __initconst = {
	"renesas,rskrza1",
	NULL,
};

DT_MACHINE_START(RSKRZA1_DT, "rskrza1")
	.init_early	= r7s72100_init_early,
	.init_machine	= rskrza1_add_standard_devices,
	.dt_compat	= rskrza1_boards_compat_dt,
	.map_io		= rza1_map_io,
MACHINE_END
