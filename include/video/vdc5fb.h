/*
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

#ifndef _VDC5FB_H_
#define _VDC5FB_H_

#ifdef __KERNEL__

#include <linux/fb.h>

/* NUMBER OF CHANNELS */
#define	VDC5FB_NUM_CH		2

/* NUMBER OF RESOURCES */
#define	VDC5FB_NUM_RES		3
/* MEM resource 0 is used for registers */
/* MEM resource 1 is used for framebuffer(s) */
/* IRQ resource 0 is used for irq numbers */

/* BASE ADDRESS AND SIZE OF REGISTERS */
#define	VDC5FB_REG_BASE(x)	(0xFCFF6000 + (0x2000 * (x)))
#define	VDC5FB_REG_SIZE		0x2000

/* START AND TOTAL NUMBER OF IRQS */
#define	VDC5FB_IRQ_BASE(x)	(75 + (24 * (x)))
#define	VDC5FB_IRQ_SIZE		23

/* clock_source */
enum {
	ICKSEL_INPSEL = 0,
	ICKSEL_EXTCLK0,
	ICKSEL_EXTCLK1,
	ICKSEL_P1CLK,
};

/* tcon_sel */
enum {				/* index */
	LCD_TCON0 = 0,
	LCD_TCON1,
	LCD_TCON2,
	LCD_TCON3,
	LCD_TCON4,
	LCD_TCON5,
	LCD_TCON6,
	LCD_MAX_TCON,
};
#define	TCON_SEL_STVA		0	/* STVA/VS */
#define	TCON_SEL_STVB		1	/* STVB/VE */
#define	TCON_SEL_STH		2	/* STH/SP/HS */
#define	TCON_SEL_STB		3	/* STB/LP/HE */
#define	TCON_SEL_CPV		4	/* CPV/GCK */
#define	TCON_SEL_POLA		5	/* POLA */
#define	TCON_SEL_POLB		6	/* POLB */
#define	TCON_SEL_DE		7	/* DE */
#define	TCON_SEL_UNUSED		0xff

/* out_format */
#define	OUT_FORMAT_RGB888	0
#define	OUT_FORMAT_RGB666	1
#define	OUT_FORMAT_RGB565	2

/* board-specific data */
struct vdc5fb_pdata {
	const char *name;
	struct fb_videomode *videomode;
	int bpp;		/* should be 16 or 32 */
	int panel_icksel;	/* should be ICKSEL_P1CLK */
	unsigned long panel_width;
	unsigned long panel_height;
	unsigned long flm_max;
	int out_format;
	int use_lvds;
	int flags;
#define	FLAGS_RESIZE	0x01	/* enable resize */
#define	FLAGS_RETIMING	0x02	/* enable resize */
#define	FLAGS_DS	0x04	/* enable down-scale */
	unsigned char tcon_sel[LCD_MAX_TCON];
/* board specific setting function */
	int (*pinmux)(struct platform_device *pdev);
};
#endif /* __KERNEL__ */

/*****************/
/* VDC5FB IOCTLs */
/*****************/

#define	FBIOGET_CONTRAST	_IOR('F', 0x21, int)
#define	FBIOPUT_CONTRAST	_IOW('F', 0x22, int)
#define	FBIOGET_BRIGHT		_IOR('F', 0x23, int)
#define	FBIOPUT_BRIGHT		_IOW('F', 0x24, int)
#define	FBIOGET_FRAME		_IOW('F', 0x25, int)
#define	FBIOPUT_FRAME		_IOW('F', 0x26, int)

struct fbio_bright {
	unsigned short pbrt_r;	/* 0-1023, inclusive */
	unsigned short pbrt_g;
	unsigned short pbrt_b;
};

struct fbio_contrast {
	unsigned short cont_r;	/* 0-255, inclusive */
	unsigned short cont_g;
	unsigned short cont_b;
};

struct fbio_frame {
	unsigned short fr_num;	/* 0-(frame_max-1), inclusive */
	unsigned short fr_max;
};

#endif /* _VDC5FB_H_ */
