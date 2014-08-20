/*
 * SuperH SPIBSC bus driver
 *
 * Copyright (C) 2011-2014 Renesas Solutions Corp.
 *
 * Based on spi-sh-hspi.c:
 *
 * Copyright (C) 2011  Kuninori Morimoto
 *
 * Based on spi-sh.c:
 * Based on pxa2xx_spi.c:
 * Copyright (C) 2005 Stephen Street / StreetFire Sound Labs
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
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/spi/sh_spibsc.h>

/* SPIBSC registers */
#define	CMNCR	0x00
#define	SSLDR	0x04
#define SPBCR	0x08
#define DRCR	0x0c
#define	SMCR	0x20
#define	SMCMR	0x24
#define	SMADR	0x28
#define	SMOPR	0x2c
#define	SMENR	0x30
#define SMRDR0	0x38
#define SMRDR1	0x3c
#define	SMWDR0	0x40
#define SMWDR1	0x44
#define	CMNSR	0x48
#define SMDMCR	0x60
#define SMDRENR	0x64

/* CMNCR */
#define	CMNCR_MD	(1u << 31)
#define	CMNCR_SFDE	(1u << 24)

#define	CMNCR_MOIIO3(x)		(((u32)(x) & 0x3) << 22)
#define	CMNCR_MOIIO2(x)		(((u32)(x) & 0x3) << 20)
#define	CMNCR_MOIIO1(x)		(((u32)(x) & 0x3) << 18)
#define	CMNCR_MOIIO0(x)		(((u32)(x) & 0x3) << 16)
#define	CMNCR_IO3FV(x)		(((u32)(x) & 0x3) << 14)
#define	CMNCR_IO2FV(x)		(((u32)(x) & 0x3) << 12)
#define	CMNCR_IO0FV(x)		(((u32)(x) & 0x3) << 8)

#define	CMNCR_CPHAT	(1u << 6)
#define	CMNCR_CPHAR	(1u << 5)
#define	CMNCR_SSLP	(1u << 4)
#define	CMNCR_CPOL	(1u << 3)
#define	CMNCR_BSZ(n)	(((u32)(n) & 0x3) << 0)

#define	OUT_0		(0u)
#define	OUT_1		(1u)
#define	OUT_REV		(2u)
#define	OUT_HIZ		(3u)

#define	BSZ_SINGLE	(0)
#define	BSZ_DUAL	(1)

#define CMNCR_INIT	(CMNCR_MD | \
			CMNCR_SFDE | \
			CMNCR_MOIIO3(OUT_HIZ) | \
			CMNCR_MOIIO2(OUT_HIZ) | \
			CMNCR_MOIIO1(OUT_HIZ) | \
			CMNCR_MOIIO0(OUT_HIZ) | \
			CMNCR_IO3FV(OUT_HIZ) | \
			CMNCR_IO2FV(OUT_HIZ) | \
			CMNCR_IO0FV(OUT_HIZ) | \
			CMNCR_CPHAR | \
			CMNCR_BSZ(BSZ_SINGLE))

/* SSLDR */
#define	SSLDR_SPNDL(x)	(((u32)(x) & 0x7) << 16)
#define	SSLDR_SLNDL(x)	(((u32)(x) & 0x7) << 8)
#define	SSLDR_SCKDL(x)	(((u32)(x) & 0x7) << 0)

#define	SPBCLK_1_0	(0)
#define	SPBCLK_1_5	(0)
#define	SPBCLK_2_0	(1)
#define	SPBCLK_2_5	(1)
#define	SPBCLK_3_0	(2)
#define	SPBCLK_3_5	(2)
#define	SPBCLK_4_0	(3)
#define	SPBCLK_4_5	(3)
#define	SPBCLK_5_0	(4)
#define	SPBCLK_5_5	(4)
#define	SPBCLK_6_0	(5)
#define	SPBCLK_6_5	(5)
#define	SPBCLK_7_0	(6)
#define	SPBCLK_7_5	(6)
#define	SPBCLK_8_0	(7)
#define	SPBCLK_8_5	(7)

#define	SSLDR_INIT	(SSLDR_SPNDL(SPBCLK_1_0) | \
			SSLDR_SLNDL(SPBCLK_1_0) | \
			SSLDR_SCKDL(SPBCLK_1_0))

/* SPBCR */
#define	SPBCR_SPBR(x)		(((u32)(x) & 0xff) << 8)
#define	SPBCR_BRDV(x)		(((u32)(x) & 0x3) << 0)


#define SPBCR_INIT	(SPBCR_SPBR(2) | SPBCR_BRDV(0))	/* Clock : 33MHz */

/* DRCR (read mode) */
#define	DRCR_SSLN		(1u << 24)
#define	DRCR_RBURST(x)		(((u32)(x) & 0xf) << 16)
#define	DRCR_RCF		(1u << 9)
#define	DRCR_RBE		(1u << 8)
#define	DRCR_SSLE		(1u << 0)

/* DRCMR (read mode) */
#define	DRCMR_CMD(c)		(((u32)(c) & 0xff) << 16)
#define	DRCMR_OCMD(c)		(((u32)(c) & 0xff) << 0)

/* DREAR (read mode) */
#define	DREAR_EAV(v)		(((u32)(v) & 0xff) << 16)
#define	DREAR_EAC(v)		(((u32)(v) & 0x7) << 0)

/* DROPR (read mode) */
#define	DROPR_OPD3(o)		(((u32)(o) & 0xff) << 24)
#define	DROPR_OPD2(o)		(((u32)(o) & 0xff) << 16)
#define	DROPR_OPD1(o)		(((u32)(o) & 0xff) << 8)
#define	DROPR_OPD0(o)		(((u32)(o) & 0xff) << 0)

/* DRENR (read mode) */
#define	DRENR_CDB(b)		(((u32)(b) & 0x3) << 30)
#define	DRENR_OCDB(b)		(((u32)(b) & 0x3) << 28)
#define	DRENR_ADB(b)		(((u32)(b) & 0x3) << 24)
#define	DRENR_OPDB(b)		(((u32)(b) & 0x3) << 20)
#define	DRENR_DRDB(b)		(((u32)(b) & 0x3) << 16)
#define	DRENR_DME		(1u << 15)
#define	DRENR_CDE		(1u << 14)
#define	DRENR_OCDE		(1u << 12)
#define	DRENR_ADE(a)		(((u32)(a) & 0xf) << 8)
#define	DRENR_OPDE(o)		(((u32)(o) & 0xf) << 4)

/* SMCR (spi mode) */
#define	SMCR_SSLKP		(1u << 8)
#define	SMCR_SPIRE		(1u << 2)
#define	SMCR_SPIWE		(1u << 1)
#define	SMCR_SPIE		(1u << 0)

/* SMCMR (spi mode) */
#define	SMCMR_CMD(c)		(((u32)(c) & 0xff) << 16)
#define	SMCMR_OCMD(o)		(((u32)(o) & 0xff) << 0)

/* SMADR (spi mode) */

/* SMOPR (spi mode) */
#define	SMOPR_OPD3(o)		(((u32)(o) & 0xff) << 24)
#define	SMOPR_OPD2(o)		(((u32)(o) & 0xff) << 16)
#define	SMOPR_OPD1(o)		(((u32)(o) & 0xff) << 8)
#define	SMOPR_OPD0(o)		(((u32)(o) & 0xff) << 0)

/* SMENR (spi mode) */
#define	SMENR_CDB(b)		(((u32)(b) & 0x3) << 30)
#define	SMENR_OCDB(b)		(((u32)(b) & 0x3) << 28)
#define	SMENR_ADB(b)		(((u32)(b) & 0x3) << 24)
#define	SMENR_OPDB(b)		(((u32)(b) & 0x3) << 20)
#define	SMENR_SPIDB(b)		(((u32)(b) & 0x3) << 16)
#define	SMENR_DME		(1u << 15)
#define	SMENR_CDE		(1u << 14)
#define	SMENR_OCDE		(1u << 12)
#define	SMENR_ADE(b)		(((u32)(b) & 0xf) << 8)
#define	SMENR_OPDE(b)		(((u32)(b) & 0xf) << 4)
#define	SMENR_SPIDE(b)		(((u32)(b) & 0xf) << 0)

#define	ADE_23_16	(0x4)
#define	ADE_23_8	(0x6)
#define	ADE_23_0	(0x7)
#define	ADE_31_0	(0xf)

#define	BITW_1BIT	(0)
#define	BITW_2BIT	(1)
#define	BITW_4BIT	(2)

#define	SPIDE_8BITS	(0x8)
#define	SPIDE_16BITS	(0xc)
#define	SPIDE_32BITS	(0xf)

#define	OPDE_3		(0x8)
#define	OPDE_3_2	(0xc)
#define	OPDE_3_2_1	(0xe)
#define	OPDE_3_2_1_0	(0xf)

/* SMRDR0 (spi mode) */
/* SMRDR1 (spi mode) */
/* SMWDR0 (spi mode) */
/* SMWDR1 (spi mode) */

/* CMNSR (spi mode) */
#define	CMNSR_SSLF	(1u << 1)
#define	CMNSR_TEND	(1u << 0)

/* DRDMCR (read mode) */
#define	DRDMCR_DMDB(b)		(((u32)(b) & 0x3) << 16)
#define	DRDMCR_DMCYC(b)		(((u32)(b) & 0x7) << 0)

/* DRDRENR (read mode) */
#define	DRDRENR_ADDRE	(1u << 8)
#define	DRDRENR_OPDRE	(1u << 4)
#define	DRDRENR_DRDRE	(1u << 0)

/* SMDMCR (spi mode) */
#define	SMDMCR_DMDB(b)		(((u32)(b) & 0x3) << 16)
#define	SMDMCR_DMCYC(b)		(((u32)(b) & 0x7) << 0)

/* SMDRENR (spi mode) */
#define	SMDRENR_ADDRE	(1u << 8)
#define	SMDRENR_OPDRE	(1u << 4)
#define	SMDRENR_SPIDRE	(1u << 0)

/*
 *  FlashROM Chip Commands
 */
#define	CMD_READ_ID	(0x90)	/* Read Electronic Manufacturer Signature */
#define	CMD_PP		(0x02)	/* Page Program (3-byte address) */
#define	CMD_QPP		(0x32)	/* Quad Page Program (3-byte address) */
#define	CMD_READ	(0x03)	/* Read (3-byte address) */
#define	CMD_FAST_READ	(0x0b)	/* Fast Read (3-byte address) */
#define	CMD_DOR		(0x3b)	/* Read Dual Out (3-byte address) */
#define	CMD_QOR		(0x6b)	/* Read Quad Out (3-byte address) */
#define	CMD_WREN	(0x06)	/* Write enable */
#define CMD_RDSR	(0x05)	/* Read status */
#define CMD_SE		(0xd8)	/* Sector erase */
#define	CMD_RDID	(0x9f)	/* Read Identification */
#define	CMD_WRR		(0x01)	/* Write SR1, CR1 register */
#define	CMD_RDCR	(0x35)	/* Read configuration register */
#define	CMD_BRWR	(0x17)	/* Bank register write*/
#define	CMD_CLSR	(0x30)	/* clear E_ERR(SR1:bit5) and P_ERR(SR1:bit6) */
#define	CMD_WRDI	(0x04)	/* set WEL(SR1:bit1)bit to 0 */
#define	CMD_RESET	(0xF0)	/* Software Reset */
#define CMD_MBR		(0xFF)	/* Mode Bit Reset */

/* SR1 register bit */
#define	SR1_SRWD	0x80	/* Status Register Write Disable */
#define	SR1_P_ERR	0x40	/* Programming Error */
#define	SR1_E_ERR	0x20	/* Erase Error */
#define	SR1_BP_MASK	0x1c	/* Block Protection */
#define	SR1_WEL		0x02	/* Write Enable Latch */
#define	SR1_WIP		0x01	/* Write In Progress */
/* CR1 register bit */
#define	CR1_LC_MASK	0xc0	/* Latency Code */
#define	CR1_BPNV	0x08	/* Block Protection Non-Volatile */
#define	CR1_QUAD	0x02	/* Quad Data Width */
#define	CR1_FREEZE	0x01	/* Freeze Protection */

/* SPIBSC registers */
#define	CMNCR	0x00
#define	SSLDR	0x04
#define SPBCR	0x08
#define DRCR	0x0c
#define	SMCR	0x20
#define	SMCMR	0x24
#define	SMADR	0x28
#define	SMOPR	0x2c
#define	SMENR	0x30
#define SMRDR0	0x38
#define SMRDR1	0x3c
#define	SMWDR0	0x40
#define SMWDR1	0x44
#define	CMNSR	0x48
#define SMDMCR	0x60
#define SMDRENR	0x64

struct spibsc_priv {
	void __iomem *addr;
	struct spi_master *master;
	struct device *dev;
	struct clk *clk;
	struct sh_spibsc_info *info;
	u8 bspw;	/* bits per word */
	u32 max_speed;	/* max speed hz */
	u32 bitw;	/* data_bitw */
	u32 dcyle;	/* dmy_cycle */
};

#define BITW_1BIT	(0)
#define BITW_2BIT	(1)
#define BITW_4BIT	(2)

#undef DEBUG
#ifdef DEBUG
#define	DEBUG_COMMAND()						\
	do {							\
		int i;						\
		dev_err(sbsc->dev, "command:");			\
		for (i = 0; i < len; i++)			\
			dev_err(sbsc->dev, " %02X", command[i]);\
		dev_err(sbsc->dev, "\n");			\
	} while (0)
#define	DEBUG_SEND()						\
	do {							\
		int i;						\
		dev_err(sbsc->dev, "send data:");		\
		for (i = 0; i < len; i++)			\
			dev_err(sbsc->dev, " %02X", data[i]);	\
			dev_err(sbsc->dev, "\n");		\
	} while (0)
#define DEBUG_RECEIVE()						\
	do {							\
		int i;						\
		dev_err(sbsc->dev, "receive data:");		\
		for (i = 0; i < len; i++)			\
			dev_err(sbsc->dev, " %02X", data[i]);	\
			dev_err(sbsc->dev, "\n");		\
	} while (0)
#else
#define	DEBUG_COMMAND()	do {} while (0)
#define	DEBUG_SEND()	do {} while (0)
#define DEBUG_RECEIVE()	do {} while (0)
#endif

/* select value for SMENR.SPIDBbit(bit width) */
/*     and SMDMCR.DMCYC(number of dummy cycle)*/
static void spibsc_set_busio(struct spibsc_priv *sbsc, u8 cmd)
{
	u32 bitw, dcyle;

	switch (cmd) {
	case CMD_FAST_READ:	/* 0x0b Fast Read (3-byte address) */
		dcyle = 8;
		bitw = BITW_1BIT;
		break;

	case CMD_DOR:		/* 0x3b Read Dual Out (3-byte address) */
		dcyle = 8;
		bitw = BITW_2BIT;
		break;

	case CMD_QOR:		/* 0x6b Read Quad Out (3-byte address) */
		dcyle = 8;
		bitw = BITW_4BIT;
		break;

	case CMD_QPP:		/* 0x32 Quad Page Program (3-byte address) */
		dcyle = 0;
		bitw = BITW_4BIT;
		break;

	default:
		dcyle = 0;
		bitw = BITW_1BIT;
		break;
	}

	sbsc->bitw	= bitw;
	sbsc->dcyle	= dcyle;
}

static void spibsc_write(struct spibsc_priv *sbsc, int reg, u32 val)
{
	iowrite32(val, sbsc->addr + reg);
}
static void spibsc_write8(struct spibsc_priv *sbsc, int reg, u8 val)
{
	iowrite8(val, sbsc->addr + reg);
}
static void spibsc_write16(struct spibsc_priv *sbsc, int reg, u16 val)
{
	iowrite16(val, sbsc->addr + reg);
}

static u32 spibsc_read(struct spibsc_priv *sbsc, int reg)
{
	return ioread32(sbsc->addr + reg);
}
static u8 spibsc_read8(struct spibsc_priv *sbsc, int reg)
{
	return ioread8(sbsc->addr + reg);
}
static u16 spibsc_read16(struct spibsc_priv *sbsc, int reg)
{
	return ioread16(sbsc->addr + reg);
}


static int spibsc_wait_trans_completion(struct spibsc_priv *sbsc)
{
	int t = 256 * 100000;

	while (t--) {
		if (spibsc_read(sbsc, CMNSR) & CMNSR_TEND)
			return 0;

		ndelay(1);
	}

	dev_err(sbsc->dev, "timeout\n");
	return -ETIMEDOUT;
}

static int spibsc_do_send_data(
	struct spibsc_priv *sbsc,
	const u8 *data,
	int len);

static int spibsc_do_send_cmd(
	struct spibsc_priv *sbsc,
	const u8 *command,
	int clen)
{
	int ret;
	u32 cmd, addr = 0, smopr = 0, smenr, smcr, smdmcr = 0;

	cmd = SMCMR_CMD(command[0]);
	smenr = SMENR_CDE | SMENR_CDB(BITW_1BIT);

	switch (clen) {
	case 1:
		/* NOTHING TO DO... */
		if (command[0] == CMD_WREN)
			break;	/* WREN */
		if (command[0] == CMD_WRDI)
			break;	/* WRDI */
		if (command[0] == CMD_CLSR)
			break;	/* CLSR */
		if (command[0] == CMD_RDSR)
			break;	/* RDSR + read-data(1) */
		if (command[0] == CMD_RDCR)
			break;	/* RDCR + read-data(1) */
		if (command[0] == CMD_RDID)
			break;	/* RDID + read-data */
		if (command[0] == CMD_RESET)
			break;	/* Software Reset */
		if (command[0] == CMD_MBR)
			break;	/* Mode Bit Reset */
		goto _show_me;
		break;
	case 2:
		smopr |= SMOPR_OPD3(command[1]);
		/* output optional command(op3) */
		smenr |= SMENR_OPDE(OPDE_3) | SMENR_OPDB(BITW_1BIT);

		if (command[0] == CMD_BRWR)
			break;	/* BRWR + write-data(1) */
		goto _show_me;
		break;
	case 3:
		smopr |= SMOPR_OPD3(command[1]);
		smopr |= SMOPR_OPD2(command[2]);
		/* output optional command(op3,2) */
		smenr |= SMENR_OPDE(OPDE_3_2) | SMENR_OPDB(BITW_1BIT);

		if (command[0] == CMD_WRR)
			break;	/* WRR + write-data(1or2) */
		goto _show_me;
		break;
	case 4:		/* set 3byte address */
		addr = (command[1] << 16) |
			   (command[2] <<  8) |
			    command[3];

		smenr |= SMENR_ADE(ADE_23_0) | SMENR_ADB(BITW_1BIT);

		/* set dummy param. */
		if (sbsc->dcyle > 0) {
			smenr |= SMENR_DME;
			smdmcr = SMDMCR_DMDB(BITW_1BIT) |
					 SMDMCR_DMCYC(sbsc->dcyle - 1);
		}

		if (command[0] == CMD_READ)
			break;	/* READ      + ADDR24 */
		if (command[0] == CMD_FAST_READ)
			break;	/* FAST_READ + ADDR24 */
		if (command[0] == CMD_DOR)
			break;	/* DOR       + ADDR24 */
		if (command[0] == CMD_QOR)
			break;	/* QOR       + ADDR24 */
		if (command[0] == CMD_PP)
			break;	/* PP        + ADDR24 */
		if (command[0] == CMD_QPP)
			break;	/* QPP       + ADDR32 */
		if (command[0] == CMD_SE)
			break;	/* SE        + ADDR24 */
		if (command[0] == CMD_READ_ID)
			break;	/* READ_ID   + ADDR24 */
		goto _show_me;
		break;
	case 6:
		/* CMD_FAST_READ with 4 byte address comes here */
		/* m25p80 adds 1 extra byte as dummy cycle... */

		/* FALL INTO */
	case 5:		/* set 4byte address */
		addr =	(command[1] << 24) |
				(command[2] << 16) |
				(command[3] << 8)  |
				 command[4];

		smenr |= SMENR_ADE(ADE_31_0) | SMENR_ADB(BITW_1BIT);

		/* set dummy param. */
		if (sbsc->dcyle > 0) {
			smenr |= SMENR_DME;
			smdmcr = SMDMCR_DMDB(BITW_1BIT) |
					 SMDMCR_DMCYC(sbsc->dcyle - 1);
		}

		if (command[0] == CMD_READ)
			break;	/* READ      + ADDR32 */
		if (command[0] == CMD_FAST_READ)
			break;	/* FAST_READ + ADDR32 */
		if (command[0] == CMD_DOR)
			break;	/* DOR       + ADDR32 */
		if (command[0] == CMD_QOR)
			break;	/* QOR       + ADDR32 */
		if (command[0] == CMD_PP)
			break;	/* PP        + ADDR32 */
		if (command[0] == CMD_QPP)
			break;	/* QPP       + ADDR32 */
		if (command[0] == CMD_SE)
			break;	/* SE        + ADDR32 */
		goto _show_me;
		break;
_show_me:
	default:
		pr_err("%s: ERROR bad clen(%d) or command(%02x)\n",
							__func__, clen, cmd);
		return -ENODATA;
		break;
	}

	/* set params */
	spibsc_write(sbsc, SMCMR, cmd);
	spibsc_write(sbsc, SMADR, addr);
	spibsc_write(sbsc, SMOPR, smopr);
	spibsc_write(sbsc, SMENR, smenr);
	spibsc_write(sbsc, SMDMCR, smdmcr);

	/* start spi transfer*/
	smcr = SMCR_SPIE;
	switch (command[0]) {
	case CMD_WREN:
	case CMD_SE:
	case CMD_WRR:
	case CMD_BRWR:
	case CMD_CLSR:
	case CMD_WRDI:
	case CMD_RESET:
	case CMD_MBR:
		break;
	default:
		/* Hold signal level (SPBSSL#(=CS#)) up to the next access */
		smcr |= SMCR_SSLKP;
		break;
	}
	spibsc_write(sbsc, SMCR, smcr);

	/* wait for spi transfer completed */
	ret = spibsc_wait_trans_completion(sbsc);
	if (ret)
		return ret;	/* return error */

	if (command[0] == CMD_RESET)
		udelay(40);

	return 0;
}

static int spibsc_do_send_data(
	struct spibsc_priv *sbsc,
	const u8 *data,
	int len)
{
	u32 smcr, smenr, smwdr0;
	int ret, unit, sslkp = 1;

	while (len > 0) {
		if (len >= 4) {
			unit = 4;
			smenr = SMENR_SPIDE(SPIDE_32BITS) |
					SMENR_SPIDB(sbsc->bitw);
		} else {
			unit = len;
			if (unit == 3)
				unit = 2;

			if (unit >= 2)
				smenr = SMENR_SPIDE(SPIDE_16BITS) |
						SMENR_SPIDB(sbsc->bitw);
			else
				smenr = SMENR_SPIDE(SPIDE_8BITS) |
						SMENR_SPIDB(sbsc->bitw);
		}

		/* set 4bytes data, bit stream */
		smwdr0 = *data++;
		if (unit >= 2)
			smwdr0 |= (u32)(*data++ << 8);
		if (unit >= 3)
			smwdr0 |= (u32)(*data++ << 16);
		if (unit >= 4)
			smwdr0 |= (u32)(*data++ << 24);

		/* mask unwrite area */
		if (unit == 3)
			smwdr0 |= 0xFF000000;
		else if (unit == 2)
			smwdr0 |= 0xFFFF0000;
		else if (unit == 1)
			smwdr0 |= 0xFFFFFF00;

		/* write send data. */
		if (unit == 2)
			spibsc_write16(sbsc, SMWDR0, (u16)smwdr0);
		else if (unit == 1)
			spibsc_write8(sbsc, SMWDR0, (u8)smwdr0);
		else
		spibsc_write(sbsc, SMWDR0, smwdr0);

		len -= unit;
		if (len <= 0)
			sslkp = 0;

		/* set params */
		spibsc_write(sbsc, SMCMR, 0);
		spibsc_write(sbsc, SMADR, 0);
		spibsc_write(sbsc, SMOPR, 0);
		spibsc_write(sbsc, SMENR, smenr);

		/* start spi transfer */
		smcr = SMCR_SPIE|SMCR_SPIWE;
		if (sslkp)
			smcr |= SMCR_SSLKP;
		spibsc_write(sbsc, SMCR, smcr);

		/* wait for spi transfer completed */
		ret = spibsc_wait_trans_completion(sbsc);
		if (ret)
			return  ret;	/* return error */
	}
	return 0;
}

static int spibsc_do_receive_data(struct spibsc_priv *sbsc, u8 *data, int len)
{
	u32 smcr, smenr, smrdr0;
	int ret, unit, sslkp = 1;

	while (len > 0) {
		if (len >= 4) {
			unit = 4;
			smenr = SMENR_SPIDE(SPIDE_32BITS) |
						SMENR_SPIDB(sbsc->bitw);
		} else {
			unit = len;
			if (unit == 3)
				unit = 2;

			if (unit >= 2)
				smenr = SMENR_SPIDE(SPIDE_16BITS) |
						SMENR_SPIDB(sbsc->bitw);
			else
				smenr = SMENR_SPIDE(SPIDE_8BITS) |
						SMENR_SPIDB(sbsc->bitw);
		}

		len -= unit;
		if (len <= 0)
			sslkp = 0;

		/* set params */
		spibsc_write(sbsc, SMCMR, 0);
		spibsc_write(sbsc, SMADR, 0);
		spibsc_write(sbsc, SMOPR, 0);
		spibsc_write(sbsc, SMENR, smenr);

		/* start spi transfer */
		smcr = SMCR_SPIE|SMCR_SPIRE;
		if (sbsc->bitw == BITW_1BIT)
			smcr |= SMCR_SPIWE;
		if (sslkp)
			smcr |= SMCR_SSLKP;
		spibsc_write(sbsc, SMCR, smcr);

		/* wait for spi transfer completed */
		ret = spibsc_wait_trans_completion(sbsc);
		if (ret)
			return ret;	/* return error */

		/* read SMRDR */
		if (unit == 2)
			smrdr0 = (u32)spibsc_read16(sbsc, SMRDR0);
		else if (unit == 1)
			smrdr0 = (u32)spibsc_read8(sbsc, SMRDR0);
		else
		smrdr0 = spibsc_read(sbsc, SMRDR0);

		*data++ = (u8)(smrdr0 & 0xff);
		if (unit >= 2)
			*data++ = (u8)((smrdr0 >> 8) & 0xff);
		if (unit >= 3)
			*data++ = (u8)((smrdr0 >> 16) & 0xff);
		if (unit >= 4)
			*data++ = (u8)((smrdr0 >> 24) & 0xff);
	}
	return 0;
}

static int spibsc_send_cmd(struct spibsc_priv *sbsc, struct spi_transfer *t)
{
	const u8 *command;
	int len, ret;

	/* wait for spi transfer completed */
	ret = spibsc_wait_trans_completion(sbsc);
	if (ret)
		return	ret;	/* return error */

	command = t->tx_buf;
	len = t->len;

	DEBUG_COMMAND();

	{
#define CMD_LENGTH (10)
		u8 cmd[CMD_LENGTH];
		int loop;

		if (len > CMD_LENGTH) {
			pr_err("%s: command length error\n", __func__);
			return -EIO;
		}

		cmd[0] = (u8)command[0];
		switch (command[0]) {
		case CMD_FAST_READ:
			/* cmd[0] = CMD_DOR; */	/* change Dual Mode */
			cmd[0] = CMD_QOR;	/* change Quad Mode */
			break;
		#if 0	/* comment out because not tested */
		case CMD_PP:
			cmd[0] = CMD_QPP;	/* change Quad Mode */
			break;	/* nothing to do */
		#endif
		case CMD_READ:
		default:
			break;	/* nothing to do */
		}
		/* command copy */
		for (loop = 1; loop < len; loop++)
			cmd[loop] = (u8)command[loop];

		spibsc_set_busio(sbsc, cmd[0]);
		return spibsc_do_send_cmd(sbsc, cmd, len);

	}

}

static int spibsc_send_data(struct spibsc_priv *sbsc, struct spi_transfer *t)
{
	const u8 *data;
	int len, ret;

	/* wait for spi transfer completed */
	ret = spibsc_wait_trans_completion(sbsc);
	if (ret)
		return	ret;	/* return error */

	data = t->tx_buf;
	len = t->len;

	DEBUG_SEND();

	return spibsc_do_send_data(sbsc, data, len);
}

static int spibsc_receive_data(struct spibsc_priv *sbsc, struct spi_transfer *t)
{
	u8 *data;
	int len, ret;

	/* wait for spi transfer completed */
	ret = spibsc_wait_trans_completion(sbsc);
	if (ret)
		return	ret;	/* return error */

	data = t->rx_buf;
	len = t->len;

	ret = spibsc_do_receive_data(sbsc, data, len);

	DEBUG_RECEIVE();

	return ret;
}

static void spibsc_hw_cs_disable(struct spibsc_priv *sbsc)
{
	u8 cmd = CMD_RDSR, data;

	/* send Read status command to negate SSL line */
	spibsc_do_send_cmd(sbsc, &cmd, 1);
	spibsc_do_receive_data(sbsc, &data, 1);
}

/*
 *		spi master function
 */
static int spibsc_prepare_transfer(struct spi_master *master)
{
	struct spibsc_priv *sbsc = spi_master_get_devdata(master);

	pm_runtime_get_sync(sbsc->dev);
	return 0;
}

static int spibsc_unprepare_transfer(struct spi_master *master)
{
	struct spibsc_priv *spibsc = spi_master_get_devdata(master);

	pm_runtime_put_sync(spibsc->dev);
	return 0;
}

static int spibsc_transfer_one_message(struct spi_master *master,
				     struct spi_message *msg)
{
	struct spibsc_priv *sbsc = spi_master_get_devdata(master);
	struct spi_transfer *t;
	int ret;
	unsigned int cs_change;

	dev_dbg(sbsc->dev, "%s\n", __func__);

	/* start transfer each of struct spi_transfer */
	cs_change = 1;
	ret = 0;
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		/* send command or send data or recevie data */
		if (cs_change) {
			ret = spibsc_send_cmd(sbsc, t);	/* send command */
			if (ret)
				break;

		} else {
			if (t->tx_buf) {	/* send data */
				ret = spibsc_send_data(sbsc, t);
				if (ret)
					break;
			} else if (t->rx_buf) { /* receive data */
				ret = spibsc_receive_data(sbsc, t);
				if (ret)
					break;
			}
		}
		cs_change = t->cs_change;
		msg->actual_length += t->len;
	}

	if (!cs_change)
		spibsc_hw_cs_disable(sbsc);

	msg->status = ret;
	spi_finalize_current_message(master);

	return ret;
}

static int spibsc_send_then_receive(
	struct spibsc_priv *sbsc,
	const u8 *command,
	int clen,
	u8 *data,
	int len)
{
	int ret;

	do {
		ret = spibsc_wait_trans_completion(sbsc);
		if (ret)
			break;

		spibsc_set_busio(sbsc, command[0]);

		ret = spibsc_do_send_cmd(sbsc, command, clen);
		if (ret)
			break;

		if (len > 0) {
			ret = spibsc_do_receive_data(sbsc, data, len);
			if (ret)
				break;
		}
		return 0;
	} while (0);

	dev_err(sbsc->dev, "error %d for command %02x\n", ret, command[0]);
	return ret;	/* return error */
}

static int wait_for_device_ready(struct spibsc_priv *sbsc)
{
	u8 cmd[1], sr1;
	int timeout;
#define	TIMEOUT		50000
	int err;

	timeout = 0;
	do {
		cmd[0] = CMD_RDSR;
		err = spibsc_send_then_receive(sbsc, cmd, 1, &sr1, 1);
		if (err < 0) {
			dev_err(sbsc->dev, "error %d reading SR\n", err);
				return -EIO;
		}
		if (sr1 & (SR1_P_ERR | SR1_E_ERR)) {	/* Error Occurred */
			/* clear P_ERR/E_ERR(SR1:bit6/5) */
			cmd[0] = CMD_CLSR;
			(void) spibsc_send_then_receive(sbsc, cmd, 1, NULL, 0);
			continue;
		}
		if (sr1 & SR1_WIP) {	/* device busy */
			udelay(10);
			continue;
		}
		if (sr1 & SR1_WEL) {	/* Device accepts Write Registers */
			cmd[0] = CMD_WRDI;	/* clear WEL(SR1:bit1)*/
			(void) spibsc_send_then_receive(sbsc, cmd, 1, NULL, 0);
			continue;
		}
		/* Only SRWD and BP0-BP2 bits are valid here */
		/* P_ERR, E_ERR, WEL and WIP bits are cleared here */
		break;
	} while (timeout++ < TIMEOUT);

	if (timeout >= TIMEOUT)
		pr_err("%s: timedout (last sr1=%02x)\n", __func__, sr1);

	return 0;
}

static int set_quad_mode(struct spibsc_priv *sbsc)
{
	u8 cmd[3];
	u8 sr1, cr1;

	wait_for_device_ready(sbsc);		/* RZLSP */

	cmd[0] = CMD_MBR;					/* Mode Bit Reset */
	(void) spibsc_send_then_receive(sbsc, cmd, 1, NULL, 0);
	cmd[0] = CMD_RESET;					/* Software Reset */
	(void) spibsc_send_then_receive(sbsc, cmd, 1, NULL, 0);
	cmd[0] = CMD_RDSR;
	(void) spibsc_send_then_receive(sbsc, cmd, 1, &sr1, 1);
	cmd[0] = CMD_RDCR;
	(void) spibsc_send_then_receive(sbsc, cmd, 1, &cr1, 1);

	if ((cr1 & (CR1_QUAD | CR1_BPNV)) != CR1_QUAD) {

		/* operation for CR1:BPNV(OTP bit)=1  */

		cmd[0] = CMD_WREN;	/* set WEL bit(SR1:bit1) to 1 */
		(void) spibsc_send_then_receive(sbsc, cmd, 1, NULL, 0);

		cmd[0] = CMD_RDSR;
		(void) spibsc_send_then_receive(sbsc, cmd, 1, &sr1, 1);

		if (!(sr1 & SR1_WEL))
			pr_err("%s: Can't set SR1.WEL=1\n", __func__);

		/* to no protection */
		cmd[0] = CMD_WRR;
		cmd[1] = sr1  & ~(SR1_SRWD | SR1_BP_MASK);
		cmd[2] = (cr1 & ~(CR1_LC_MASK | CR1_FREEZE)) | CR1_QUAD;
		(void) spibsc_send_then_receive(sbsc, cmd, 3, NULL, 0);

		wait_for_device_ready(sbsc);
	}

	cmd[0] = CMD_RDSR;
	(void) spibsc_send_then_receive(sbsc, cmd, 1, &sr1, 1);
	cmd[0] = CMD_RDCR;
	(void) spibsc_send_then_receive(sbsc, cmd, 1, &cr1, 1);

	/* otp bit check */
	if (cr1 & 0x28)
		pr_err("Info : OTP bit = 1 (CR1 = 0x%02x, SR1 = 0x%02x)\n", cr1, sr1);

	/* Quad mode check */
	if ((cr1 & CR1_QUAD) != CR1_QUAD) {
		pr_err("%s: Can't set CR1.QUAD=1\n", __func__);
		return -1;
	}

	return 0;
}

static int spibsc_setup(struct spi_device *spi)
{
	struct spibsc_priv *sbsc = spi_master_get_devdata(spi->master);
	struct device *dev = sbsc->dev;

	if (8 != spi->bits_per_word) {
		dev_err(dev, "bits_per_word should be 8\n");
		return -EIO;
	}

	/* initilaize spibsc */
	spibsc_write(sbsc, CMNCR, CMNCR_INIT);
	spibsc_write(sbsc, DRCR, DRCR_RCF);
	spibsc_write(sbsc, SSLDR, SSLDR_INIT);
	spibsc_write(sbsc, SPBCR, SPBCR_INIT);

	dev_dbg(dev, "%s setup\n", spi->modalias);

	if (0 != set_quad_mode(sbsc)) {
		pr_err("%s: set_quad_mode error\n", __func__);
		return -EIO;
	}

	return 0;
}

static void spibsc_cleanup(struct spi_device *spi)
{
	struct spibsc_priv *sbsc = spi_master_get_devdata(spi->master);
	struct device *dev = sbsc->dev;

	dev_dbg(dev, "%s cleanup\n", spi->modalias);
}

static int spibsc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct spi_master *master;
	struct spibsc_priv *sbsc;
	struct clk *clk;
	int ret;
	char clk_name[16];

	/* get base addr */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "invalid resource\n");
		return -EINVAL;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(*sbsc));
	if (!master) {
		dev_err(&pdev->dev, "spi_alloc_master error.\n");
		return -ENOMEM;
	}

	snprintf(clk_name, sizeof(clk_name), "spibsc%d", pdev->id);
	clk = clk_get(&pdev->dev, clk_name);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "cannot get spibsc clock\n");
		ret = -EINVAL;
		goto error0;
	}
	clk_enable(clk);

	sbsc = spi_master_get_devdata(master);
	dev_set_drvdata(&pdev->dev, sbsc);
	sbsc->info = pdev->dev.platform_data;

	/* init sbsc */
	sbsc->master	= master;
	sbsc->dev	= &pdev->dev;
	sbsc->clk	= clk;
	sbsc->addr	= devm_ioremap(sbsc->dev,
				       res->start, resource_size(res));
	if (!sbsc->addr) {
		dev_err(&pdev->dev, "ioremap error.\n");
		ret = -ENOMEM;
		goto error1;
	}

	master->num_chipselect	= 1;
	master->bus_num		= sbsc->info->bus_num;
	master->setup		= spibsc_setup;
	master->cleanup		= spibsc_cleanup;
	master->mode_bits	= SPI_CPOL | SPI_CPHA;
	master->prepare_transfer_hardware	= spibsc_prepare_transfer;
	master->transfer_one_message		= spibsc_transfer_one_message;
	master->unprepare_transfer_hardware	= spibsc_unprepare_transfer;
	ret = spi_register_master(master);
	if (ret < 0) {
		dev_err(&pdev->dev, "spi_register_master error.\n");
		goto error1;
	}

	pm_runtime_enable(&pdev->dev);

	dev_info(&pdev->dev, "probed\n");

	return 0;

 error1:
	clk_put(clk);
 error0:
	spi_master_put(master);

	return ret;
}

static int spibsc_remove(struct platform_device *pdev)
{
	struct spibsc_priv *sbsc = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(&pdev->dev);

	clk_put(sbsc->clk);
	spi_unregister_master(sbsc->master);

	return 0;
}

static struct platform_driver spibsc_driver = {
	.probe = spibsc_probe,
	.remove = spibsc_remove,
	.driver = {
		.name = "spibsc",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(spibsc_driver);

MODULE_DESCRIPTION("SuperH SPIBSC bus driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("RSO");
MODULE_ALIAS("platform:sh_spi");
