/*
 * sound/soc/sh/scux_dai.c
 *     This file is ALSA SoC driver for SCUX peripheral.
 *
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Copyright (C) 2013 Renesas Electronics Corporation
 *
 * This file is based on the sound/soc/sh/siu_dai.c
 *
 * siu_dai.c - ALSA SoC driver for Renesas SH7343, SH7722 SIU peripheral.
 *
 * Copyright (C) 2009-2010 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * Copyright (C) 2006 Carlos Munoz <carlos@kenati.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/clk.h>

#include <sound/control.h>
#include <sound/soc.h>
#include <sound/sh_scux.h>

#undef DEBUG
#ifdef DEBUG
#define FNC_ENTRY	pr_info("entry:%s:%d\n", __func__, __LINE__);
#define FNC_EXIT	pr_info("exit:%s:%d\n", __func__, __LINE__);
#define DBG_POINT()	pr_info("check:%s:%d\n", __func__, __LINE__);
#define DBG_MSG(args...)	pr_info(args)
#else  /* DEBUG */
#define FNC_ENTRY
#define FNC_EXIT
#define DBG_POINT()
#define DBG_MSG(args...)
#endif /* DEBUG */

static struct scux_reg_info *scux_reg;
static spinlock_t *sculock;
static struct scu_audio_info *ainfo;
static struct scu_platform_data *pdata;

int32_t getvolume0;
int32_t getvolume1;

struct scu_route_info *scu_get_route_info(void)
{
	return &ainfo->routeinfo;
}
EXPORT_SYMBOL(scu_get_route_info);

struct scu_platform_data *scu_get_platform_data(void)
{
	return pdata;
}
EXPORT_SYMBOL(scu_get_platform_data);

/************************************************************************
	peripheral function
************************************************************************/
static void scu_ssif_softreset(int ch)
{
	u8 reg;

	switch (ch) {
	case 0:
		writeb(SWRSTCR1_SRST16, (u32 *)SWRSTCR1); /* Soft Reset SSIF0 */
		break;
	case 1:
		writeb(SWRSTCR1_SRST15, (u32 *)SWRSTCR1); /* Soft Reset SSIF1 */
		break;
	case 2:
		writeb(SWRSTCR1_SRST14, (u32 *)SWRSTCR1); /* Soft Reset SSIF2 */
		break;
	case 3:
		writeb(SWRSTCR1_SRST13, (u32 *)SWRSTCR1); /* Soft Reset SSIF3 */
		break;
	case 4:
		writeb(SWRSTCR1_SRST12, (u32 *)SWRSTCR1); /* Soft Reset SSIF4 */
		break;
	case 5:
		writeb(SWRSTCR1_SRST11, (u32 *)SWRSTCR1); /* Soft Reset SSIF5 */
		break;
	}
	udelay(10);
	writeb(SWRSTCR1_INIT, (u32 *)SWRSTCR1);
	udelay(10);
	reg = readb((u32 *)SWRSTCR1);	/* dummy read */
}

static void scu_ssif_init(void)
{
	FNC_ENTRY

	/* SSI TDM Mode Register Setting */
	writel(SSI_SSITDMR_BASE_INIT_VALUE, &scux_reg->ssifreg[0]->ssitdmr);
	/* SSI Control register setting */
	writel(SSI_SSICR_BASE_INIT_VALUE, &scux_reg->ssifreg[0]->ssicr);
	/* SSI FIFO Control register setting */
	writel(SSI_SSIFCR_BASE_INIT_VALUE, &scux_reg->ssifreg[0]->ssifcr);
	/* SSI Status register clear */
	writel(0, &scux_reg->ssifreg[0]->ssisr);

	FNC_EXIT
	return;
}

static void scu_ssi_control(int master_ch, int slave_ch, int mode)
{
	FNC_ENTRY
	/* SSI setting */
	if ((readl(&scux_reg->ssifreg[master_ch]->ssicr) & SSICR_ENABLE) == 0) {
		writel(SSICR_PLAY_WM8978_ST,
			&scux_reg->ssifreg[master_ch]->ssicr);
		writel(SSITDMR_CONT_EN, &scux_reg->ssifreg[master_ch]->ssitdmr);
	}
	if ((mode == SSI_SLAVE) &&
	    ((readl(&scux_reg->ssifreg[slave_ch]->ssicr) & SSICR_ENABLE) == 0))
		writel(SSICR_CAP_WM8978_ST,
			&scux_reg->ssifreg[slave_ch]->ssicr);

	FNC_EXIT
	return;
}

static void scu_ssi_start(int ssi_ch, int ssi_dir)
{
	u32 val;

	FNC_ENTRY

	if (ssi_dir == SSI_OUT) {
		/* SSI enable (figure.39.12 flow) */
		val = readl(&scux_reg->ssifreg[ssi_ch]->ssicr);
		val |= (SSICR_TUIEN | SSICR_TOIEN);
		val &= ~(SSICR_IIEN);
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssicr);

		val = (SSIFCR_TIE | SSIFCR_RFRST);
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssifcr);

		val = readl(&scux_reg->ssifreg[ssi_ch]->ssicr);
		val |= SSICR_TEN_EN;
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssicr);
	} else { /* ssi_dir == SSI_IN */
		/* SSI enable (figure.39.14 flow) */
		val = readl(&scux_reg->ssifreg[ssi_ch]->ssicr);
		val |= (SSICR_RUIEN | SSICR_ROIEN);
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssicr);

		val = (SSIFCR_RIE | SSIFCR_TFRST);
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssifcr);

		val = readl(&scux_reg->ssifreg[ssi_ch]->ssicr);
		val |= SSICR_REN_EN;
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssicr);
	}

	FNC_EXIT
	return;
}

static void scu_ssi_stop(int ssi_ch, int ssi_dir)
{
	u32 val;
	int tmout;

	FNC_ENTRY

	if (ssi_dir == SSI_OUT) {
		/* SSI disable (figure.39.13 flow) */
		val = readl(&scux_reg->ssifreg[ssi_ch]->ssicr);
		val &= ~(SSICR_TUIEN | SSICR_TOIEN | SSICR_TEN_EN);
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssicr);

		val = SSI_SSIFCR_BASE_INIT_VALUE;
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssifcr);

		tmout = 1000;
		while (--tmout &&
		       !(readl(&scux_reg->ssifreg[ssi_ch]->ssisr) & SSISR_IIRQ))
			udelay(1);
		if (!tmout)
			tmout = 1000;
		while (--tmout &&
		       !(readl(&scux_reg->ssifreg[ssi_ch]->ssisr) & SSISR_IDST))
			udelay(1);
		if (!tmout)
			pr_info("timeout waiting for SSI idle\n");

	} else { /* ssi_dir == SSI_IN */
		/* SSI disable (figure.39.15 flow) */
		val = readl(&scux_reg->ssifreg[ssi_ch]->ssicr);
		val &= ~(SSICR_RUIEN | SSICR_ROIEN | SSICR_REN_EN);
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssicr);

		val = SSI_SSIFCR_BASE_INIT_VALUE;
		writel(val, &scux_reg->ssifreg[ssi_ch]->ssifcr);

		tmout = 1000;
		while (--tmout &&
		       !(readl(&scux_reg->ssifreg[ssi_ch]->ssisr) & SSISR_IDST))
			udelay(1);
		if (!tmout)
			pr_info("timeout waiting for SSI idle\n");
	}

	scu_ssif_softreset(ssi_ch);

	FNC_EXIT
}

static void scu_src_control(int src_ch, unsigned int rate, unsigned int sync_sw)
{
	u32 reg, futsel;

	FNC_ENTRY

	/* SRC0 is capture,SRC1 is playback */
	/* Return to prohibit the enable register of sending and receiving */
	reg = readl(&scux_reg->ssifreg[0]->ssicr);
	reg &= ~(SSICR_TEN_EN | SSICR_REN_EN);
	writel(reg, &scux_reg->ssifreg[0]->ssicr);

	/* 1.SCUX RESET:when DVU is already reset from scu_init_dvc() */
	if (ainfo->routeinfo.p_route != RP_MEM_SRC1_DVC1_SSI0) {
		writel(SWRSR_SWRST_RST, &scux_reg->cimreg->swrsr);
		udelay(10);
		writel(SWRSR_SWRST_OPE, &scux_reg->cimreg->swrsr);
		udelay(10);
	}

	/* Capture route */
	if (src_ch == SRC0) {
		/* 2.Setting the transfer route */
		/* FFD and FFU route Setting */
		writel(FFDPR_NO_PASS_SELECT, &scux_reg->ffdreg[src_ch]->ffdpr);
		writel(FFUPR_CIM_FFU_OPC_ASYNC,
			&scux_reg->ffureg[src_ch]->ffupr);

		/* IPC and OPC route Setting */
		writel(IPSLR_SSIF_IPC_ASYNC, &scux_reg->ipcreg[src_ch]->ipslr);
		writel(OPSLR_ASYNC_OPC_FFU, &scux_reg->opcreg[src_ch]->opslr);

		/* SSIF route choice */
		reg = readl(&scux_reg->cimreg->ssirsel_cim);
		reg &= SSIRSEL_SISEL0_USE_SSIF0;
		writel(reg, &scux_reg->cimreg->ssirsel_cim);

		/* SRCRSELn_CIM (n=0,1,2,3), MIXRSEL_CIM */
		/* NO SETTING */

		/* SSICTRL_CIM is Direct send setting */
		reg = readl((u32 *)&scux_reg->cimreg->ssictrl_cim);
		reg |= SSICTRL_SSI0RX_EN;
		writel(reg, &scux_reg->cimreg->ssictrl_cim);

		/* 3.Setting of timing */
		/* FUTSELn_CIM SCKDIV setting ,AUDIO_X1(22579200Hz) / Fout */
		reg = AUDIO_X1 / rate;
		if (reg > SCKDIV_MAX)
			reg = SCKDIV_MAX;

		/* FFU0_0 : setting clock */
		reg = (reg << 16);
		reg |= FUTSEL_SCKSEL_AUIDIO_X1;
		writel(reg, &scux_reg->cimreg->futsel0_cim);
		futsel = reg;

		/* FFD0_1 */
		/* NO Setting */

		/* 4.SSIF setting pin */
		/* SSIPMD_CIM SSIF pinmode setting */
		writel(SSIPMD_SSI0CKS, &scux_reg->cimreg->ssipmd_cim);

		/* 5.FFD0_1 */
		/* NO SETTING */

		/* 6.FFU0_1 */
		writel(FUAIR_AUDIO_CH_2, &scux_reg->ffureg[src_ch]->fuair);
		writel(URQSR_CH0CH1_REQ_SIZE_256,
			&scux_reg->ffureg[src_ch]->urqsr);
		writel(UEVMR_VALUE, &scux_reg->ffureg[src_ch]->uevmr);

		/* 7.SRCn */
		writel(SRCBR_BYPASS_OFF, &scux_reg->srcreg[0]->srcbr0);
		writel((SADIR_OTBL_16BIT | SADIR_AUDIO_CH_2),
			&scux_reg->srcreg[0]->sadir0);

		/* IFSCRp_2SRC0_m is used setting INTIFS */
		writel(IFSCR_VALUE, &scux_reg->srcreg[0]->ifscr0);

		/* IFSVRp_2SRC0_m is setting INTIFS */
		switch (rate) {
		case SRC_IFS_8KHZ:
			reg = INTIFS_IN44K_OUT8K;
			break;
		case SRC_IFS_11KHZ:
			reg = INTIFS_IN44K_OUT11K;
			break;
		case SRC_IFS_12KHZ:
			reg = INTIFS_IN44K_OUT12K;
			break;
		case SRC_IFS_16KHZ:
			reg = INTIFS_IN44K_OUT16K;
			break;
		case SRC_IFS_22KHZ:
			reg = INTIFS_IN44K_OUT22K;
			break;
		case SRC_IFS_24KHZ:
			reg = INTIFS_IN44K_OUT24K;
			break;
		case SRC_IFS_32KHZ:
			reg = INTIFS_IN44K_OUT32K;
			break;
		default:
		case SRC_IFS_44KHZ:
			reg = INTIFS_IN44K_OUT44K;
			break;
		case SRC_IFS_48KHZ:
			reg = INTIFS_IN44K_OUT48K;
			break;
		}
		writel(reg, &scux_reg->srcreg[0]->ifsvr0);
		writel((SRC_CR_BIT16 | SRC_CR_BIT8 | SRC_CR_BIT4),
			&scux_reg->srcreg[0]->srccr0);
		/* MNFSRp_2SRC0_m is setting minimum of FS */
		/* MINFS = INITFS * 90% */
		reg = reg * MIN_FS_RATIO_90 / MIN_FS_RATIO_100;
		writel(reg, &scux_reg->srcreg[0]->mnfsr0);

		/* BFSSRp_2SRC0_m NO Setting*/
		/* WATSRp_2SRC0_m NO Setting */
		/* SEVMRp_2SRC0_m NO Setting when transfer start*/

		/* 8.DVUn */
		/* DVU BYPASS setting */
		/* NO SETTING */

		/* 9.MIX */
		/* MIX BYPASS setting */
		/* NO SETTING */

		/* DMA transfer setting */
		reg = readl(&scux_reg->cimreg->dmacr_cim);
		reg |= DMACR_DMAMDFFU0_EN;
		writel(reg, &scux_reg->cimreg->dmacr_cim);

		/* FFD Init off */
		writel(FFDIR_INIT_OFF, &scux_reg->ffdreg[src_ch]->ffdir);

		/* FFD BOOT */
		writel(FFDBR_HALT_ON, &scux_reg->ffdreg[src_ch]->ffdbr);

		/* FFU,SRC,IPC,OPC Init off */
		writel(INIT_OFF, &scux_reg->ffureg[src_ch]->ffuir);
		writel(INIT_OFF, &scux_reg->srcreg[0]->srcir0);
		writel(INIT_OFF, &scux_reg->srcreg[0]->srcirr1);
		writel(INIT_OFF, &scux_reg->ipcreg[src_ch]->ipcir);
		writel(INIT_OFF, &scux_reg->opcreg[src_ch]->opcir);

		/* Send start setting */
		reg = readl(&scux_reg->ssifreg[0]->ssicr);
		reg |= SSICR_REN_EN;
		writel(reg, &scux_reg->ssifreg[0]->ssicr);
		futsel |= FUTSEL_DIVEN_START;
		writel(futsel, &scux_reg->cimreg->futsel0_cim);

	} else if (src_ch == SRC1) {

		reg = readl(&scux_reg->cimreg->ssirsel_cim);
		reg |= SSIRSEL_SOSEL0_USE_SRC1;
		/* 0xE8209738:SSIRSEL_CIM  SOSEL0 = 01b */
		writel(reg, &scux_reg->cimreg->ssirsel_cim);
		/* 0xE8208104:IPSLR_IPC0_n  IPC_PASS_SEL = 011b */
		writel(IPSLR_FFD_IPC_ASYNC, &scux_reg->ipcreg[src_ch]->ipslr);
		/* 0xE8208504:OPSLR_OPC0_n  OPC_PASS_SEL = 001b	 */
		writel(OPSLR_ASYNC_OPC_DVU, &scux_reg->opcreg[src_ch]->opslr);
		/* 0xE820890C:FFDPR_FFD0_n  PASS = 01b */
		writel(FFDPR_CIM_FFD_IPC_ASYNC,
			&scux_reg->ffdreg[src_ch]->ffdpr);
		/* 0xE8208D0C:FFUPR_FFU0_n  PASS = 00b */
		writel(FFUPR_NO_PASS_SELECT, &scux_reg->ffureg[src_ch]->ffupr);
		/* 0xE820975C:SSIPMD_CIM    SSI345EN = 0, SSI012EN = 0 */
		writel(THIS_CH_NOT_USED, &scux_reg->cimreg->ssipmd_cim);

		/* SSICTRL_CIM is Direct send setting */
		reg = readl((u32 *)&scux_reg->cimreg->ssictrl_cim);
		/* SSICTRL_CIM  SSI3RX = 0, SSI3TX = 0	: value = 0xBBFFFFFF */
		reg &= ~(SSICTRL_SSI3TX_EN | SSICTRL_SSI3RX_EN);
		/* SSICTRL_CIM  SSI0TX = 1		: value = 0x00004000 */
		reg |= SSICTRL_SSI0TX_EN;
		/* 0xE8209760 : SSI0TX = 1 */
		writel(reg, &scux_reg->cimreg->ssictrl_cim);

		/* FDTSELn_CIM SCKDIV setting */
		/* FDTSELn_CIM    SCKDIV = AUDIO_X1(22579200Hz) / Fin  */
		/* FDTSELn_CIM SCKSEL = 0001b(AUDIO_X1) */
		/* MNFSRp_2SRC0_m MINFS  = (2^22 * Fin / Fout) * 90% */
		/* IFSVRp_2SRC0_m INTIFS = 2^22 * Fin / Fout  */
		/* Fin = Input Rate, Fout = Output Rate(44100Hz) */
		reg = AUDIO_X1 / rate;			/* FDTSELn_CIM SCKDIV */
		if (reg > SCKDIV_MAX)
			reg = SCKDIV_MAX;
		reg = (reg << 16);
		reg |= FDTSEL_SCKSEL_AUIDIO_X1;
		writel(reg, &scux_reg->cimreg->fdtsel1_cim);

		/* IFSVRp_2SRC0_m  INTIFS */
		switch (rate) {
		case SRC_IFS_8KHZ:
			reg = INTIFS_IN8K_OUT44K;
			break;
		case SRC_IFS_11KHZ:
			reg = INTIFS_IN11K_OUT44K;
			break;
		case SRC_IFS_12KHZ:
			reg = INTIFS_IN12K_OUT44K;
			break;
		case SRC_IFS_16KHZ:
			reg = INTIFS_IN16K_OUT44K;
			break;
		case SRC_IFS_22KHZ:
			reg = INTIFS_IN22K_OUT44K;
			break;
		case SRC_IFS_24KHZ:
			reg = INTIFS_IN24K_OUT44K;
			break;
		case SRC_IFS_32KHZ:
			reg = INTIFS_IN32K_OUT44K;
			break;
		default:
		case SRC_IFS_44KHZ:
			reg = INTIFS_IN44K_OUT44K;
			break;
		case SRC_IFS_48KHZ:
			reg = INTIFS_IN48K_OUT44K;
			break;
		}
		writel(reg, &scux_reg->srcreg[0]->ifsvr1);
		/* MNFSRp_2SRC0_m is setting minimum of FS */
		/* MINFS = INITFS * 90% */
		reg = reg * MIN_FS_RATIO_90 / MIN_FS_RATIO_100;
		writel(reg, &scux_reg->srcreg[0]->mnfsr1);
		writel(IFSCR_VALUE, &scux_reg->srcreg[0]->ifscr1);
		/* FUTSELn_CIM NO Setting  */
		/* FFD Setting */
		/* 0xE8208904:FDAIR_FFD0_n CHNUM = 0010b(2 channel) */
		writel(FDAIR_AUDIO_CH_2, &scux_reg->ffdreg[src_ch]->fdair);
		/* 0xE8208908:DRQSR_FFD0_n SIZE = 0001b(256 data) */
		writel(DRQSR_REQ_SIZE_256, &scux_reg->ffdreg[src_ch]->drqsr);
		/* 0xE8208914:DEVMR_FFD0_n DEVMUF = 1, DEVMOF = 1, DEVMOL = 1,
			      DEVMIUF = 1, DEVMRQ = 0 */
		writel(DEVMR_VALUE, &scux_reg->ffdreg[src_ch]->devmr);

		/* SRC Setting */
		/* 0xE820903C:SRCBRp_2SRC0_m  BYPASS = 0 */
		writel(SRCBR_BYPASS_OFF, &scux_reg->srcreg[0]->srcbr1);
		/* 0xE8209038:SADIRp_2SRC0_m  OTBL = 01000(16 bit),
					      CHNUM = 0010(2 channel) */
		writel((SADIR_OTBL_16BIT | SADIR_AUDIO_CH_2),
			&scux_reg->srcreg[0]->sadir1);
		/* 0xE8209048:SRCCRp_2SRC0_m */
		writel((SRC_CR_BIT16 | SRC_CR_BIT8 | SRC_CR_BIT4),
			&scux_reg->srcreg[0]->srccr1);
		/* BFSSRp_2SRC0_m NO Setting*/
		/* WATSRp_2SRC0_m NO Setting */
		/* SEVMRp_2SRC0_m NO Setting */

		/* DVU Setting */
		/* 0xE8209304:VADIR_DVU0_n  OTBL = 01000b(16 bit),
					    CHNUM = 0010b(2 channel) */
		writel((VADIR_OTBL_16BIT | VADIR_CHNUM_CH_2),
			&scux_reg->dvureg[1]->vadir);
		/* DVU BYPASS setting */
		/* 0xE8209308:DVUBR_DVU0_n  BYPASS = 1 */
		if (ainfo->routeinfo.p_route != RP_MEM_SRC1_DVC1_SSI0)
			writel(DVUBR_BYPASS_ON, &scux_reg->dvureg[1]->dvubr);

		/* MIX BYPASS setting */
		/* 0xE8209608:MIXBR_MIX0_0  BPSYS 01b(Input Data of system B),
			      BYPASS = 1 */
		writel(MIXBR_BYPASS_ON, &scux_reg->mixreg->mdber);

		/* DMA transfer setting */
		reg = readl(&scux_reg->cimreg->dmacr_cim);
		/* 0xE8209704:DMACR_CIM  DMAMDFFD1 = 1 */
		reg |= DMACR_DMAMDFFD1_EN;
		writel(reg, &scux_reg->cimreg->dmacr_cim);

		/* FFD Init off */
		/* 0xE8208900:FFDIR_FFD0_n  INIT = 0 */
		writel(FFDIR_INIT_OFF, &scux_reg->ffdreg[src_ch]->ffdir);

		/* FFD BOOT */
		/* 0xE8208910:FFDBR_FFD0_n  BOOT = 1 */
		writel(FFDBR_BOOT_ON, &scux_reg->ffdreg[src_ch]->ffdbr);

		/* SRC,DVU,MIX,IPC,OPC Init off */
		/* 0xE8208D00:FFUIR_FFU0_n  INIT = 0 */
		writel(INIT_OFF, &scux_reg->ffureg[src_ch]->ffuir);
		/* 0xE8209034:SRCIRp_2SRC0_m  INIT = 0 */
		writel(INIT_OFF, &scux_reg->srcreg[0]->srcir1);
		/* 0xE8209068:SRCIRR_2SRC0_m  INIT = 0 */
		writel(INIT_OFF, &scux_reg->srcreg[0]->srcirr1);
		/* 0xE8209300:DVUIR_DVU0_n  INIT = 0 */
		writel(INIT_OFF, &scux_reg->dvureg[1]->dvuir);
		/* 0xE8209600:MIXIR_MIX0_0  INIT = 0 */
		writel(INIT_OFF, &scux_reg->mixreg->mixir);
		/* 0xE8208100:IPCIR_IPC0_n  INIT = 0 */
		writel(INIT_OFF, &scux_reg->ipcreg[src_ch]->ipcir);
		/* 0xE8208500:OPCIR_OPC0_n  INIT = 0 */
		writel(INIT_OFF, &scux_reg->opcreg[src_ch]->opcir);
		/* Send start setting */
		reg = readl(&scux_reg->cimreg->fdtsel1_cim);
		reg |= SCUX_FDTSEL_BIT_DIVEN;
		/* 0xE8209740:FDTSELn_CIM  DIVEN = 1 */
		writel(reg, &scux_reg->cimreg->fdtsel1_cim);

		reg = readl(&scux_reg->ssifreg[0]->ssicr);
		reg |= SSICR_TEN_EN;
		writel(reg, &scux_reg->ssifreg[0]->ssicr);

	} else
		pr_info("error SRC CH:%d\n", src_ch);
	FNC_EXIT
	return;
}

static void scu_dvc_deinit(int dvc_ch)
{
	FNC_ENTRY

	/* DVUIR : DVU init on */
	/* 0xE8209300:DVUIR_INIT_ON  */
	writel(DVUIR_INIT_ON, &scux_reg->dvureg[1]->dvuir);

	FNC_EXIT
	return;
}

void scu_dvc_control(int dvc_ch)
{
	FNC_ENTRY

	/*  DVU_ADINR */
	/* only stereo now */
	/* DVU0_1 VADIR :0xE8209304  */
	writel((VADIR_OTBL_16BIT | VADIR_CHNUM_CH_2),
		(u32 *)&scux_reg->dvureg[dvc_ch]->vadir);

	/*  DVU_DVUCR  DVU mode select */
	/* DVU0_1 DVUCR :0xE820930C  */
	writel((DVUCR_VVMD_USE | DVUCR_VRMD_USE),
		(u32 *)&scux_reg->dvureg[dvc_ch]->dvucr);
	/*  DVU_VRCTR volume ramp action setting */
	/* DVU0_1 VRCTR : 0xE8209314 */
	writel((VRCTR_VREN0_EN | VRCTR_VREN1_EN),
		(u32 *)&scux_reg->dvureg[dvc_ch]->vrctr);

	/* VRPDR volume ramp time setting */
	/* DVU0_1 VRPDR : 0xE8209318 */	/* SCUX_VRPDR_BIT_VRPDUP */
	writel((VRPDR_VRPDUP_PERIOD_1 | VRPDR_VRPDDW_PERIOD_1),
		(u32 *)&scux_reg->dvureg[dvc_ch]->vrpdr);

	/* volume ramp gain level setting */
	/* DVU0_1 VRDBR : 0xE820931C */
	writel(VRDBR_VRDB_0DB, (u32 *)&scux_reg->dvureg[dvc_ch]->vrdbr);

	/* Digital Volume Function Parameter */
	/* DVU0_1 VOL0R : 0xE8209324 */
	writel(getvolume0, (u32 *)&scux_reg->dvureg[dvc_ch]->vol0r);
	/* DVU0_1 VOL1R : 0xE8209328 */
	writel(getvolume1, (u32 *)&scux_reg->dvureg[dvc_ch]->vol1r);

	/* Zero Cross Mute Function */
	/* DVU0_1 ZCMCR : 0xE8209310 */
	writel(0, (u32 *)&scux_reg->dvureg[dvc_ch]->zcmcr);

	/* VRWTR volume ramp wait time setting */
	/* DVU0_1 VRWTR : 0xE8209320 */
	writel(0, (u32 *)&scux_reg->dvureg[dvc_ch]->vrwtr);

	/* vevmr SCUDVIn request intrrupt enable/disable setting */
	/* DVU0_1 VERVMR : 0xE820934C */
	writel(0, (u32 *)&scux_reg->dvureg[dvc_ch]->vevmr);

	/* DVUER */
	/* DVU0_1 DVUER : 0xE8209344 */
	writel(DVUER_DVUEN_EN, (u32 *)&scux_reg->dvureg[dvc_ch]->dvuer);

	FNC_EXIT
	return;
}

static void scu_dvc_stop(int dvc_ch)
{
	FNC_ENTRY
	/* DVU_DVUER */
	writel(0, (u32 *)&scux_reg->dvureg[dvc_ch]->dvuer);
	FNC_EXIT
	return;
}

/************************************************************************

	DAPM callback function

************************************************************************/
void scu_init_ssi(int master_ch, int slave_ch, int mode, int ind, int dir)
{
	int ch = (mode == SSI_MASTER) ? master_ch : slave_ch;

	scu_ssif_softreset(ch);

	/* SSI init */
	scu_ssi_control(master_ch, slave_ch, mode);

	/* SSI start */
	scu_ssi_start(ch, dir);
}
EXPORT_SYMBOL(scu_init_ssi);

void scu_deinit_ssi(int ch, int mode, int ind, int dir)
{
	/* SSI stop */
	scu_ssi_stop(ch, dir);

}
EXPORT_SYMBOL(scu_deinit_ssi);

void scu_init_src(int src_ch, unsigned int rate, unsigned int sync_sw)
{
	scu_src_control(src_ch, rate, sync_sw);
}
EXPORT_SYMBOL(scu_init_src);

void scu_deinit_src(int src_ch)
{
	/* SCUX reset */
	/* 0xE8209700:SWRSR_CIM SWRST = 0 */
	writel(SWRSR_SWRST_RST, &scux_reg->cimreg->swrsr);
	udelay(10);
	/* 0xE8209700:SWRSR_CIM SWRST = 1 */
	writel(SWRSR_SWRST_OPE, &scux_reg->cimreg->swrsr);
	udelay(10);
}
EXPORT_SYMBOL(scu_deinit_src);

void scu_init_dvc(int dvc_ch)
{
	/* SCUX reset */
	/* 0xE8209700:SWRSR_CIM SWRST = 0 */
	writel(SWRSR_SWRST_RST, &scux_reg->cimreg->swrsr);
	udelay(10);
	/* 0xE8209700:SWRSR_CIM SWRST = 1 */
	writel(SWRSR_SWRST_OPE, &scux_reg->cimreg->swrsr);
	udelay(10);

	scu_dvc_control(dvc_ch);
}
EXPORT_SYMBOL(scu_init_dvc);

void scu_deinit_dvc(int dvc_ch)
{
	/* stop dvu */
	scu_dvc_stop(dvc_ch);
	scu_dvc_deinit(dvc_ch);
}
EXPORT_SYMBOL(scu_deinit_dvc);

/************************************************************************

	dai ops

************************************************************************/
/* Playback and capture hardware properties are identical */
static struct snd_pcm_hardware scu_dai_pcm_hw = {
	.info			= (SNDRV_PCM_INFO_INTERLEAVED	|
				   SNDRV_PCM_INFO_MMAP		|
				   SNDRV_PCM_INFO_MMAP_VALID	|
				   SNDRV_PCM_INFO_PAUSE),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_8000_48000,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= SCU_BUFFER_BYTES_MAX,
	.period_bytes_min	= SCU_PERIOD_BYTES_MIN,
	.period_bytes_max	= SCU_PERIOD_BYTES_MAX,
	.periods_min		= SCU_PERIODS_MIN,
	.periods_max		= SCU_PERIODS_MAX,
};

static int scu_dai_info_rate(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = RATE_MAX;

	return 0;
}

static int scu_dai_get_rate(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_value *ucontrol)
{
	struct scu_audio_info *ainfo = snd_kcontrol_chip(kctrl);

	switch (kctrl->private_value) {
	case CTRL_PLAYBACK:
		ucontrol->value.integer.value[0] = ainfo->rate[0];
		break;
	case CTRL_CAPTURE:
		ucontrol->value.integer.value[0] = ainfo->rate[1];
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int scu_dai_put_rate(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_value *ucontrol)
{
	struct scu_audio_info *ainfo = snd_kcontrol_chip(kctrl);
	int change = 0;

	switch (kctrl->private_value) {
	case CTRL_PLAYBACK:
		change |= (ainfo->rate[0] != ucontrol->value.integer.value[0]);
		if (change)
			ainfo->rate[0] = ucontrol->value.integer.value[0];
		break;
	case CTRL_CAPTURE:
		change |= (ainfo->rate[1] != ucontrol->value.integer.value[0]);
		if (change)
			ainfo->rate[1] = ucontrol->value.integer.value[0];
		break;
	default:
		return -EINVAL;
	}

	return change;
}

static struct snd_kcontrol_new playback_rate_controls = {
	.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		= "PCM Playback Sampling Rate",
	.index		= 0,
	.info		= scu_dai_info_rate,
	.get		= scu_dai_get_rate,
	.put		= scu_dai_put_rate,
	.private_value	= CTRL_PLAYBACK,
};

static struct snd_kcontrol_new capture_rate_controls = {
	.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		= "PCM Capture Sampling Rate",
	.index		= 0,
	.info		= scu_dai_info_rate,
	.get		= scu_dai_get_rate,
	.put		= scu_dai_put_rate,
	.private_value	= CTRL_CAPTURE,
};

static int scu_dai_info_volume(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = VOLUME_MAX_DVC;

	return 0;
}

static int scu_dai_get_volume(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_value *ucontrol)
{
	struct scu_audio_info *ainfo = snd_kcontrol_chip(kctrl);

	switch (kctrl->private_value) {
	case CTRL_PLAYBACK:
		ucontrol->value.integer.value[0] = ainfo->volume[0][0];
		ucontrol->value.integer.value[1] = ainfo->volume[0][1];
		/* save volume */
		getvolume0 = ucontrol->value.integer.value[0];
		getvolume1 = ucontrol->value.integer.value[1];
		break;
	case CTRL_CAPTURE:
		ucontrol->value.integer.value[0] = ainfo->volume[1][0];
		ucontrol->value.integer.value[1] = ainfo->volume[1][1];
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int scu_dai_put_volume(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_value *ucontrol)
{
	struct scu_audio_info *ainfo = snd_kcontrol_chip(kctrl);
	int change = 0;

	if (ucontrol->value.integer.value[0] < 0 ||
		ucontrol->value.integer.value[0] > VOLUME_MAX_DVC ||
		ucontrol->value.integer.value[1] < 0 ||
		ucontrol->value.integer.value[1] > VOLUME_MAX_DVC)
		return -EINVAL;

	switch (kctrl->private_value) {
	case CTRL_PLAYBACK:
		change |= ((ainfo->volume[0][0] !=
			ucontrol->value.integer.value[0]) ||
			(ainfo->volume[0][1] !=
			ucontrol->value.integer.value[1]));
		if (change) {
			ainfo->volume[0][0] = ucontrol->value.integer.value[0];
			ainfo->volume[0][1] = ucontrol->value.integer.value[1];
			/* DVU0_1 L:vol0r R:vol1r */
			/* DVU0_1 VOL0R : 0xE8209324 */
			writel(ainfo->volume[0][0],
				&scux_reg->dvureg[1]->vol0r);
			/* DVU0_1 VOL1R : 0xE8209328 */
			writel(ainfo->volume[0][1],
				&scux_reg->dvureg[1]->vol1r);
		}
		break;
	case CTRL_CAPTURE:
		change |= ((ainfo->volume[1][0] !=
			ucontrol->value.integer.value[0]) ||
			(ainfo->volume[1][1] !=
			ucontrol->value.integer.value[1]));
		if (change) {
			ainfo->volume[1][0] = ucontrol->value.integer.value[0];
			ainfo->volume[1][1] = ucontrol->value.integer.value[1];
			/* DVU0_0 L:vol1r R:vol0r */
			writel(ainfo->volume[1][0],
			       (u32 *)&scux_reg->dvureg[0]->vol1r);
			writel(ainfo->volume[1][1],
			       (u32 *)&scux_reg->dvureg[0]->vol0r);
		}
		break;
	default:
		return -EINVAL;
	}

	return change;
}

static struct snd_kcontrol_new playback_volume_controls = {
	.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		= "PCM Playback Volume",
	.index		= 0,
	.info		= scu_dai_info_volume,
	.get		= scu_dai_get_volume,
	.put		= scu_dai_put_volume,
	.private_value	= CTRL_PLAYBACK,
};

static struct snd_kcontrol_new capture_volume_controls = {
	.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		= "Capture Volume",
	.index		= 0,
	.info		= scu_dai_info_volume,
	.get		= scu_dai_get_volume,
	.put		= scu_dai_put_volume,
	.private_value	= CTRL_CAPTURE,
};

static int scu_dai_info_mute(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}

static int scu_dai_get_mute(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_value *ucontrol)
{
	struct scu_audio_info *ainfo = snd_kcontrol_chip(kctrl);

	ucontrol->value.integer.value[0] = ainfo->mute[0];
	ucontrol->value.integer.value[1] = ainfo->mute[1];

	return 0;
}

static int scu_dai_put_mute(struct snd_kcontrol *kctrl,
				struct snd_ctl_elem_value *ucontrol)
{
	struct scu_audio_info *ainfo = snd_kcontrol_chip(kctrl);
	int change = 0;
	u32 mute = 0;

	if (ucontrol->value.integer.value[0] < 0 ||
	    ucontrol->value.integer.value[0] > 1 ||
	    ucontrol->value.integer.value[1] < 0 ||
	    ucontrol->value.integer.value[1] > 1)
		return -EINVAL;

	change |= ((ainfo->mute[0] != ucontrol->value.integer.value[0]) ||
			(ainfo->mute[1] != ucontrol->value.integer.value[1]));
	if (change) {
		ainfo->mute[0] = ucontrol->value.integer.value[0];
		ainfo->mute[1] = ucontrol->value.integer.value[1];
		mute = (ainfo->mute[1] << 1) + ainfo->mute[0];
		mute = ~mute & 0x3;
		writel(mute, (u32 *)&scux_reg->dvureg[1]->zcmcr);
	}

	return change;
}

static struct snd_kcontrol_new playback_mute_controls = {
	.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		= "PCM Playback Switch",
	.index		= 0,
	.info		= scu_dai_info_mute,
	.get		= scu_dai_get_mute,
	.put		= scu_dai_put_mute,
};

int scu_dai_add_control(struct snd_card *card)
{
	struct device *dev = card->dev;
	struct snd_kcontrol *kctrl;
	int i, j, ret;

	/* initial value */
	for (i = 0; i < 2; i++) {
		ainfo->rate[i] = 0;
		ainfo->mute[i] = 1;
		for (j = 0; j < 2; j++)
			ainfo->volume[i][j] = VOLUME_DEFAULT;
	}

	getvolume0 = VOLUME_DEFAULT;
	getvolume1 = VOLUME_DEFAULT;
	kctrl = snd_ctl_new1(&playback_rate_controls, ainfo);
	ret = snd_ctl_add(card, kctrl);
	if (ret < 0) {
		dev_err(dev, "failed to add playback rate err=%d\n", ret);
		return ret;
	}

	kctrl = snd_ctl_new1(&capture_rate_controls, ainfo);
	ret = snd_ctl_add(card, kctrl);
	if (ret < 0) {
		dev_err(dev, "failed to add capture rate err=%d\n", ret);
		return ret;
	}

	kctrl = snd_ctl_new1(&playback_volume_controls, ainfo);
	ret = snd_ctl_add(card, kctrl);
	if (ret < 0) {
		dev_err(dev, "failed to add playback volume err=%d\n", ret);
		return ret;
	}

	kctrl = snd_ctl_new1(&capture_volume_controls, ainfo);
	ret = snd_ctl_add(card, kctrl);
	if (ret < 0) {
		dev_err(dev, "failed to add capture volume err=%d\n", ret);
		return ret;
	}

	kctrl = snd_ctl_new1(&playback_mute_controls, ainfo);
	ret = snd_ctl_add(card, kctrl);
	if (ret < 0) {
		dev_err(dev, "failed to add playback mute err=%d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(scu_dai_add_control);

static int scu_dai_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	int ret = 0;

	FNC_ENTRY
	snd_soc_set_runtime_hwparams(substream, &scu_dai_pcm_hw);

	FNC_EXIT
	return ret;
}

static void scu_dai_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	FNC_ENTRY
	FNC_EXIT
	return;
}

static int scu_dai_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	FNC_ENTRY
	FNC_EXIT
	return 0;
}

static int scu_dai_set_fmt(struct snd_soc_dai *dai,
				unsigned int fmt)
{
	FNC_ENTRY
	FNC_EXIT
	return 0;
}

static int scu_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				unsigned int freq, int dir)
{
	FNC_ENTRY
	FNC_EXIT
	return 0;
}

static const struct snd_soc_dai_ops scu_dai_ops = {
	.startup	= scu_dai_startup,
	.shutdown	= scu_dai_shutdown,
	.prepare	= scu_dai_prepare,
	.set_sysclk	= scu_dai_set_sysclk,
	.set_fmt	= scu_dai_set_fmt,
};

static const struct snd_soc_component_driver scu_ssi_component = {
	.name		= "scu-ssi",
};

static struct snd_soc_dai_driver scu_ssi_dai = {
	.name	= "scu-ssi-dai",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.formats = SNDRV_PCM_FMTBIT_S16,
		.rates = SNDRV_PCM_RATE_8000_48000,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.formats = SNDRV_PCM_FMTBIT_S16,
		.rates = SNDRV_PCM_RATE_8000_48000,
	},
	.ops = &scu_dai_ops,
};

static void scu_alloc_scureg(void __iomem *mem)
{
	int i;
	void __iomem *offset;

	/* SCUX common */
	scux_reg->scuxreg = mem;

	/* IPC */
	offset = mem;
	for (i = 0; i < 3; i++) {
		scux_reg->ipcreg[i] = (struct scux_ipc_regs *)offset;
		offset += 0x100;
	}

	/* OPC */
	offset = mem + 0x0400;
	for (i = 0; i < 3; i++) {
		scux_reg->opcreg[i] = (struct scux_opc_regs *)offset;
		offset += 0x100;
	}

	/* FFD */
	offset = mem + 0x0800;
	for (i = 0; i < 3; i++) {
		scux_reg->ffdreg[i] = (struct scux_ffd_regs *)offset;
		offset += 0x100;
	}

	/* FFU */
	offset = mem + 0x0C00;
	for (i = 0; i < 3; i++) {
		scux_reg->ffureg[i] = (struct scux_ffu_regs *)offset;
		offset += 0x100;
	}

	/* SRC */
	offset = mem + 0x1000;
	scux_reg->srcreg[SCUX_2SRC0_0] = (struct scux_src_regs *)offset;
	offset = mem + 0x1100;
	scux_reg->srcreg[SCUX_2SRC0_1] = (struct scux_src_regs *)offset;

	/* DVU */
	offset = mem + 0x1200;
	for (i = 0; i < 3; i++) {
		scux_reg->dvureg[i] = (struct scux_dvu_regs *)offset;
		offset += 0x100;
	}

	/* MIX */
	offset = mem + 0x1600;
	scux_reg->mixreg = (struct scux_mix_regs *)offset;

	/* CIM */
	offset = mem + 0x1700;
	scux_reg->cimreg = (struct scux_cim_regs *)offset;

	return;
}

static void scu_alloc_ssif0reg(void __iomem *mem)
{
	int i;
	void __iomem *offset;

	offset = mem;
	for (i = 0; i < 5; i++) {
		scux_reg->ssifreg[i] = (struct ssif_regs *)offset;
		offset += 0x800;
		DBG_MSG("ssireg[%2d]=%08x\n", i, (int)scux_reg->ssireg[i]);
	}

	return;
}

static int scu_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct scu_clock_info *cinfo;
	struct resource *scu_res;
	struct resource *scu_region = NULL;
	struct resource *ssi_res;
	struct resource *ssi_region = NULL;
	void __iomem *mem;

	FNC_ENTRY
	if (pdev->id != 0) {
		dev_err(&pdev->dev, "current scu support id 0 only now\n");
		return -ENODEV;
	}
	pdata = pdev->dev.platform_data;

	ainfo = kzalloc(sizeof(struct scu_audio_info), GFP_KERNEL);
	if (!ainfo) {
		dev_err(&pdev->dev, "no memory\n");
		return -ENOMEM;
	}
	cinfo = &ainfo->clockinfo;
	scux_reg = &ainfo->reginfo;

	spin_lock_init(&ainfo->scu_lock);
	sculock = &ainfo->scu_lock;

	/* resource */
	scu_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!scu_res) {
		dev_err(&pdev->dev, "No memory (0) resource\n");
		ret = -ENODEV;
		goto error_clk_put;
	}

	ssi_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (!ssi_res) {
		dev_err(&pdev->dev, "No memory (2) resource\n");
		ret = -ENODEV;
		goto error_clk_put;
	}

	scu_region = request_mem_region(scu_res->start,
					resource_size(scu_res), pdev->name);
	if (!scu_region) {
		dev_err(&pdev->dev, "SCUX region already claimed\n");
		ret = -EBUSY;
		goto error_release;
	}

	ssi_region = request_mem_region(ssi_res->start,
					resource_size(ssi_res), pdev->name);
	if (!ssi_region) {
		dev_err(&pdev->dev, "SSI region already claimed\n");
		ret = -EBUSY;
		goto error_release;
	}

	/* mapping scux */
	mem = ioremap_nocache(scu_res->start, resource_size(scu_res));
	if (!mem) {
		dev_err(&pdev->dev, "ioremap failed for scu\n");
		ret = -ENOMEM;
		goto error_unmap;
	}
	scu_alloc_scureg(mem);

	mem = ioremap_nocache(ssi_res->start, resource_size(ssi_res));
	if (!mem) {
		dev_err(&pdev->dev, "ioremap failed for ssi\n");
		ret = -ENOMEM;
		goto error_unmap;
	}
	scu_alloc_ssif0reg(mem);

	ret = snd_soc_register_platform(&pdev->dev, &scu_platform);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot snd soc register\n");
		goto error_unmap;
	}

	ret = snd_soc_register_component(&pdev->dev, &scu_ssi_component,
					 &scu_ssi_dai, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot snd soc dais register\n");
		goto error_unregister;
	}

	scu_ssif_init();

	FNC_EXIT
	return ret;

error_unregister:
	snd_soc_unregister_platform(&pdev->dev);

error_unmap:
	if (scux_reg->scuxreg)
		iounmap(scux_reg->scuxreg);

	if (scux_reg->ssireg)
		iounmap(scux_reg->ssireg);

error_release:
	if (scu_region)
		release_mem_region(scu_res->start, resource_size(scu_res));
	if (ssi_region)
		release_mem_region(ssi_res->start, resource_size(ssi_res));

error_clk_put:
	FNC_EXIT
	return ret;
}

static int scu_remove(struct platform_device *pdev)
{
	struct resource *res;

	FNC_ENTRY
	snd_soc_unregister_component(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);

	iounmap(scux_reg->scuxreg);
	iounmap(scux_reg->ssireg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res)
		release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res)
		release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (res)
		release_mem_region(res->start, resource_size(res));

	kfree(ainfo);

	FNC_EXIT
	return 0;
}

static struct platform_driver scu_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "scux-pcm-audio",
	},
	.probe		= scu_probe,
	.remove		= scu_remove,
};

module_platform_driver(scu_driver);

MODULE_AUTHOR(" Renesas Solutions Corp.");
MODULE_DESCRIPTION("ALSA SoC RZA1H SCUX driver");
MODULE_LICENSE("GPL");
