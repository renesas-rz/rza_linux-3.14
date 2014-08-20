/*
 * include/sound/sh_scux.h
 *     This file is header file for SCUX peripheral.
 *
 * Copyright (C) 2013 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef SH_SCUX_H
#define SH_SCUX_H

#include <linux/dmaengine.h>
#include <linux/list.h>
#include <linux/types.h>
#include <sound/soc.h>

/************************************************************************
	ALSA
************************************************************************/
/* buffer information */
#define SCU_BUFFER_BYTES_MAX	(32 * 1024)
#define SCU_PERIOD_BYTES_MIN	512
#define SCU_PERIOD_BYTES_MAX	8192
#define SCU_PERIODS_MIN		(SCU_BUFFER_BYTES_MAX / SCU_PERIOD_BYTES_MAX)
#define SCU_PERIODS_MAX		(SCU_BUFFER_BYTES_MAX / SCU_PERIOD_BYTES_MIN)

/* scux dapm route playback route */
#define	W_SSI0			0x00000001
#define	W_SRC1			0x00000002
#define	W_DVC1			0x00000004
#define RP_MEM_SSI0		W_SSI0
#define RP_MEM_SRC1_SSI0	(W_SSI0 | W_SRC1)
#define RP_MEM_SRC1_DVC1_SSI0	(W_SSI0 | W_SRC1 | W_DVC1)
/* scux dapm route capture route */
#define	W_SSI0_CAP		0x00010000
#define	W_SRC0			0x00020000
#define	RC_SSI0_MEM		W_SSI0_CAP
#define RC_SSI0_SRC0_MEM	(W_SSI0_CAP | W_SRC0)
/* dma direction */
#define	DMA_DIR(d)	((d == 0) ? DMA_TO_DEVICE : DMA_FROM_DEVICE)

/* IP channel */
#define	MAXCH_SSIF	6
#define	MAXCH_SRC	2
#define	MAXCH_DVU	4
#define MAXCH_DMA	5 /* DMA slave IDs .ex:RP_MEM_SRC1_DVC1_SSI0 */

/* dai Control */
#define CTRL_PLAYBACK	0
#define CTRL_CAPTURE	1

/* clock */
#define AUDIO_X1	22579200	/* 22.5792Mhz */

/*  rate  */
#define	RATE_MAX	0x7fffffff

/*  volume  */
#define	VOLUME_MAX_DVC	0x007fffff
#define	VOLUME_DEFAULT	0x00100000

/* SRC channel */
enum {
	SRC0,
	SRC1,
	SRC2,
	SRC3,
};
#define	SCUX_2SRC0_0			(0)
#define	SCUX_2SRC0_1			(1)

/* SRC_BSDSR */
#define	SRC_BSD012349_BUFDATA_1_6	(0x180 << 16)
#define	SRC_BSD012349_BUFDATA_1_4	(0x100 << 16)
#define	SRC_BSD012349_BUFDATA_1_3	(0x0c0 << 16)
#define	SRC_BSD012349_BUFDATA_1_2	(0x080 << 16)
#define	SRC_BSD012349_BUFDATA_2_3	(0x060 << 16)
#define	SRC_BSD012349_BUFDATA_1_1	(0x040 << 16)
#define	SRC_BSD5678_BUFDATA_1_6		(0x240 << 16)
#define	SRC_BSD5678_BUFDATA_1_4		(0x180 << 16)
#define	SRC_BSD5678_BUFDATA_1_3		(0x120 << 16)
#define	SRC_BSD5678_BUFDATA_1_2		(0x0c0 << 16)
#define	SRC_BSD5678_BUFDATA_2_3		(0x090 << 16)
#define	SRC_BSD5678_BUFDATA_1_1		(0x060 << 16)

/* SRC_BSISR */
#define	SRC_BSI_IJECPREC		(0x10 << 16)
#define	SRC_BSI_IJECSIZE_1_6		0x60
#define	SRC_BSI_IJECSIZE_1_4		0x40
#define	SRC_BSI_IJECSIZE_1_3		0x30
#define	SRC_BSI_IJECSIZE_1_2		0x20
#define	SRC_BSI_IJECSIZE_2_3		0x20
#define	SRC_BSI_IJECSIZE_1_1		0x20

/* BUFIJEC bit */
#define BUFIJEC_LOW_DELAY_OFF		(0uL)
#define BUFIJEC_LOW_DELAY_ON		(5uL)
#define	BFSSR_BIT_BUFDATA_16BYTE	(16<<16)

/* DVC channel */
enum {
	DVC0,
	DVC1,
};

/* SSI channel */
enum {
	SSI0,
	SSI1,
	SSI2,
	SSI3,
	SSI4,
	SSI5,
	SSI6,
	SSI7,
	SSI8,
	SSI9,
};

/* SSI input/output */
#define	SSI_OUT		0
#define	SSI_IN		1

/* SSI mode */
enum {
	SSI_MASTER,
	SSI_SLAVE,
};

/* SSI dependant/independat transfer */
enum {
	SSI_DEPENDANT,
	SSI_INDEPENDANT,
};

/************************************************************************
	structure
************************************************************************/
struct scu_pcm_callback {
	void (*init_ssi)(int, int, int, int, int);
	void (*init_src)(int, unsigned int, unsigned int);
	void (*init_dvc)(int);
	void (*deinit_ssi)(int, int, int, int);
	void (*deinit_src)(int);
	void (*deinit_dvc)(int);
};
struct scu_route_info {
	int p_route;		/* playback route */
	int c_route;		/* capture route */
	/* snd_kcontrol */
	int route_ssi[MAXCH_SSIF];
	int route_src[MAXCH_SRC];
	int route_mix[1];
	int route_dvc[MAXCH_DVU];
	/* playback callback */
	struct scu_pcm_callback pcb;
	/* capture callback */
	struct scu_pcm_callback ccb;
};

struct scu_clock_info {
	struct clk *adg_clk;
	struct clk *scu_clk;
	struct clk *src0_clk;
	struct clk *src1_clk;
	struct clk *dvc0_clk;
	struct clk *dvc1_clk;
	struct clk *ssiu_clk;
	struct clk *ssi0_clk;
	struct clk *ssi1_clk;
};

struct scu_config {
	int label;
	int value;
};

struct scu_platform_data {
	int ssi_master;
	int ssi_slave;
	struct scu_config *ssi_ch;
	int ssi_ch_num;
	struct scu_config *src_ch;
	int src_ch_num;
	struct scu_config *dvc_ch;
	int dvc_ch_num;
	int dma_slave_maxnum;
	struct scu_config *audma_slave;
	int audma_slave_num;
	struct scu_config *ssi_depend;
	int ssi_depend_num;
	struct scu_config *ssi_mode;
	int ssi_mode_num;
	struct scu_config *src_mode;
	int src_mode_num;
};

struct scu_pcm_info {
	int flag_first;			/* for PCM 1st process */
	int flag_start;			/* for DMA control */
	unsigned int buf_offset;	/* for buffer control */
	unsigned int tran_size;		/* total size of transferred frame */
	unsigned int period_piece;	/* piece of period to transfer */
	unsigned int tran_period_piece;	/* transferred piece of period */
	spinlock_t pcm_lock;		/* for trigger process */
	struct dma_chan **de_chan;
	struct rza1_dma_slave *de_param;
	struct work_struct work;
	struct workqueue_struct *workq;
	struct scu_route_info *routeinfo;
	struct snd_pcm_substream *ss;
	struct scu_platform_data *pdata;
};

/************************************************************************
	inline function
************************************************************************/
static inline int scu_find_data(int val, struct scu_config *data, int size)
{
	int i;
	struct scu_config *data_p = data;

	for (i = 0; i < size; data_p++, i++) {
		if (val == data_p->label)
			return data_p->value;
	}

	return -1;
}
/******************************************************************************
 ==== SCUX ====
******************************************************************************/
enum scux_ch {
	SCUX_CH_0,
	SCUX_CH_1,
	SCUX_CH_2,
	SCUX_CH_3,
	NUM_SCUX_CH,	/* 4ch */
};
/******************************************************************************
 ==== SCUX register value list ====
******************************************************************************/
/* IPCIR Register Value */
#define IPCIR_INIT_ON		(1uL)	/* Set INIT of IPC */

/* IPSLR Register Value */
#define IPSLR_SSIF_IPC_ASYNC	(1uL)
#define IPSLR_FFD_IPC_ASYNC	(3uL)
#define IPSLR_FFD_IPC_SYNC	(4uL)

/* OPCIR Register Value */
#define OPCIR_INIT_ON		(1uL)	/* Set INIT of OPC */

/* OPSLR Register Value */
#define OPSLR_ASYNC_OPC_DVU	(1uL)
#define OPSLR_ASYNC_OPC_FFU	(3uL)
#define OPSLR_SYNC_OPC_FFU	(4uL)

/* FFDIR Register Value */
#define FFDIR_INIT_OFF		(0uL)	/* Set INIT of FFD */
#define FFDIR_INIT_ON		(1uL)	/* Set INIT of FFD */

/* FDAIR Register Value */
#define FDAIR_AUDIO_CH_NONE	(0uL)
#define FDAIR_AUDIO_CH_1	(1uL)
#define FDAIR_AUDIO_CH_2	(2uL)
#define FDAIR_AUDIO_CH_4	(4uL)
#define FDAIR_AUDIO_CH_6	(6uL)
#define FDAIR_AUDIO_CH_8	(8uL)

/* DRQSR Register Value */
#define DRQSR_REQ_SIZE_256	(0uL)	/* 256data (FFU0_0 and FFU0_1 Only) */
#define DRQSR_REQ_SIZE_128	(1uL)	/* 128data (FFU0_0 and FFU0_1 Only) */
#define DRQSR_REQ_SIZE_64	(2uL)	/* 64data   */
#define DRQSR_REQ_SIZE_32	(3uL)	/* 32data   */
#define DRQSR_REQ_SIZE_16	(4uL)	/* 16data   */
#define DRQSR_REQ_SIZE_8	(5uL)	/* 8data    */
#define DRQSR_REQ_SIZE_4	(6uL)	/* 4data    */
#define DRQSR_REQ_SIZE_2	(7uL)	/* 2data    */
#define DRQSR_REQ_SIZE_1	(8uL)	/* 1data    */

/* FFDPR Register Value */
#define FFDPR_NO_PASS_SELECT	(0uL)
#define FFDPR_CIM_FFD_IPC_ASYNC	(1uL)
#define FFDPR_CIM_FFD_IPC_SYNC	(2uL)

/* FFDBR Register Value */
#define FFDBR_HALT_ON		(0uL)
#define FFDBR_BOOT_ON		(1uL)

/* DEVMR Register Value */
/* Configure SCUX
DEVMR_FFD0_n - FFD0_n FIFO Download Event Mask Register(n=0,1,2,3)
b31   DEVMUF - FFD Underflow Mask - Interrupt enabled
b30   DEVMOF - FFD Overflow Mask - Interrupt enabled
b29   DEVMOL - FFD Overlap Mask - Interrupt enabled
b28   DEVMIUF - FFD Initialization Underflow Mask - Interrupt enabled
b15   DEVMRQ - FFD Request Packet Mask - Interrupt disabled
Other Reserved - The write value should always be 0 */
/* Initial value of DEVMR_FFD0_n Register */
#define DEVMR_VALUE         (0xF0000000uL)

/* DEVCR Register Value */
/* Request ON bits make 0, Other bits make 1 */
#define DEVCR_REG_MASK      (0xF0008000uL)

/* FFUIR Register Value */
#define FFUIR_INIT_OFF		(0uL)		/* Set INIT of FFU */
#define FFUIR_INIT_ON		(1uL)		/* Set INIT of FFU */

/* FUAIR Register Value */
#define FUAIR_AUDIO_CH_NONE	(0uL)
#define FUAIR_AUDIO_CH_1	(1uL)
#define FUAIR_AUDIO_CH_2	(2uL)
#define FUAIR_AUDIO_CH_4	(4uL)
#define FUAIR_AUDIO_CH_6	(6uL)
#define FUAIR_AUDIO_CH_8	(8uL)

/* URQSR Register Value */
/*URQSR_FFU0_n - FFU0_n FIFO Upload Request Size Register(n=0,1,2,3)
b3:b0 SIZE - Request size data - Set parameter
Other Reserved - The write value should always be 0 */
#define URQSR_CH0CH1_REQ_SIZE_256	(0uL)	/* 256data (FFU0_0,0_1 Only) */
#define URQSR_CH0CH1_REQ_SIZE_128	(1uL)	/* 128data (FFU0_0,0_1 Only) */
#define URQSR_CH0CH1_REQ_SIZE_64	(2uL)	/* 64data   */
#define URQSR_CH0CH1_REQ_SIZE_32	(3uL)	/* 32data   */
#define URQSR_CH0CH1_REQ_SIZE_16	(4uL)	/* 16data   */
#define URQSR_CH0CH1_REQ_SIZE_8		(5uL)	/* 8data    */
#define URQSR_CH0CH1_REQ_SIZE_4		(6uL)	/* 4data    */
#define URQSR_CH0CH1_REQ_SIZE_2		(7uL)	/* 2data    */
#define URQSR_CH0CH1_REQ_SIZE_1		(8uL)	/* 1data    */
/* In the case of SCUX Channel 2 or 3, a value of 64 data is 0.*/
#define URQSR_CH2CH3_REQ_SIZE_64	(0uL)	/* 64data   */
#define URQSR_CH2CH3_REQ_SIZE_32	(1uL)	/* 32data   */
#define URQSR_CH2CH3_REQ_SIZE_16	(2uL)	/* 16data   */
#define URQSR_CH2CH3_REQ_SIZE_8		(3uL)	/* 8data    */
#define URQSR_CH2CH3_REQ_SIZE_4		(4uL)	/* 4data    */
#define URQSR_CH2CH3_REQ_SIZE_2		(5uL)	/* 2data    */
#define URQSR_CH2CH3_REQ_SIZE_1		(6uL)	/* 1data    */

/* FFUPR Register Value */
#define FFUPR_NO_PASS_SELECT	(0uL)
#define FFUPR_CIM_FFU_OPC_ASYNC	(1uL)
#define FFUPR_CIM_FFU_OPC_SYNC	(2uL)

/* UEVMR Register Value */
/* Initial value of UEVMR_FFU0_n Register */
#define UEVMR_VALUE         (0xE0000000uL)

/* UEVCR Register Value */
#define UEVCR_REG_MASK		(0xE0008000uL)
#define UEVCR_BIT_UEVCUF	(0x80000000uL)
#define UEVCR_BIT_UEVCOF	(0x40000000uL)
#define UEVCR_BIT_UEVCOL	(0x20000000uL)
#define UEVCR_BIT_UEVCRQ	(0x00008000uL)

/* SRCIR Register Value */
#define INIT_OFF		(0uL)	/* Clear INIT of SRC */
#define SRCIR_INIT_ON		(1uL)	/* Set INIT of SRC */

/* SRC_ADINR */
#define	SRCADIN_OTBL_24BIT	(0<<16)
#define	SRCADIN_OTBL_22BIT	(2<<16)
#define	SRCADIN_OTBL_20BIT	(4<<16)
#define	SRCADIN_OTBL_18BIT	(6<<16)
#define	SRCADIN_OTBL_16BIT	(8<<16)
#define	SRCADIN_OTBL_8BIT	(16<<16)
#define	SRCADIN_CHNUM_0		(0<<0)
#define	SRCADIN_CHNUM_1		(1<<0)
#define	SRCADIN_CHNUM_2		(2<<0)
#define	SRCADIN_CHNUM_4		(4<<0)
#define	SRCADIN_CHNUM_6		(6<<0)
#define	SRCADIN_CHNUM_8		(8<<0)

/* SADIR Register Value */
/* bit 3-0 */
#define SADIR_AUDIO_CH_NONE	(0uL)
#define SADIR_AUDIO_CH_1	(1uL)
#define SADIR_AUDIO_CH_2	(2uL)
#define SADIR_AUDIO_CH_4	(4uL)
#define SADIR_AUDIO_CH_6	(6uL)
#define SADIR_AUDIO_CH_8	(8uL)
/*bit 20-16*/
#define SADIR_OTBL_24BIT	(0<<16)
#define SADIR_OTBL_16BIT	(8<<16)

/* SRCBR Register Value */
/*SRCBRp_2SRC0_m - 2SRC0_m Bypass Register p(m=0,1 p=0,1)
b0    BYPASS - Bypass Mode - Set parameter
Other Reserved - The write value should always be 0 */
#define SRCBR_BYPASS_OFF	(0uL)
#define SRCBR_BYPASS_ON		(1uL)

/* IFSCR Register Value */
/* IFSCRp_2SRC0_m - 2SRC0_m IFS Control Register p(m=0,1 p=0,1)
b0    INTIFSEN - INTIFS value setting enable - Initial value enabled
Other Reserved - The write value should always be 0 */
/* Value of IFSCRp_2SRC0_m Register */
#define IFSCR_VALUE		(1uL)


/* IFSVR Register Value */
#define IFSVR_INTIFS_NOIFSVR	(0uL)
#define NUM_OUTPUT_FS		(7uL)
#define NUM_INPUT_FS		(12uL)
#define	SRC_IFS_FSO		(0x00400000uL)	/* 2^22 */
#define	SRC_IFS_8KHZ		(8000uL)
#define	SRC_IFS_11KHZ		(11025uL)
#define	SRC_IFS_12KHZ		(12000uL)
#define	SRC_IFS_16KHZ		(16000uL)
#define	SRC_IFS_22KHZ		(22050uL)
#define	SRC_IFS_24KHZ		(24000uL)
#define	SRC_IFS_32KHZ		(32000uL)
#define	SRC_IFS_44KHZ		(44100uL)
#define	SRC_IFS_48KHZ		(48000uL)
#define INTIFS_IN44K_OUT8K	0x160cccc;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT11K	0x1000000;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT12K	0x0EB3333;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT16K	0x0b06666;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT22K	0x0802531;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT24K	0x0759999;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT32K	0x0583333;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT44K	0x0400000;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT48K	0x03acccc;	/* IFSVRp_2SRC0_0 INTIFS */

#define INTIFS_IN8K_OUT44K	0x00b9c27;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN11K_OUT44K	0x0100000;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN12K_OUT44K	0x0116a3b;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN16K_OUT44K	0x017384e;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN22K_OUT44K	0x0200000;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN24K_OUT44K	0x022d476;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN32K_OUT44K	0x02e709d;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN44K_OUT44K	0x0400000;	/* IFSVRp_2SRC0_0 INTIFS */
#define INTIFS_IN48K_OUT44K	0x045a8ec;	/* IFSVRp_2SRC0_0 INTIFS */

/* SRCCR Register Value */
/* SRCCRp_2SRC0_m - 2SRC0_m Control Register p(m=0,1 p=0,1)
b20     WATMD - Wait Time Control of SRC - Set parameter
b12     BUFMD - Low Delay Control by Buffer Size of SRC - Set parameter
b0      SRCMD - Select SRC Mode of SRCI - Set parameter
b16,8,4 Reserved - These bits should be set to 1
Other   Reserved - The write value should always be 0 */
#define	SRC_CR_BIT16		(1<<16)
#define	SRC_CR_BIT12		(1<<12)
#define	SRC_CR_LOW_DELAY_NON	(0<<12)
#define	SRC_CR_BIT8		(1<<8)
#define	SRC_CR_BIT4		(1<<4)
#define	SRC_CR_SYNC		(1<<0)
#define	SRC_CR_ASYNC		(0<<0)

/* MNFSR Register Value */
/*MNFSRp_2SRC0_m - 2SRC0_m Minimum FS Setting Register p(m=0,1 p=0,1)
b27:b0 MINFS - Set the minimum FS ratio - Set parameter
Other  Reserved - The write value should always be 0 */
/* value of min_fs_ratio */
#define MIN_FS_RATIO_90			(90)	/* 90% */
#define MIN_FS_RATIO_98			(98)	/* 98% */
#define MIN_FS_RATIO_100		(100)	/* 100 */
#define MIN_FS_RATIO_DECIMAL_DIGITS	(24uL)	/* digits of min_fs_ratio */

/* BFSSR Register Value */
/* BFSSRp_2SRC0_m - 2SRC0_m Buffer Size Setting Register p(m=0,1 p=0,1)
b25:b16 BUFDATA - Set the buffer size of 1 channel in DATA RAM - Set parameter
b3:b0   BUFIJEC - Set the buffer size of 1 channel in IJEC RAM -
	Sets 0x5 for low delay mode
Other   Reserved - The write value should always be 0 */
/* Sets 0x80 for low delay mode.1 */
#define BFSSR_BUFDATA_MODE1 (0x80uL << 16)
/* Sets 0x40 for low delay mode.2 */
#define BFSSR_BUFDATA_MODE2 (0x40uL << 16)
#define BFSSR_BUFIN_LOW_DELAY_OFF	(0uL)
#define BFSSR_BUFIN_LOW_DELAY_ON	(5uL)	/* Sets 0x5 for low delay mode*/

/* SC2SR Register Value */
/* ALL Bit is ReadOnly */
#define BFSSR_SRCWSTS			(1<<31) /* 1:Reached to wait time */
#define BFSSR_SC2MUTE			(1<<30) /* 1:Output data of sampling
						   rate conversion is stable. */
#define BFSSR_SRCWSTS_REST		(0<<28) /* 00:SRC is resetting */
#define BFSSR_SRCWSTS_INIT		(1<<28) /* 01:SRC is initialized */
#define BFSSR_SRCWSTS_OPER		(2<<28) /* 10:SRC is operating */

/* WATSR Register Value */
/*WATSRp_2SRC0_m - 2SRC0_m Wait Time Setting Register p(m=0,1 p=0,1)
b23:b0 WTIME - wait time - Set parameter
Other  Reserved - The write value should always be 0 */
/* SCUX SRC wait time */
#define SCUX_MIN_WAIT		(0x00000000L)
#define SCUX_MAX_WAIT		(0x00FFFFFFL)
#define SCUX_NO_WAIT		(-1L)

/* SEVMR Register Value */
/* SEVMRp_2SRC0_m - 2SRC0_m Event Mask Register p(m=0,1 p=0,1)
b31 EVMUF - Sampling Rate Converter Underflow Mask - Interrupt enabled
b30 EVMOF - Sampling Rate Converter Overflow Mask - Interrupt enabled
b14 EVMWAIT - Sampling Rate Converter Wait Time Mask - Interrupt enabled
Other  Reserved - The write value should always be 0 */
/* Value of SEVMRp_2SRC0_m Register */
#define SEVMR_VALUE           (0xc0004000uL)

/* SEVCR Register Value */
#define SEVCR_REG_MASK		(0xC0004000uL)
#define SEVCR_EVCUF		(0x80000000uL)
#define SEVCR_EVCOF		(0x40000000uL)
#define SEVCR_EVCWAIT		(0x00004000uL)

/* SRCIRR Register Value */
#define SRCIRR_INIT_ON		(1uL)		/* Set INIT of SRC */

/* DVUIR Register Value */
#define DVUIR_INIT_ON		(1uL)		/* Set INIT of DVU */

/* VADIR Register Value */
/* VADIR_DVU0_n - DVU0_n Audio Information Register(n=0,1,2,3)
b20:b16 OTBL - Bit Length of Output Audio Data - Set parameter
b3:b0   CHNUM - Channel Number - Set parameter
Other   Reserved - The write value should always be 0 */
/*bit 20-16*/
#define VADIR_OTBL_24BIT	(0<<16)
#define VADIR_OTBL_16BIT	(8<<16)
/* bit 3-0 */
#define VADIR_CHNUM_CH_NONE	(0uL)
#define VADIR_CHNUM_CH_1	(1uL)
#define VADIR_CHNUM_CH_2	(2uL)
#define VADIR_CHNUM_CH_4	(4uL)
#define VADIR_CHNUM_CH_6	(6uL)
#define VADIR_CHNUM_CH_8	(8uL)

/* DVUBR Register Value */
#define DVUBR_BYPASS_OFF	(0uL)
#define DVUBR_BYPASS_ON		(1uL)

/* DVUCR Register Value */
#define	DVUCR_VVMD_SLEEP	(0<<8)
#define	DVUCR_VVMD_USE		(1<<8)
#define	DVUCR_VRMD_SLEEP	(0<<4)
#define	DVUCR_VRMD_USE		(1<<4)
#define	DVUCR_ZCMD_SLEEP	(0<<0)
#define	DVUCR_ZCMD_USE		(1<<0)

/* ZCMCR Register Value */
#define ZCMCR_ZCEN0_EN		(1<<0)
#define ZCMCR_ZCEN1_EN		(1<<1)
#define ZCMCR_ZCEN2_EN		(1<<2)
#define ZCMCR_ZCEN3_EN		(1<<3)
#define ZCMCR_ZCEN4_EN		(1<<4)
#define ZCMCR_ZCEN5_EN		(1<<5)
#define ZCMCR_ZCEN6_EN		(1<<6)
#define ZCMCR_ZCEN7_EN		(1<<7)

/* VRCTR Register Value */
/* VRCTR_DVU0_n - DVU0_n Volume Ramp Control Register(n=0,1,2,3)
b7:b0 VREN7:VREN0 - Volume Ramp Enable for Channel 7 to 0 - Set parameter
Other Reserved - The write value should always be 0 */
#define VRCTR_VREN0_EN		(1<<0)
#define VRCTR_VREN1_EN		(1<<1)
#define VRCTR_VREN2_EN		(1<<2)
#define VRCTR_VREN3_EN		(1<<3)
#define VRCTR_VREN4_EN		(1<<4)
#define VRCTR_VREN5_EN		(1<<5)
#define VRCTR_VREN6_EN		(1<<6)
#define VRCTR_VREN7_EN		(1<<7)

/* VRPDR Register Value */
enum scux_ramp_period {
	/*                vol_up , vol_down             */
	DVU_PERIOD_1,		/*       1sample (  128dB,   -128dB/1step)   */
	DVU_PERIOD_2,		/*       2sample (   64dB,    -64dB/1step)   */
	DVU_PERIOD_4,		/*       4sample (   32dB,    -32dB/1step)   */
	DVU_PERIOD_8,		/*       8sample (   16dB,    -16dB/1step)   */
	DVU_PERIOD_16,		/*      16sample (    8dB,     -8dB/1step)   */
	DVU_PERIOD_32,		/*      32sample (    4dB,     -4dB/1step)   */
	DVU_PERIOD_64,		/*      64sample (    2dB,     -2dB/1step)   */
	DVU_PERIOD_128,		/*     128sample (    1dB,     -1dB/1step)   */
	DVU_PERIOD_256,		/*     256sample (  0.5dB,   -0.5dB/1step)   */
	DVU_PERIOD_512,		/*     512sample ( 0.25dB,  -0.25dB/1step)   */
	DVU_PERIOD_1024,	/*    1024sample (0.125dB, -0.125dB/1step)   */
	DVU_PERIOD_2048,	/*    2048sample (0.125dB, -0.125dB/2step)   */
	DVU_PERIOD_4096,	/*    4096sample (0.125dB, -0.125dB/4step)   */
	DVU_PERIOD_8192,	/*    8192sample (0.125dB, -0.125dB/8step)   */
	DVU_PERIOD_16384,	/*   16384sample (0.125dB, -0.125dB/16step)  */
	DVU_PERIOD_32768,	/*   32768sample (0.125dB, -0.125dB/32step)  */
	DVU_PERIOD_65536,	/*   65536sample (0.125dB, -0.125dB/64step)  */
	DVU_PERIOD_131072,	/*  131072sample (0.125dB, -0.125dB/128step) */
	DVU_PERIOD_262144,	/*  262144sample (0.125dB, -0.125dB/256step) */
	DVU_PERIOD_524288,	/*  524288sample (0.125dB, -0.125dB/512step) */
	DVU_PERIOD_1048576,	/* 1048576sample (0.125dB, -0.125dB/1024step)*/
	DVU_PERIOD_2097152,	/* 2097152sample (0.125dB, -0.125dB/2048step)*/
	DVU_PERIOD_4194304,	/* 4194304sample (0.125dB, -0.125dB/4096step)*/
	DVU_PERIOD_8388608,	/* 8388608sample (0.125dB, -0.125dB/8192step)*/
};
#define VRPDR_VRPDUP_PERIOD_1       (DVU_PERIOD_1       << 8)
#define VRPDR_VRPDUP_PERIOD_2       (DVU_PERIOD_2       << 8)
#define VRPDR_VRPDUP_PERIOD_4       (DVU_PERIOD_4       << 8)
#define VRPDR_VRPDUP_PERIOD_8       (DVU_PERIOD_8       << 8)
#define VRPDR_VRPDUP_PERIOD_16      (DVU_PERIOD_16      << 8)
#define VRPDR_VRPDUP_PERIOD_32      (DVU_PERIOD_32      << 8)
#define VRPDR_VRPDUP_PERIOD_64      (DVU_PERIOD_64      << 8)
#define VRPDR_VRPDUP_PERIOD_128     (DVU_PERIOD_128     << 8)
#define VRPDR_VRPDUP_PERIOD_256     (DVU_PERIOD_256     << 8)
#define VRPDR_VRPDUP_PERIOD_512     (DVU_PERIOD_512     << 8)
#define VRPDR_VRPDUP_PERIOD_1024    (DVU_PERIOD_1024    << 8)
#define VRPDR_VRPDUP_PERIOD_2048    (DVU_PERIOD_2048    << 8)
#define VRPDR_VRPDUP_PERIOD_4096    (DVU_PERIOD_4096    << 8)
#define VRPDR_VRPDUP_PERIOD_8192    (DVU_PERIOD_8192    << 8)
#define VRPDR_VRPDUP_PERIOD_16384   (DVU_PERIOD_16384   << 8)
#define VRPDR_VRPDUP_PERIOD_32768   (DVU_PERIOD_32768   << 8)
#define VRPDR_VRPDUP_PERIOD_65536   (DVU_PERIOD_65536   << 8)
#define VRPDR_VRPDUP_PERIOD_131072  (DVU_PERIOD_131072  << 8)
#define VRPDR_VRPDUP_PERIOD_262144  (DVU_PERIOD_262144  << 8)
#define VRPDR_VRPDUP_PERIOD_524288  (DVU_PERIOD_524288  << 8)
#define VRPDR_VRPDUP_PERIOD_1048576 (DVU_PERIOD_1048576 << 8)
#define VRPDR_VRPDUP_PERIOD_2097152 (DVU_PERIOD_2097152 << 8)
#define VRPDR_VRPDUP_PERIOD_4194304 (DVU_PERIOD_4194304 << 8)
#define VRPDR_VRPDUP_PERIOD_8388608 (DVU_PERIOD_8388608 << 8)

#define VRPDR_VRPDDW_PERIOD_1       (DVU_PERIOD_1)
#define VRPDR_VRPDDW_PERIOD_2       (DVU_PERIOD_2)
#define VRPDR_VRPDDW_PERIOD_4       (DVU_PERIOD_4)
#define VRPDR_VRPDDW_PERIOD_8       (DVU_PERIOD_8)
#define VRPDR_VRPDDW_PERIOD_16      (DVU_PERIOD_16)
#define VRPDR_VRPDDW_PERIOD_32      (DVU_PERIOD_32)
#define VRPDR_VRPDDW_PERIOD_64      (DVU_PERIOD_64)
#define VRPDR_VRPDDW_PERIOD_128     (DVU_PERIOD_128)
#define VRPDR_VRPDDW_PERIOD_256     (DVU_PERIOD_256)
#define VRPDR_VRPDDW_PERIOD_512     (DVU_PERIOD_512)
#define VRPDR_VRPDDW_PERIOD_1024    (DVU_PERIOD_1024)
#define VRPDR_VRPDDW_PERIOD_2048    (DVU_PERIOD_2048)
#define VRPDR_VRPDDW_PERIOD_4096    (DVU_PERIOD_4096)
#define VRPDR_VRPDDW_PERIOD_8192    (DVU_PERIOD_8192)
#define VRPDR_VRPDDW_PERIOD_16384   (DVU_PERIOD_16384)
#define VRPDR_VRPDDW_PERIOD_32768   (DVU_PERIOD_32768)
#define VRPDR_VRPDDW_PERIOD_65536   (DVU_PERIOD_65536)
#define VRPDR_VRPDDW_PERIOD_131072  (DVU_PERIOD_131072)
#define VRPDR_VRPDDW_PERIOD_262144  (DVU_PERIOD_262144)
#define VRPDR_VRPDDW_PERIOD_524288  (DVU_PERIOD_524288)
#define VRPDR_VRPDDW_PERIOD_1048576 (DVU_PERIOD_1048576)
#define VRPDR_VRPDDW_PERIOD_2097152 (DVU_PERIOD_2097152)
#define VRPDR_VRPDDW_PERIOD_4194304 (DVU_PERIOD_4194304)
#define VRPDR_VRPDDW_PERIOD_8388608 (DVU_PERIOD_8388608)

/* VRDBR Register Value */
#define VRDBR_VRDB_MUTE		(0x000003FFuL)
#define VRDBR_VRDB_0DB		(0x00000000uL)

/* VRWTR Register Value */
#define VRWTR_VRWT		(0x00FFFFFFuL)

/* VOL0R~VOL7R Register Value */
#define SCUX_MAX_VOL_VALUE  (0x007FFFFFuL) /* SCUX DVU digital volume level */

/* DVUER Register Value */
#define DVUER_DVUEN_DI		(0uL)
#define DVUER_DVUEN_EN (1uL) /* Set DVUER_DVU0_n register - DVUEN bit */

/* DVUSR Register Value */
#define DVUSR_VRSTS_MUTE (0uL)
#define DVUSR_VRSTS_VOLRAMPDOWN	(1uL)
#define DVUSR_VRSTS_VOLRAMPUP	(2uL)
#define DVUSR_VRSTS_VRDBRREG	(3uL)
#define DVUSR_VRSTS_VOLINDATA	(4uL)

#define DVUSR_ZSTS0_MUTE	(1<<16)
#define DVUSR_ZSTS1_MUTE	(1<<17)
#define DVUSR_ZSTS2_MUTE	(1<<18)
#define DVUSR_ZSTS3_MUTE	(1<<19)
#define DVUSR_ZSTS4_MUTE	(1<<20)
#define DVUSR_ZSTS5_MUTE	(1<<21)
#define DVUSR_ZSTS6_MUTE	(1<<22)
#define DVUSR_ZSTS7_MUTE	(1<<23)

/* VEVMR Register Value */
#define VEVMR_VEVMZCM7_EN	(1<<31)
#define VEVMR_VEVMZCM6_EN	(1<<30)
#define VEVMR_VEVMZCM5_EN	(1<<29)
#define VEVMR_VEVMZCM4_EN	(1<<28)
#define VEVMR_VEVMZCM3_EN	(1<<27)
#define VEVMR_VEVMZCM2_EN	(1<<26)
#define VEVMR_VEVMZCM1_EN	(1<<25)
#define VEVMR_VEVMZCM0_EN	(1<<24)
#define VEVMR_VEVMVR_EN		(1<<15)

/* VEVCR Register Value */
#define VEVCR_VEVCZCM7_EN	(1<<31)
#define VEVCR_VEVCZCM6_EN	(1<<30)
#define VEVCR_VEVCZCM5_EN	(1<<29)
#define VEVCR_VEVCZCM4_EN	(1<<28)
#define VEVCR_VEVCZCM3_EN	(1<<27)
#define VEVCR_VEVCZCM2_EN	(1<<26)
#define VEVCR_VEVCZCM1_EN	(1<<25)
#define VEVCR_VEVCZCM0_EN	(1<<24)
#define VEVCR_VEVCVR_EN		(1<<15)

/* SWRSR Register Value */
#define SWRSR_SWRST_RST		(0uL)
#define SWRSR_SWRST_OPE		(1uL)

/* DMACR Register Value */
#define DMACR_DMAMDFFD0_EN	(1<<0)
#define DMACR_DMAMDFFD1_EN	(1<<1)
#define DMACR_DMAMDFFD2_EN	(1<<2)
#define DMACR_DMAMDFFD3_EN	(1<<3)
#define DMACR_DMAMDFFU0_EN	(1<<4)
#define DMACR_DMAMDFFU1_EN	(1<<5)
#define DMACR_DMAMDFFU2_EN	(1<<6)
#define DMACR_DMAMDFFU3_EN	(1<<7)

/* SSIRSEL Register Value */
/* SSIRSEL_CIM - SSI Route Select Register
b31:b30 SISEL3 - SRC3 SSIF Input Select - Set parameter
b29:b28 SISEL2 - SRC2 SSIF Input Select - Set parameter
b27:b26 SISEL1 - SRC1 SSIF Input Select - Input from SSIF3
b25:b24 SISEL0 - SRC0 SSIF Input Select - input from SSIF0
b21:b20 SOSEL5 - SSIF5 Output Select - Output from SRC3 (DVU0_3)
b17:b16 SOSEL4 - SSIF4 Output Select - Output from SRC2 (DVU0_2)
b13:b12 SOSEL3 - SSIF3 Output Select - Set parameter
b9:b8   SOSEL2 - SSIF2 Output Select - Output from SRC3 (DVU0_3)
b5:b4   SOSEL1 - SSIF1 Output Select - Output from SRC2 (DVU0_2)
b1:b0   SOSEL0 - SSIF0 Output Select - Set parameter
Other Reserved - The write value should always be 0 */
#define SSIRSEL_SISEL3_USE_SSIF2	(0<<30)
#define SSIRSEL_SISEL3_USE_SSIF5	(1<<30)
#define SSIRSEL_SISEL2_USE_SSIF1	(0<<28)
#define SSIRSEL_SISEL2_USE_SSIF4	(1<<28)
#define SSIRSEL_SISEL1_USE_SSIF3	(0<<26)
#define SSIRSEL_SISEL1_USE_SSIF345	(1<<26)
#define SSIRSEL_SISEL0_USE_SSIF0	(0<<24)
#define SSIRSEL_SISEL0_USE_SSIF012	(1<<24)
#define SSIRSEL_SOSEL5_USE_SRC3		(0<<20)
#define SSIRSEL_SOSEL5_USE_SRC1		(1<<20)
#define SSIRSEL_SOSEL5_USE_SRC0		(2<<20)
#define SSIRSEL_SOSEL4_USE_SRC2		(0<<16)
#define SSIRSEL_SOSEL4_USE_SRC1		(1<<16)
#define SSIRSEL_SOSEL4_USE_SRC0		(2<<16)
#define SSIRSEL_SOSEL3_USE_SRC1		(0<<12)
#define SSIRSEL_SOSEL3_USE_SRC0		(1<<12)
#define SSIRSEL_SOSEL2_USE_SRC3		(0<<8)
#define SSIRSEL_SOSEL2_USE_SRC0		(1<<8)
#define SSIRSEL_SOSEL2_USE_SRC1		(2<<8)
#define SSIRSEL_SOSEL1_USE_SRC2		(0<<4)
#define SSIRSEL_SOSEL1_USE_SRC0		(1<<4)
#define SSIRSEL_SOSEL1_USE_SRC1		(2<<4)
#define SSIRSEL_SOSEL0_USE_SRC0		(0)
#define SSIRSEL_SOSEL0_USE_SRC1		(1)

/* FDTSEL Register Value */
/* Configure SCUX
FDTSELn_CIM - FFD0_n Timing Select Register(n=0,1,2,3)
b26:16 SCKDIV - Division Ratio
b8     DIVEN - Division Enable - Division disabled
b3:0   SCKSEL - Clock Select
Other Reserved - The write value should always be 0 */

#define FDTSEL_SCKSEL_AUDIO_CLK		(0x0uL)
#define FDTSEL_SCKSEL_AUIDIO_X1		(0x1uL)
#define FDTSEL_SCKSEL_MLB_CLK		(0x2uL)
#define FDTSEL_SCKSEL_USB_X1		(0x3uL)
#define FDTSEL_SCKSEL_CLKP1_PHY_DIV2	(0x4uL)
#define FDTSEL_SCKSEL_SSIF0_WS		(0x8uL)
#define FDTSEL_SCKSEL_SSIF1_WS		(0x9uL)
#define FDTSEL_SCKSEL_SSIF2_WS		(0xAuL)
#define FDTSEL_SCKSEL_SSIF3_WS		(0xBuL)
#define FDTSEL_SCKSEL_SSIF4_WS		(0xCuL)
#define FDTSEL_SCKSEL_SSIF5_WS		(0xDuL)

#define SCUX_FDTSEL_BIT_DIVEN		(0x00000100uL)

#define FDTSEL_CH_0_SCKDIV		(0<<16)
#define FDTSEL_CH_1_SCKDIV		(694uL)	/* P1PHY(66.67MHz)/2/48KHz */
#define FDTSEL_CH_2_SCKDIV		(694uL)	/* P1PHY(66.67MHz)/2/48KHz */
#define FDTSEL_CH_3_SCKDIV		(694uL)	/* P1PHY(66.67MHz)/2/48KHz */

/* FUTSEL Register Value */
#define FUTSEL_SCKSEL_AUDIO_CLK		(0x0uL)
#define FUTSEL_SCKSEL_AUIDIO_X1		(0x1uL)
#define FUTSEL_SCKSEL_MLB_CLK		(0x2uL)
#define FUTSEL_SCKSEL_USB_X1		(0x3uL)
#define FUTSEL_SCKSEL_CLKP1_PHY_DIV2	(0x4uL)
#define FUTSEL_SCKSEL_SSIF0_WS		(0x8uL)
#define FUTSEL_SCKSEL_SSIF1_WS		(0x9uL)
#define FUTSEL_SCKSEL_SSIF2_WS		(0xAuL)
#define FUTSEL_SCKSEL_SSIF3_WS		(0xBuL)
#define FUTSEL_SCKSEL_SSIF4_WS		(0xCuL)
#define FUTSEL_SCKSEL_SSIF5_WS		(0xDuL)

#define FUTSEL_DIVEN_START		(0x00000100uL)

#define FUTSEL_CH_0_SCKDIV		(0<<16)
#define FUTSEL_CH_1_SCKDIV		(0<<16)
#define FUTSEL_CH_2_SCKDIV		(0<<16)
#define FUTSEL_CH_3_SCKDIV		(0<<16)

#define SCKDIV_MAX			(2047)

/* ==== SCUX SSIPMD_CIM register setting ==== */
#define THIS_CH_NOT_USED		(0uL)

/* SSIPMD Register Value */
#define SSIPMD_SSI5CKS			(0<<21)
#define SSIPMD_SSI4CKS			(0<<20)
#define SSIPMD_SSI3CKS			(0<<19)
#define SSIPMD_SSI2CKS			(0<<18)
#define SSIPMD_SSI1CKS			(0<<17)
#define SSIPMD_SSI0CKS			(0<<16)
#define SSIPMD_SSI5PMD			(0<<8)
#define SSIPMD_SSI4PMD			(0<<10)
#define SSIPMD_SSI3PMD			(0<<14)
#define SSIPMD_SSI2PMD			(0<<2)
#define SSIPMD_SSI1PMD			(0)
#define SSIPMD_INDEPENDENCE		(0x0uL)
#define SSIPMD_COMMON_SLAVE		(0x1uL)
#define SSIPMD_COMMON_MASTER_SLAVE	(0x2uL)

/* SSICTRL Register Value */
/* SSICTRL_CIM - SSI Control Register
b30:28 SSI3TX,SSI4TX,SSI5TX - SSIF3,4,5 Direct Transmission - Set parameter
b26:24 SSI3RX,SSI4RX,SSI5RX - SSIF3,4,5 Direct Reception - Set parameter
b17    SSI345TEN - SSIF345 Transmission Enable - Transmission disabled
b16    SSI345REN - SSIF345 Reception Enable - Reception disabled
b14:12 SSI0TX,SSI1TX,SSI2TX - SSIF0,1,2 Direct Transmission - Set parameter
b11:9  SSI0RX,SSI1RX,SSI2RX - SSIF0,1,2 Direct Reception - Set parameter
b1     SSI012TEN - SSIF012 Transmission Enable - Transmission disabled
b0     SSI012REN - SSIF012 Reception Enable - Reception disabled
Other Reserved - The write value should always be 0 */
#define SSICTRL_SSI3TX_EN	(1<<30)
#define SSICTRL_SSI4TX_EN	(1<<29)
#define SSICTRL_SSI5TX_EN	(1<<28)
#define SSICTRL_SSI3RX_EN	(1<<26)
#define SSICTRL_SSI4RX_EN	(1<<25)
#define SSICTRL_SSI5RX_EN	(1<<24)
#define SSICTRL_SSI345TEN	(1<<17)
#define SSICTRL_SSI345REN	(1<<16)
#define SSICTRL_SSI0TX_EN	(1<<14)
#define SSICTRL_SSI1TX_EN	(1<<13)
#define SSICTRL_SSI2TX_EN	(1<<12)
#define SSICTRL_SSI0RX_EN	(1<<10)
#define SSICTRL_SSI1RX_EN	(1<<9)
#define SSICTRL_SSI2RX_EN	(1<<8)
#define SSICTRL_SSI012TEN	(1<<1)
#define SSICTRL_SSI012REN	(1)

/* SRCRSEL Register Value */
/* RSEL_INIT_VALUE:Initial value of SRCRSELn_CIM and MIXRSEL_CIM Register */
#define RSEL_INIT_VALUE		(0x76543210uL)
#define SRCRSEL_PLACE0		(7)
#define SRCRSEL_PLACE1		(7<<4)
#define SRCRSEL_PLACE2		(7<<8)
#define SRCRSEL_PLACE3		(7<<12)
#define SRCRSEL_PLACE4		(7<<16)
#define SRCRSEL_PLACE5		(7<<20)
#define SRCRSEL_PLACE6		(7<<24)
#define SRCRSEL_PLACE7		(7<<28)

/* MIXRSEL Register Value */
#define MIXRSEL_INIT_VALUE	(0x76543210uL)
#define MIXRSEL_PLACE0		(7)
#define MIXRSEL_PLACE1		(7<<4)
#define MIXRSEL_PLACE2		(7<<8)
#define MIXRSEL_PLACE3		(7<<12)
#define MIXRSEL_PLACE4		(7<<16)
#define MIXRSEL_PLACE5		(7<<20)
#define MIXRSEL_PLACE6		(7<<24)
#define MIXRSEL_PLACE7		(7<<28)

/* MIXBR Register Value */
#define MIXBR_BYPASS_OFF	(0uL)
#define MIXBR_BYPASS_ON		(1uL)
#define MIXBR_BPSYS_B		(1<<16)

/* CHCTRL Register */
#define DMA_CHCTRL_SWRST_BIT	(0x00000008uL)
#define DMA_CHCTRL_SETEN_BIT	(0x00000001uL)

/************************************************************************
	SCUX Register Structure
************************************************************************/
struct scux_ipc_regs {
	u32	ipcir;
	u32	ipslr;
	unsigned char  dummy_ipc[248];
};

struct scux_opc_regs {
	u32	opcir;
	u32	opslr;
	unsigned char  dummy_opc[248];
};

struct scux_ffd_regs {
	u32	ffdir;
	u32	fdair;
	u32	drqsr;
	u32	ffdpr;
	u32	ffdbr;
	u32	devmr;
	unsigned char  dummy_ffd[4];
	u32	devcr;
	unsigned char  dummy2_ffd[224];
};

struct scux_ffu_regs {
	u32	ffuir;
	u32	fuair;
	u32	urqsr;
	u32	ffupr;
	u32	uevmr;
	unsigned char  dummy_ffu[4];
	u32	uevcr;
	unsigned char  dummy2_ffu[228];
};

struct scux_src_regs {
	u32	srcir0;
	u32	sadir0;
	u32	srcbr0;
	u32	ifscr0;
	u32	ifsvr0;
	u32	srccr0;
	u32	mnfsr0;
	u32	bfssr0;
	u32	sc2sr0;
	u32	watsr0;
	u32	sevmr0;
	unsigned char  dummy_src0[4];
	u32	sevcr0;
	u32	srcir1;
	u32	sadir1;
	u32	srcbr1;
	u32	ifscr1;
	u32	ifsvr1;
	u32	srccr1;
	u32	mnfsr1;
	u32	bfssr1;
	u32	sc2sr1;
	u32	watsr1;
	u32	sevmr1;
	unsigned char  dummy_src1[4];
	u32	sevcr1;
	u32	srcirr1;
	unsigned char  dummy_src[148];
};

struct scux_dvu_regs {
	u32	dvuir;
	u32	vadir;
	u32	dvubr;
	u32	dvucr;
	u32	zcmcr;
	u32	vrctr;
	u32	vrpdr;
	u32	vrdbr;
	u32	vrwtr;
	u32	vol0r;
	u32	vol1r;
	u32	vol2r;
	u32	vol3r;
	u32	vol4r;
	u32	vol5r;
	u32	vol6r;
	u32	vol7r;
	u32	dvuer;
	u32	dvusr;
	u32	vevmr;
	unsigned char  dummy_dvu[4];
	u32	vevcr;
	unsigned char  dummy2_dvu[168];
};

struct scux_mix_regs {
	u32	mixir;
	u32	madir;
	u32	mixbr;
	u32	mixmr;
	u32	mvpdr;
	u32	mdbar;
	u32	mdbbr;
	u32	mdbcr;
	u32	mdbdr;
	u32	mdber;
	u32	mixsr;
	unsigned char  dummy_mix[212];
};
struct scux_cim_regs {
	u32 swrsr;		/*  SWRSR_CIM       */
	u32 dmacr_cim;		/*  DMACR_CIM       */
	union {			/*                  */
		/* DMATD0_CIM       */
		u32 UINT32;	/*  32-bit Access   */
		u16 UINT16[2];	/*  16-bit Access   */
	} dmatd0_cim;		/*                  */

	union {			/*                  */
		/* DMATD1_CIM       */
		u32 UINT32;	/*  32-bit Access   */
		u16 UINT16[2];	/*  16-bit Access   */
	} dmatd1_cim;		/*                  */

	union {			/*                  */
		/* DMATD2_CIM       */
		u32 UINT32;	/*  32-bit Access   */
		u16 UINT16[2];	/*  16-bit Access   */
	} dmatd2_cim;		/*                  */

	union {			/*                  */
		/* DMATD3_CIM       */
		u32 UINT32;	/*  32-bit Access   */
		u16 UINT16[2];	/*  16-bit Access   */
	} dmatd3_cim;		/*                  */

	union {			/*                  */
		/* DMATU0_CIM       */
		u32 UINT32;	/*  32-bit Access   */
		u16 UINT16[2];	/*  16-bit Access   */
	} dmatu0_cim;		/*                  */

	union {			/*                  */
		/* DMATU1_CIM       */
		u32 UINT32;	/*  32-bit Access   */
		u16 UINT16[2];	/*  16-bit Access   */
	} dmatu1_cim;		/*                  */

	union {			/*                  */
		/* DMATU2_CIM       */
		u32 UINT32;	/*  32-bit Access   */
		u16 UINT16[2];	/*  16-bit Access   */
	} dmatu2_cim;		/*                  */

	union {			/*                  */
		/* DMATU3_CIM       */
		u32 UINT32;	/*  32-bit Access   */
		u16 UINT16[2];	/*  16-bit Access   */
	} dmatu3_cim;		/*                  */
	unsigned char  dummy_cim[16];	/*                  */
	u32 ssirsel_cim;	/*  SSIRSEL_CIM     */
	u32 fdtsel0_cim;	/*  FDTSEL0_CIM     */
	u32 fdtsel1_cim;	/*  FDTSEL1_CIM     */
	u32 fdtsel2_cim;	/*  FDTSEL2_CIM     */
	u32 fdtsel3_cim;	/*  FDTSEL3_CIM     */
	u32 futsel0_cim;	/*  FUTSEL0_CIM     */
	u32 futsel1_cim;	/*  FUTSEL1_CIM     */
	u32 futsel2_cim;	/*  FUTSEL2_CIM     */
	u32 futsel3_cim;	/*  FUTSEL3_CIM     */
	u32 ssipmd_cim;		/*  SSIPMD_CIM      */
	u32 ssictrl_cim;	/*  SSICTRL_CIM     */
	u32 srcrsel0_cim;	/*  SRCRSEL0_CIM    */
	u32 srcrsel1_cim;	/*  SRCRSEL1_CIM    */
	u32 srcrsel2_cim;	/*  SRCRSEL2_CIM    */
	u32 srcrsel3_cim;	/*  SRCRSEL3_CIM    */
	u32 mixrsel_cim;	/*  MIXRSEL_CIM     */
};

struct ssif_regs {
	u32	ssicr;
	u32	ssisr;
	unsigned char dummy_ssifreg1[8];
	u32	ssifcr;
	u32	ssifsr;
	u32	ssiftdr;
	u32	ssifrdr;
	u32	ssitdmr;
	u32	ssifccr;
	u32	ssifcmr;
	u32	ssifcsr;
	unsigned char dummy_ssifreg2[2000];
};

struct dmac_regs {
	u32	chctrl;
};

struct scux_reg_info {
	void __iomem				*scuxreg;
	struct scux_ipc_regs		*ipcreg[NUM_SCUX_CH];
	struct scux_opc_regs		*opcreg[NUM_SCUX_CH];
	struct scux_ffd_regs		*ffdreg[NUM_SCUX_CH];
	struct scux_ffu_regs		*ffureg[NUM_SCUX_CH];
	struct scux_src_regs		*srcreg[MAXCH_SRC];
	struct scux_dvu_regs		*dvureg[MAXCH_DVU];
	struct scux_mix_regs		*mixreg;
	struct scux_cim_regs		*cimreg;
	void __iomem				*ssireg;
	struct ssif_regs			*ssifreg[MAXCH_SSIF];
};

struct scu_audio_info {
	struct scux_reg_info reginfo;
	struct scu_route_info routeinfo;
	struct scu_clock_info clockinfo;
	spinlock_t scu_lock;		/* for common register */
	unsigned int rate[2];
	unsigned int volume[2][2];
	unsigned int mute[2];
};

/******************************************************************************
 ==== SSIF ====
******************************************************************************/
/* SSICRn bit */
#define	SSICR_REN_DIS	(0<<0)
#define	SSICR_REN_EN	(1<<0)
#define	SSICR_TEN_DIS	(0<<1)
#define	SSICR_TEN_EN	(1<<1)
#define	SSICR_MUEN	(1<<3)
#define	SSICR_CKDV_1	(0<<4)
#define	SSICR_CKDV_2	(1<<4)
#define	SSICR_CKDV_4	(2<<4)
#define	SSICR_CKDV_8	(3<<4)
#define	SSICR_CKDV_16	(4<<4)
#define	SSICR_CKDV_32	(5<<4)
#define	SSICR_CKDV_64	(6<<4)
#define	SSICR_CKDV_128	(7<<4)
#define	SSICR_CKDV_6	(8<<4)
#define	SSICR_CKDV_12	(9<<4)
#define	SSICR_CKDV_24	(10<<4)
#define	SSICR_CKDV_48	(11<<4)
#define	SSICR_CKDV_96	(12<<4)
#define	SSICR_DEL_1CLKDEL (0<<8)
#define	SSICR_DEL_NODEL (1<<8)
#define	SSICR_PDTA	(1<<9)
#define	SSICR_SDTA	(1<<10)
#define	SSICR_SPDP	(1<<11)
#define	SSICR_SWSP	(1<<12)
#define	SSICR_SCKP	(1<<13)
#define	SSICR_SWSD_SLAVE (0<<14)
#define	SSICR_SWSD_MASTER (1<<14)
#define	SSICR_M_MASTER	(3<<14)
#define	SSICR_SCKD_SLAVE (0<<15)
#define	SSICR_SCKD_MASTER (1<<15)
#define	SSICR_SWL_ST8	(0<<16)
#define	SSICR_SWL_ST16	(1<<16)
#define	SSICR_SWL_ST24	(2<<16)
#define	SSICR_SWL_ST32	(3<<16)
#define	SSICR_SWL_ST48	(4<<16)
#define	SSICR_SWL_ST64	(5<<16)
#define	SSICR_SWL_ST128	(6<<16)
#define	SSICR_SWL_ST256	(7<<16)
#define	SSICR_DWL_ST8	(0<<19)
#define	SSICR_DWL_ST16	(1<<19)
#define	SSICR_DWL_ST18	(2<<19)
#define	SSICR_DWL_ST20	(3<<19)
#define	SSICR_DWL_ST22	(4<<19)
#define	SSICR_DWL_ST24	(5<<19)
#define	SSICR_DWL_ST32	(6<<19)
#define	SSICR_CHNL_ST1	(0<<22)
#define	SSICR_CHNL_ST2	(1<<22)
#define	SSICR_CHNL_ST3	(2<<22)
#define	SSICR_CHNL_ST4	(3<<22)
#define	SSICR_IIEN	(1<<25)
#define	SSICR_ROIEN	(1<<26)
#define	SSICR_RUIEN	(1<<27)
#define	SSICR_TOIEN	(1<<28)
#define	SSICR_TUIEN	(1<<29)
#define	SSICR_CKS	(1<<30)


/* for SSI start */
#define	SSICR_ENABLE	(SSICR_REN_EN	| \
			 SSICR_ROIEN	| \
			 SSICR_RUIEN	| \
			 SSICR_TOIEN)

/* SSISRn bit */
#define	SSISR_IDST	(1<<0)
#define	SSISR_SWNO	(1<<1)
#define	SSISR_CHNO0	(1<<2)
#define	SSISR_CHNO1	(1<<3)
#define	SSISR_DIRQ	(1<<24)
#define	SSISR_IIRQ	(1<<25)
#define	SSISR_OIRQ	(1<<26)
#define	SSISR_UIRQ	(1<<27)
#define	SSISR_DMRQ	(1<<28)
#define	SSISR_ROIRQ	(0<<26)
#define	SSISR_RUIRQ	(0<<27)
#define	SSISR_TOIRQ	(0<<28)
#define	SSISR_TUIRQ	(0<<29)

/* SSIFCRn bit */
#define SSIFCR_RFRST	(1<<0)
#define SSIFCR_TFRST	(1<<1)
#define SSIFCR_RIE	(1<<2)
#define SSIFCR_TIE	(1<<3)

/* SSI TDM Mode Register Setting */
/*  [17] RXDMUTE  : B'0 : Rx Direct Data MUTE : Rx Direct Data  */
/*  [8] CONT  : B'1 : WS Continue Mode:Enables WS continue mode */
/*  [0] TDM      : B'0 : TDM Mode : Disables TDM mode           */
#define	SSITDMR_TDM_DIS			(0<<0)
#define	SSITDMR_TDM_EN			(1<<0)
#define	SSITDMR_CONT_DIS		(0<<8)
#define	SSITDMR_CONT_EN			(1<<8)
#define	SSITDMR_RXDMUTE_RCVDATA		(0<<17)
#define	SSITDMR_RXDMUTE_0DATA		(1<<17)
#define SSI_SSITDMR_BASE_INIT_VALUE	(SSITDMR_RXDMUTE_RCVDATA | \
						SSITDMR_CONT_EN | \
						SSITDMR_TDM_DIS)

/* ==== SSI Channel0 ==== */
/* SSI0 Control Register Setting */
#define SSI_SSICR0_CKS_VALUE		(0x00000000uL)
#define SSI_SSICR0_CHNL_VALUE		(0x00000000uL)
#define SSI_SSICR0_DWL_VALUE		(0x00080000uL)
#define SSI_SSICR0_SWL_VALUE		(0x00030000uL)
#define SSI_SSICR0_SCKD_VALUE		(0x00008000uL)
#define SSI_SSICR0_SWSD_VALUE		(0x00004000uL)
#define SSI_SSICR0_SCKP_VALUE		(0x00000000uL)
#define SSI_SSICR0_SWSP_VALUE		(0x00000000uL)
#define SSI_SSICR0_SPDP_VALUE		(0x00000000uL)
#define SSI_SSICR0_SDTA_VALUE		(0x00000000uL)
#define SSI_SSICR0_PDTA_VALUE		(0x00000000uL)
#define SSI_SSICR0_DEL_VALUE		(0x00000000uL)
#define SSI_SSICR0_CKDV_VALUE		(0x00000030uL)

/*  [30]    CKS : B'0  : AUDIO_X1 input                                      */
/*  [23:22] CHNL: B'00 : 1 channel / system word                             */
/*  [21:19] DWL : B'001: 16 bit / data word                                  */
/*  [18:16] SWL : B'011: 32 bit / system word                                */
/*  [15]    SCKD: B'1  : Serial Bit Clock Direction:master mode              */
/*  [14]    SWSD: B'1  : Serial WS Direction:master mode                     */
/*  [13]    SCKP: B'0  : SSIWS and SSIDATA change at the SSISCK falling edge */
/*  [12]    SWSP: B'0  : SSIWS is low for 1st channel, high for 2nd channel  */
/*  [11]    SPDP: B'0  : Padding bits are low                                */
/*  [10]    SDTA: B'0  : Tx and Rx in the order of serial data and padding bit*/
/*  [9]     PDTA: B'0  : The lower bits of parallel data(SSITDR, SSIRDR)
				     are transferred prior to the upper bits */
/*  [8]     DEL : B'0  : 1 clock cycle delay between SSIWS and SSIDATA       */
/*  [7:4]   CKDV: B'0011: AUDIO dia / 8
			(64FS,AUDIO_X1@22.5792MHz/32bit system word) */
#define SSI_SSICR0_USER_INIT_VALUE  (SSI_SSICR0_CKS_VALUE  | \
					SSI_SSICR0_CHNL_VALUE | \
					SSI_SSICR0_DWL_VALUE  | \
					SSI_SSICR0_SWL_VALUE  | \
					SSI_SSICR0_SCKD_VALUE | \
					SSI_SSICR0_SWSD_VALUE | \
					SSI_SSICR0_SCKP_VALUE | \
					SSI_SSICR0_SWSP_VALUE | \
					SSI_SSICR0_SPDP_VALUE | \
					SSI_SSICR0_SDTA_VALUE | \
					SSI_SSICR0_PDTA_VALUE | \
					SSI_SSICR0_DEL_VALUE  | \
					SSI_SSICR0_CKDV_VALUE)

/* SSI SSICR0 Register INIT Setting */
/*  [29]    TUIEN   : B'0    : Disables an underflow interrupt              */
/*  [28]    TOIEN   : B'0    : Disables an overflow interrupt               */
/*  [27]    RUIEN   : B'0    : Disables an underflow interrupt              */
/*  [26]    ROIEN   : B'0    : Receive Overflow Interrupt Enable            */
/*  [25]    IIEN    : B'0    : Disables an idle mode interrupt              */
/*  [3]     MUEN    : B'0    : This module is not muted                     */
/*  [1]     TEN     : B'0    : Disables the transmit operation              */
/*  [0]     REN     : B'0    : Disables the receive operation               */
/*  [30:4] SSI_SSICR0_USER_INIT_VALUE SSICR0 INIT                          */
#define SSI_SSICR_TUIEN_INIT_VALUE          (0x00000000uL)
#define SSI_SSICR_TOIEN_INIT_VALUE          (0x00000000uL)
#define SSI_SSICR_RUIEN_INIT_VALUE          (0x00000000uL)
#define SSI_SSICR_ROIEN_INIT_VALUE          (0x00000000uL)
#define SSI_SSICR_IIEN_INIT_VALUE           (0x00000000uL)
#define SSI_SSICR_MUEN_INIT_VALUE           (0x00000000uL)
#define SSI_SSICR_TEN_INIT_VALUE            (0x00000000uL)
#define SSI_SSICR_REN_INIT_VALUE            (0x00000000uL)
#define SSI_SSICR_BASE_INIT_VALUE           (SSI_SSICR_TUIEN_INIT_VALUE | \
						SSI_SSICR_TOIEN_INIT_VALUE | \
						SSI_SSICR_RUIEN_INIT_VALUE | \
						SSI_SSICR_ROIEN_INIT_VALUE | \
						SSI_SSICR_IIEN_INIT_VALUE  | \
						SSI_SSICR_MUEN_INIT_VALUE  | \
						SSI_SSICR_TEN_INIT_VALUE   | \
						SSI_SSICR_REN_INIT_VALUE   | \
						SSI_SSICR0_USER_INIT_VALUE)
/*
 * SSICR setting for WM8978
 *   playback, master, 16bit, stereo
 *   SCLK=256fs(MCLK)/8=32fs
 */
#define	SSICR_PLAY_WM8978_ST	(SSICR_CHNL_ST1    | \
			 SSICR_DWL_ST16 | \
			 SSICR_SWL_ST32 | \
			 SSICR_M_MASTER | \
			 SSICR_CKDV_8)
/*
 * SSICR setting for WWM8978
 *   capture, slave, 16bit, stereo
 *   SCLK=256fs(MCLK)/8=32fs
 */
#define	SSICR_CAP_WM8978_ST	(SSICR_CHNL_ST1    | \
			 SSICR_DWL_ST16 | \
			 SSICR_SWL_ST32 | \
			 SSICR_M_MASTER | \
			 SSICR_CKDV_8)
/* SSI0 FIFO Control Register Setting */
/*  [7:6] TTRG  : B'00 : Transmit Data Trigger Number:7                       */
/*  [5:4] RTRG  : B'00 : Receive Data Trigger Number:1                        */
/*  [3]   TIE   : B'0  : Transmit data empty interrupt (TXI)
			 request is disabled */
/*  [2]   RIE   : B'0  : Receive data full interrupt (RXI)
			 request is disabled   */
/*  [1]   TFRST : B'1  : Reset is enabled                                     */
/*  [0]   RFRST : B'1  : Reset is enabled                                     */
#define SSI_SSIFCR0_TTRG_VALUE			(0x00000000uL)
#define SSI_SSIFCR0_RTRG_VALUE			(0x00000000uL)
#define SSI_SSIFCR_TIE_INIT_VALUE		(0x00000000uL)
#define SSI_SSIFCR_RIE_INIT_VALUE		(0x00000000uL)
#define SSI_SSIFCR_TFRST_INIT_VALUE		(0x00000002uL)
#define SSI_SSIFCR_RFRST_INIT_VALUE		(0x00000001uL)

#define SSI_SSIFCR_BASE_INIT_VALUE	(SSI_SSIFCR0_TTRG_VALUE     | \
					SSI_SSIFCR0_RTRG_VALUE      | \
					SSI_SSIFCR_TIE_INIT_VALUE   | \
					SSI_SSIFCR_RIE_INIT_VALUE   | \
					SSI_SSIFCR_TFRST_INIT_VALUE | \
					SSI_SSIFCR_RFRST_INIT_VALUE)

/******************************************************************************
 ==== SWRSTCR1 ====
******************************************************************************/
#define	SWRSTCR1	0xFCFE0460	/* SWRSTCR1 Address */
#define	SWRSTCR1_INIT	(0x00)		/* Initialize */
#define	SWRSTCR1_SRST11	(1<<1)		/* SSIF5SRST */
#define	SWRSTCR1_SRST12 (1<<2)		/* SSIF4SRST */
#define	SWRSTCR1_SRST13 (1<<3)		/* SSIF3SRST */
#define	SWRSTCR1_SRST14 (1<<4)		/* SSIF2SRST */
#define	SWRSTCR1_SRST15 (1<<5)		/* SSIF1SRST */
#define	SWRSTCR1_SRST16 (1<<6)		/* SSIF0SRST */
#define	SWRSTCR1_AXTALE (1<<7)		/* AXTALE    */

/************************************************************************
	external prototype declaration
************************************************************************/
extern struct snd_soc_platform_driver scu_platform;
extern struct scu_route_info *scu_get_route_info(void);
extern struct scu_platform_data *scu_get_platform_data(void);
extern void scu_init_ssi(int, int, int, int, int);
extern void scu_init_src(int, unsigned int, unsigned int);
extern void scu_init_dvc(int);
extern void scu_deinit_ssi(int, int, int, int);
extern void scu_deinit_src(int);
extern void scu_deinit_dvc(int);
extern int scu_check_route(int dir, struct scu_route_info *routeinfo);
extern int scu_dai_add_control(struct snd_card *card);

#endif /* SH_SCUX_H */
