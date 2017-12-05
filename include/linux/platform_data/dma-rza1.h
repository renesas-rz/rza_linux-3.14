/*
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_RZA1_DMA_H__
#define __ASM_ARCH_RZA1_DMA_H__

#include <linux/scatterlist.h>
#include <linux/device.h>
#include <linux/dmaengine.h>

/* DMA slave IDs */
enum {
	RZA1DMA_SLAVE_PCM_MEM_SSI0 = 1,	/* DMA0		MEM->(DMA0)->SSI0 */
	RZA1DMA_SLAVE_PCM_MEM_SRC1,		/* DMA1		MEM->(DMA1)->FFD0_1->SRC1->SSI0 */
	RZA1DMA_SLAVE_PCM_SSI0_MEM,		/* DMA2		SSI0->(DMA2)->MEM */
	RZA1DMA_SLAVE_PCM_SRC0_MEM,		/* DMA3		SSI0->SRC0->FFU0_0->(DMA3)->MEM */
	RZA1DMA_SLAVE_PCM_MAX,
	RZA1DMA_SLAVE_SDHI0_TX,
	RZA1DMA_SLAVE_SDHI0_RX,
	RZA1DMA_SLAVE_SDHI1_TX,
	RZA1DMA_SLAVE_SDHI1_RX,
	RZA1DMA_SLAVE_MMCIF_TX,
	RZA1DMA_SLAVE_MMCIF_RX,
};

union chcfg_reg {
	u32	v;
	struct {
		u32 sel:  3;	/* LSB */
		u32 reqd: 1;
		u32 loen: 1;
		u32 hien: 1;
		u32 lvl:  1;
		u32 _mbz0:1;
		u32 am:   3;
		u32 _mbz1:1;
		u32 sds:  4;
		u32 dds:  4;
		u32 _mbz2:2;
		u32 tm:   1;
		u32 _mbz3:9;
	};
};

union dmars_reg {
	u32 v;
	struct {
		u32 rid:   2;	/* LSB */
		u32 mid:   7;
		u32 _mbz0:23;
	};
};

/*
 * Drivers, using this library are expected to embed struct shdma_dev,
 * struct shdma_chan, struct shdma_desc, and struct shdma_slave
 * in their respective device, channel, descriptor and slave objects.
 */

struct rza1dma_slave {
	int slave_id;
};

/* Used by slave DMA clients to request DMA to/from a specific peripheral */
struct rza1_dma_slave {
	struct rza1dma_slave	rza1dma_slaveid;	/* Set by the platform */
};

struct rza1_dma_slave_config {
	int			slave_id;
	dma_addr_t		addr;
	union chcfg_reg		chcfg;
	union dmars_reg		dmars;
};

struct rza1_dma_pdata {
	const struct rza1_dma_slave_config *slave;
	int slave_num;
	int channel_num;
};

bool rza1dma_chan_filter(struct dma_chan *chan, void *arg);
#endif
