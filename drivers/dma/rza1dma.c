/*
 * Renesas RZA1 DMA Engine support
 *
 * base is drivers/dma/imx-dma.c
 *
 * Copyright (C) 2009-2013 Renesas Solutions Corp.
 * Copyright 2012 Javier Martin, Vista Silicon <javier.martin@vista-silicon.com>
 * Copyright (C) 2011-2012 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * Copyright 2010 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 * Copyright (C) 2009 Nobuhiro Iwamatsu <iwamatsu.nobuhiro@renesas.com>
 * Copyright (C) 2007 Freescale Semiconductor, Inc. All rights reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/module.h>

#include <asm/irq.h>
#include <linux/platform_data/dma-rza1.h>

#include "dmaengine.h"
#define RZA1DMA_MAX_CHAN_DESCRIPTORS	16

/* set the offset of regs */
#define	CHSTAT	0x0024
#define CHCTRL	0x0028
#define	CHCFG	0x002c
#define	CHITVL	0x0030
#define	CHEXT	0x0034
#define NXLA	0x0038
#define	CRLA	0x003c

#define	DCTRL		0x0000
#define	DSTAT_EN	0x0010
#define	DSTAT_ER	0x0014
#define	DSTAT_END	0x0018
#define	DSTAT_TC	0x001c
#define	DSTAT_SUS	0x0020

#define	EACH_CHANNEL_OFFSET		0x0040
#define	CHANNEL_0_7_OFFSET		0x0000
#define	CHANNEL_0_7_COMMON_BASE		0x0300
#define CHANNEL_8_15_OFFSET		0x0400
#define	CHANNEL_8_15_COMMON_BASE	0x0700

/* set bit filds */
/* CHSTAT */
#define	CHSTAT_TC	(0x1 << 6)
#define CHSTAT_END	(0x1 << 5)
#define CHSTAT_ER	(0x1 << 4)

/* CHCTRL */
#define CHCTRL_CLRINTMSK	(0x1 << 17)
#define	CHCTRL_SETINTMSK	(0x1 << 16)
#define	CHCTRL_CLRSUS		(0x1 << 9)
#define	CHCTRL_SETSUS		(0x1 << 8)
#define CHCTRL_CLRTC		(0x1 << 6)
#define	CHCTRL_CLREND		(0x1 << 5)
#define	CHCTRL_CLRRQ		(0x1 << 4)
#define	CHCTRL_SWRST		(0x1 << 3)
#define	CHCTRL_STG		(0x1 << 2)
#define	CHCTRL_CLREN		(0x1 << 1)
#define	CHCTRL_SETEN		(0x1 << 0)
#define	CHCTRL_DEFAULT	(CHCTRL_CLRINTMSK | \
			CHCTRL_CLRSUS | \
			CHCTRL_CLRTC | \
			CHCTRL_CLREND | \
			CHCTRL_CLRRQ | \
			CHCTRL_SWRST | \
			CHCTRL_CLREN)

/* CHCFG */
#define	CHCFG_DMS		(0x1 << 31)
#define	CHCFG_DEM		(0x1 << 24)
#define	CHCFG_TM(bit)		(bit << 22)
#define	CHCFG_DAD		(0x1 << 21)
#define	CHCFG_SAD		(0x1 << 20)
#define	CHCFG_8BIT	(0x00)
#define	CHCFG_16BIT	(0x01)
#define	CHCFG_32BIT	(0x02)
#define	CHCFG_64BIT	(0x03)
#define CHCFG_128BIT	(0x04)
#define	CHCFG_256BIT	(0x05)
#define	CHCFG_512BIT	(0x06)
#define	CHCFG_1024BIT	(0x07)
#define	CHCFG_DDS(bit)		(bit << 16)
#define	CHCFG_SDS(bit)		(bit << 12)
#define	CHCFG_AM(bits)		(bits << 8)
#define	CHCFG_LVL(bit)		(bit << 6)
#define	CHCFG_HIEN(bit)		(bit << 5)
#define	CHCFG_LOEN(bit)		(bit << 4)
#define	CHCFG_REQD(bit)		(bit << 3)
#define	CHCFG_SEL(bits)		((bits & 0x07) << 0)

/* DCTRL */
#define	DCTRL_LVINT		(0x1 << 1)
#define	DCTRL_PR		(0x1 << 0)
#define DCTRL_DEFAULT		(DCTRL_LVINT | DCTRL_PR)

/* DMARS */
#define	DMARS_RID(bit)		(bit << 0)
#define	DMARS_MID(bit)		(bit << 2)

/* LINK MODE DESCRIPTOR */
#define	HEADER_DIM	(0x1 << 3)
#define	HEADER_WBD	(0x1 << 2)
#define	HEADER_LE	(0x1 << 1)
#define	HEADER_LV	(0x1 << 0)

#define to_rza1dma_chan(c) container_of(c, struct rza1dma_channel, chan)

enum  rza1dma_prep_type {
	RZA1DMA_DESC_MEMCPY,
	RZA1DMA_DESC_SLAVE_SG,
};

struct format_desc {
	u32	header;
	u32	src_addr;
	u32	dst_addr;
	u32	trs_byte;
	u32	config;
	u32	interval;
	u32	extension;
	u32	next_lk_addr;
};

#define	FORMAT_DESC_NUM	64

struct rza1dma_desc {
	struct list_head		node;
	struct dma_async_tx_descriptor	desc;
	enum dma_status			status;
	dma_addr_t			src;
	dma_addr_t			dest;
	size_t				len;
	enum dma_transfer_direction	direction;
	enum rza1dma_prep_type		type;
	/* For memcpy */
	unsigned int			config_port;
	unsigned int			config_mem;
	/* For slave sg */
	struct scatterlist		*sg;
	unsigned int			sgcount;
};

struct rza1dma_channel {
	struct rza1dma_engine		*rza1dma;
	unsigned int			channel;

	struct tasklet_struct		dma_tasklet;
	struct list_head		ld_free;
	struct list_head		ld_queue;
	struct list_head		ld_active;
	int				descs_allocated;
	enum dma_slave_buswidth		word_size;
	dma_addr_t			per_address;
	struct dma_chan			chan;
	struct dma_async_tx_descriptor	desc;
	enum dma_status			status;

	const struct rza1_dma_slave_config	*slave;
	void __iomem			*ch_base;
	void __iomem			*ch_cmn_base;
	struct format_desc		*desc_base;
	dma_addr_t			desc_base_dma;

	u32	chcfg;
	u32	chctrl;
};

struct rza1dma_engine {
	struct device			*dev;
	struct device_dma_parameters	dma_parms;
	struct dma_device		dma_device;
	void __iomem			*base;
	void __iomem			*ext_base;
	spinlock_t			lock;
	struct rza1dma_channel		*channel;
	struct rza1_dma_pdata		*pdata;
};

static void rza1dma_writel(struct rza1dma_engine *rza1dma, unsigned val,
				unsigned offset)
{
	__raw_writel(val, rza1dma->base + offset);
}

static void rza1dma_ext_writel(struct rza1dma_engine *rza1dma, unsigned val,
				unsigned offset)
{
	__raw_writel(val, rza1dma->ext_base + offset);
}

static u32 rza1dma_ext_readl(struct rza1dma_engine *rza1dma, unsigned offset)
{
	return __raw_readl(rza1dma->ext_base + offset);
}

static void rza1dma_ch_writel(struct rza1dma_channel *rza1dmac, unsigned val,
				unsigned offset, int which)
{
	if (which)
		__raw_writel(val, rza1dmac->ch_base + offset);
	else
		__raw_writel(val, rza1dmac->ch_cmn_base + offset);
}

static u32 rza1dma_ch_readl(struct rza1dma_channel *rza1dmac, unsigned offset,
				int which)
{
	if (which)
		return __raw_readl(rza1dmac->ch_base + offset);
	else
		return __raw_readl(rza1dmac->ch_cmn_base + offset);
}

static void rza1dma_enable_hw(struct rza1dma_desc *d)
{
	struct dma_chan *chan = d->desc.chan;
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	int channel = rza1dmac->channel;
	unsigned long flags;
	u32 nxla = rza1dmac->desc_base_dma;
	u32 chcfg = rza1dmac->chcfg;
	u32 chctrl = rza1dmac->chctrl;

	dev_dbg(rza1dma->dev, "%s channel %d\n", __func__, channel);

	local_irq_save(flags);


	if(chctrl & CHCTRL_SETEN){			/* When [SETEN]is "0".already before process add Descriptor */
								/* Only add Descriptor case.skip write register */
		rza1dma_ch_writel(rza1dmac, nxla, NXLA, 1);		/* NXLA reg */
		rza1dma_ch_writel(rza1dmac, chcfg, CHCFG, 1);		/* CHCFG reg */

		rza1dma_ch_writel(rza1dmac, CHCTRL_SWRST, CHCTRL, 1);	/* CHCTRL reg */
		rza1dma_ch_writel(rza1dmac, chctrl, CHCTRL, 1);		/* CHCTRL reg */
	}

	local_irq_restore(flags);
}

static void rza1dma_disable_hw(struct rza1dma_channel *rza1dmac)
{
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	int channel = rza1dmac->channel;
	unsigned long flags;

	dev_dbg(rza1dma->dev, "%s channel %d\n", __func__, channel);

	local_irq_save(flags);
	rza1dma_ch_writel(rza1dmac, CHCTRL_DEFAULT, CHCTRL, 1); /* CHCTRL reg */
	local_irq_restore(flags);
}

static void dma_irq_handle_channel(struct rza1dma_channel *rza1dmac)
{
	u32 chstat, chctrl;
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	int channel = rza1dmac->channel;

	chstat = rza1dma_ch_readl(rza1dmac, CHSTAT, 1);
	if (chstat & CHSTAT_ER) {
		dev_err(rza1dma->dev, "RZA1 DMAC error ocurred\n");
		dev_err(rza1dma->dev, "CHSTAT_%d = %08X\n", channel, chstat);
		rza1dma_ch_writel(rza1dmac,
				CHCTRL_DEFAULT,
				CHCTRL, 1);
		goto schedule;
	}
	if (!(chstat & CHSTAT_END))
		return;

	chctrl = rza1dma_ch_readl(rza1dmac, CHCTRL, 1);
	rza1dma_ch_writel(rza1dmac,
			chctrl | CHCTRL_CLREND | CHCTRL_CLRRQ,
			CHCTRL, 1);
schedule:
	/* Tasklet irq */
	tasklet_schedule(&rza1dmac->dma_tasklet);
}

static irqreturn_t rza1dma_irq_handler(int irq, void *dev_id)
{
	struct rza1dma_engine *rza1dma = dev_id;
	struct rza1_dma_pdata *pdata = rza1dma->pdata;
	int i, channel_num = pdata->channel_num;

	dev_dbg(rza1dma->dev, "%s called\n", __func__);

	for (i = 0; i < channel_num; i++)
		dma_irq_handle_channel(&rza1dma->channel[i]);

	return IRQ_HANDLED;
}

static void set_dmars_register(struct rza1dma_engine *rza1dma,
				int channel, u32 dmars)
{
	u32 dmars_offset = (channel / 2) * 4;
	u32 dmars32;

	dmars32 = rza1dma_ext_readl(rza1dma, dmars_offset);
	if (channel % 2) {
		dmars32 &= 0x0000ffff;
		dmars32 |= dmars << 16;
	} else {
		dmars32 &= 0xffff0000;
		dmars32 |= dmars;
	}
	rza1dma_ext_writel(rza1dma, dmars32, dmars_offset);
	dmars32 = rza1dma_ext_readl(rza1dma, dmars_offset);
}

static void prepare_desc_for_memcpy(struct rza1dma_desc *d)
{
	struct dma_chan *chan = d->desc.chan;
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	struct format_desc *descs = rza1dmac->desc_base;
	int channel = rza1dmac->channel;
	u32 chcfg = 0x80400008;
	u32 dmars = 0;

	dev_dbg(rza1dma->dev, "%s called\n", __func__);

	/* prepare descriptor */
	descs[0].src_addr = d->src;
	descs[0].dst_addr = d->dest;
	descs[0].trs_byte = d->len;
	descs[0].config = chcfg;
	descs[0].interval = 0;
	descs[0].extension = 0;
	descs[0].header = HEADER_LV | HEADER_LE;
	descs[0].next_lk_addr = 0;

	/* and set DMARS register */
	set_dmars_register(rza1dma, channel, dmars);

	rza1dmac->chcfg = chcfg;
	rza1dmac->chctrl = CHCTRL_STG | CHCTRL_SETEN;
}

static void prepare_descs_for_slave_sg(struct rza1dma_desc *d)
{
	struct dma_chan *chan = d->desc.chan;
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	struct format_desc *descs = rza1dmac->desc_base;
	const struct rza1_dma_slave_config *slave = rza1dmac->slave;
	const struct chcfg_reg *chcfg_p = &slave->chcfg;
	const struct dmars_reg *dmars_p = &slave->dmars;
	int channel = rza1dmac->channel;
	struct scatterlist *sg, *sgl = d->sg;
	unsigned int i, sg_len = d->sgcount;
	u32 chcfg;
	u32 dmars;

	unsigned long flags;
	
	int seten_flag = 1, desc_add_flag = 0;

	dev_dbg(rza1dma->dev, "%s called\n", __func__);

	chcfg = (CHCFG_SEL(channel) |
		CHCFG_REQD(chcfg_p->reqd) |
		CHCFG_LOEN(chcfg_p->loen) |
		CHCFG_HIEN(chcfg_p->hien) |
		CHCFG_LVL(chcfg_p->lvl) |
		CHCFG_AM(chcfg_p->am) |
		CHCFG_SDS(chcfg_p->sds) |
		CHCFG_DDS(chcfg_p->dds) |
		CHCFG_TM(chcfg_p->tm) |
		CHCFG_DEM |
		CHCFG_DMS);

	if (d->direction == DMA_DEV_TO_MEM)
		chcfg |= CHCFG_SAD;	/* source address is fixed */
	else
		chcfg |= CHCFG_DAD;	/* distation address is fixed */

	rza1dmac->per_address = slave->addr;	/* slave device address */

	spin_lock_irqsave(&rza1dma->lock, flags);

	if(sg_len == 1){	/*[sg_len = 1]other than normal process( come value "1" from Audio Driver) */
		for (i = 0; i < FORMAT_DESC_NUM; i++, sg_len++) {
			/* Descripter[0]-[1]is use only at first time.Back to 2 If you go up to 63 */
			if(descs[i].header & HEADER_LE){	/* Link End? */
				if(descs[i].header & HEADER_LV){
					if(i >= 2){
						if(i <= 3){
							if(descs[(FORMAT_DESC_NUM - 4) + i].header & HEADER_LV){
								desc_add_flag = 1;
							}else{
								if(descs[i - 2].header & HEADER_LV){
									desc_add_flag = 1;
								}
							}
						}else{
							if(descs[i - 2].header & HEADER_LV){
								desc_add_flag = 1;
							}
						}
					}else{
						desc_add_flag = 1;
					}

					if(desc_add_flag == 0){
						dev_err(rza1dma->dev, "RZA1 DMAC Link End Now descs !!\n");
						i = 0;
						sg_len = 1;
					}else{
						descs[i].header &= ~HEADER_LE;	/* Link End Clear */
						if(i >= FORMAT_DESC_NUM - 1){	/* if Descripter is over [63] */
							descs[i].next_lk_addr = rza1dmac->desc_base_dma +
										sizeof(struct format_desc) * 2;
							i = 2;
							sg_len = 3;
							seten_flag = 0;
						}else{
							descs[i].next_lk_addr = rza1dmac->desc_base_dma +
										sizeof(struct format_desc) * (i + 1);
							if(i > 1){
								seten_flag = 0;
							}
							i++;
							sg_len++;
						}
					}
				}else{
					i = 0;
					sg_len = 1;
				}
				break;
			}
		}

		if(descs[i].header & HEADER_LV){
			dev_err(rza1dma->dev, "RZA1 DMAC descs over flow!!\n");
		}

		if(i == FORMAT_DESC_NUM){
			i = 0;
			sg_len = 1;
		}
	}else{
		i = 0;
	}

	/* Prepare descriptors */
	for (sg = sgl; i < sg_len; i++, sg = sg_next(sg)) {
		if (d->direction == DMA_DEV_TO_MEM) {
			descs[i].src_addr = rza1dmac->per_address;
			descs[i].dst_addr = sg_dma_address(sg);
		} else {
			descs[i].src_addr = sg_dma_address(sg);
			descs[i].dst_addr = rza1dmac->per_address;
		}

		descs[i].trs_byte = sg_dma_len(sg);
		descs[i].config = chcfg;
		descs[i].interval = 0x00ff;
		descs[i].extension = 0;
		descs[i].next_lk_addr = rza1dmac->desc_base_dma +
					sizeof(struct format_desc) * (i + 1);
		descs[i].header = HEADER_LV;
	}
	descs[sg_len - 1].header |= HEADER_LE;
	descs[sg_len - 1].config &= ~CHCFG_DEM;
	descs[sg_len - 1].next_lk_addr = 0;

	/* and set DMARS register */
	dmars = DMARS_RID(dmars_p->rid) | DMARS_MID(dmars_p->mid);
	set_dmars_register(rza1dma, channel, dmars);

	rza1dmac->chcfg = descs[sg_len - 1].config;
	if(seten_flag == 1){
		rza1dmac->chctrl = CHCTRL_SETEN;
	}else{
		rza1dmac->chctrl = 0;
	}

	spin_unlock_irqrestore(&rza1dma->lock, flags);

}

static int rza1dma_xfer_desc(struct rza1dma_desc *d)
{
	/* Configure and enable */
	switch (d->type) {
	case RZA1DMA_DESC_MEMCPY:
		prepare_desc_for_memcpy(d);
		break;

	case RZA1DMA_DESC_SLAVE_SG:
		prepare_descs_for_slave_sg(d);
		break;

	default:
		return -EINVAL;
	}

	rza1dma_enable_hw(d);
	return 0;
}

static void rza1dma_tasklet(unsigned long data)
{
	struct rza1dma_channel *rza1dmac = (void *)data;
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	struct rza1dma_desc *desc;
	unsigned long flags;

	dev_dbg(rza1dma->dev, "%s called\n", __func__);

	/* Protection of critical section */
	spin_lock_irqsave(&rza1dma->lock, flags);

	if (list_empty(&rza1dmac->ld_active)) {
		/* Someone might have called terminate all */
		goto out;
	}

	desc = list_first_entry(&rza1dmac->ld_active, struct rza1dma_desc, node);

	if (desc->desc.callback)
		desc->desc.callback(desc->desc.callback_param);

	dma_cookie_complete(&desc->desc);


	if (list_empty(&rza1dmac->ld_active)) {
		goto out;
	}else{
		list_move_tail(rza1dmac->ld_active.next, &rza1dmac->ld_free);
	}

	if (!list_empty(&rza1dmac->ld_queue)) {
		desc = list_first_entry(&rza1dmac->ld_queue, struct rza1dma_desc,
					node);
		if (rza1dma_xfer_desc(desc) < 0){
			dev_warn(rza1dma->dev, "%s: channel: %d couldn't xfer desc\n",
				 __func__, rza1dmac->channel);
		}else{
			list_move_tail(rza1dmac->ld_queue.next, &rza1dmac->ld_active);
		}
	}
out:
	spin_unlock_irqrestore(&rza1dma->lock, flags);
}

static int rza1dma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
			unsigned long arg)
{
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct dma_slave_config *dmaengine_cfg = (void *)arg;
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	unsigned long flags;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		rza1dma_disable_hw(rza1dmac);

		spin_lock_irqsave(&rza1dma->lock, flags);
		list_splice_tail_init(&rza1dmac->ld_active, &rza1dmac->ld_free);
		list_splice_tail_init(&rza1dmac->ld_queue, &rza1dmac->ld_free);
		spin_unlock_irqrestore(&rza1dma->lock, flags);
		return 0;
	case DMA_SLAVE_CONFIG:
		if (dmaengine_cfg->direction == DMA_DEV_TO_MEM) {
			rza1dmac->per_address = dmaengine_cfg->src_addr;
			rza1dmac->word_size = dmaengine_cfg->src_addr_width;
		} else {
			rza1dmac->per_address = dmaengine_cfg->dst_addr;
			rza1dmac->word_size = dmaengine_cfg->dst_addr_width;
		}
		return 0;
	default:
		return -ENOSYS;
	}

	return -EINVAL;
}

static const struct rza1_dma_slave_config *dma_find_slave(
		const struct rza1_dma_slave_config *slave,
		int slave_num,
		int slave_id)
{
	int i;

	for (i = 0; i < slave_num; i++) {
		const struct rza1_dma_slave_config *t = &slave[i];

		if (slave_id == t->slave_id)
			return t;
	}

	return NULL;
}

bool rza1dma_chan_filter(struct dma_chan *chan, void *arg)
{
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	struct rza1_dma_pdata *pdata = rza1dma->pdata;
	const struct rza1_dma_slave_config *slave = pdata->slave;
	const struct rza1_dma_slave_config *hit;
	int slave_num = pdata->slave_num;
	int slave_id = (int)arg;

	hit = dma_find_slave(slave, slave_num, slave_id);
	if (hit) {
		rza1dmac->slave = hit;
		return true;
	}
	return false;
}
EXPORT_SYMBOL(rza1dma_chan_filter);

static enum dma_status rza1dma_tx_status(struct dma_chan *chan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
	return dma_cookie_status(chan, cookie, txstate);
}

static dma_cookie_t rza1dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct dma_chan *chan = tx->chan;
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	dma_cookie_t cookie;
	unsigned long flags;

	spin_lock_irqsave(&rza1dma->lock, flags);
	list_move_tail(rza1dmac->ld_free.next, &rza1dmac->ld_queue);
	cookie = dma_cookie_assign(tx);
	spin_unlock_irqrestore(&rza1dma->lock, flags);

	return cookie;
}

static int rza1dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	struct rza1_dma_pdata *pdata = rza1dma->pdata;
	const struct rza1_dma_slave_config *slave = pdata->slave;
	const struct rza1_dma_slave_config *hit;
	int slave_num = pdata->slave_num;
	int *slave_id = chan->private;

	if (slave_id) {
		hit = dma_find_slave(slave, slave_num, *slave_id);
		if (!hit)
			return -ENODEV;
		rza1dmac->slave = hit;
	}

	while (rza1dmac->descs_allocated < RZA1DMA_MAX_CHAN_DESCRIPTORS) {
		struct rza1dma_desc *desc;

		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc)
			break;
		__memzero(&desc->desc, sizeof(struct dma_async_tx_descriptor));
		dma_async_tx_descriptor_init(&desc->desc, chan);
		desc->desc.tx_submit = rza1dma_tx_submit;
		/* txd.flags will be overwritten in prep funcs */
		desc->desc.flags = DMA_CTRL_ACK;
		desc->status = DMA_COMPLETE;

		list_add_tail(&desc->node, &rza1dmac->ld_free);
		rza1dmac->descs_allocated++;
	}
	if (!rza1dmac->descs_allocated)
		return -ENOMEM;

	return rza1dmac->descs_allocated;
}

static void rza1dma_free_chan_resources(struct dma_chan *chan)
{
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	struct format_desc *descs = rza1dmac->desc_base;
	struct rza1dma_desc *desc, *_desc;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&rza1dma->lock, flags);

	for (i = 0; i < FORMAT_DESC_NUM; i++) {
		descs[i].header = 0;
	}

	rza1dma_disable_hw(rza1dmac);
	list_splice_tail_init(&rza1dmac->ld_active, &rza1dmac->ld_free);
	list_splice_tail_init(&rza1dmac->ld_queue, &rza1dmac->ld_free);

	spin_unlock_irqrestore(&rza1dma->lock, flags);

	list_for_each_entry_safe(desc, _desc, &rza1dmac->ld_free, node) {
		kfree(desc);
		rza1dmac->descs_allocated--;
	}
	INIT_LIST_HEAD(&rza1dmac->ld_free);
}

static struct dma_async_tx_descriptor *rza1dma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct scatterlist *sg;
	int i, dma_length = 0;
	struct rza1dma_desc *desc;

	if (list_empty(&rza1dmac->ld_free))
		return NULL;

	desc = list_first_entry(&rza1dmac->ld_free, struct rza1dma_desc, node);

	for_each_sg(sgl, sg, sg_len, i) {
		dma_length += sg_dma_len(sg);
	}

	desc->type = RZA1DMA_DESC_SLAVE_SG;
	desc->sg = sgl;
	desc->sgcount = sg_len;
	desc->len = dma_length;
	desc->direction = direction;

	if (direction == DMA_DEV_TO_MEM)
		desc->src = rza1dmac->per_address;
	else
		desc->dest = rza1dmac->per_address;

	desc->desc.callback = NULL;
	desc->desc.callback_param = NULL;

	return &desc->desc;
}

static struct dma_async_tx_descriptor *rza1dma_prep_dma_memcpy(
	struct dma_chan *chan, dma_addr_t dest,
	dma_addr_t src, size_t len, unsigned long flags)
{
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	struct rza1dma_desc *desc;

	dev_dbg(rza1dma->dev, "%s channel: %d src=0x%x dst=0x%x len=%d\n",
			__func__, rza1dmac->channel, src, dest, len);

	if (list_empty(&rza1dmac->ld_free))
		return NULL;

	desc = list_first_entry(&rza1dmac->ld_free, struct rza1dma_desc, node);

	desc->type = RZA1DMA_DESC_MEMCPY;
	desc->src = src;
	desc->dest = dest;
	desc->len = len;
	desc->direction = DMA_MEM_TO_MEM;
	desc->desc.callback = NULL;
	desc->desc.callback_param = NULL;

	return &desc->desc;
}

static void rza1dma_issue_pending(struct dma_chan *chan)
{
	struct rza1dma_channel *rza1dmac = to_rza1dma_chan(chan);
	struct rza1dma_engine *rza1dma = rza1dmac->rza1dma;
	struct rza1dma_desc *desc;
	unsigned long flags;

	spin_lock_irqsave(&rza1dma->lock, flags);

	/* queue is piled up on the next active even during execution DMA forwarding */
	if (!list_empty(&rza1dmac->ld_queue)) {
		desc = list_first_entry(&rza1dmac->ld_queue,
					struct rza1dma_desc, node);

		if (rza1dma_xfer_desc(desc) < 0) {
			dev_warn(rza1dma->dev,
				 "%s: channel: %d couldn't issue DMA xfer\n",
				 __func__, rza1dmac->channel);
		} else {
			list_move_tail(rza1dmac->ld_queue.next,
				       &rza1dmac->ld_active);
		}
	}
	spin_unlock_irqrestore(&rza1dma->lock, flags);
}

static int __init rza1dma_probe(struct platform_device *pdev)
{
	struct rza1_dma_pdata *pdata = pdev->dev.platform_data;
	int channel_num = pdata->channel_num;
	struct rza1dma_engine *rza1dma;
	struct resource *base_res, *ext_res, *cirq_res, *eirq_res;
	int ret, i;
	int irq, irq_err;

	rza1dma = devm_kzalloc(&pdev->dev, sizeof(*rza1dma), GFP_KERNEL);
	if (!rza1dma)
		return -ENOMEM;

	/* Get io base address */
	base_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!base_res)
		return -ENODEV;
	rza1dma->base = devm_request_and_ioremap(&pdev->dev, base_res);
	if (!rza1dma->base)
		return -EADDRNOTAVAIL;

	/* Get extension io base address */
	ext_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!ext_res)
		return -ENODEV;
	rza1dma->ext_base = devm_request_and_ioremap(&pdev->dev, ext_res);
	if (!rza1dma->ext_base)
		return -EADDRNOTAVAIL;

	/* Register interrupt handler for channels */
	cirq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!cirq_res)
		return -ENODEV;

	for (irq = cirq_res->start; irq <= cirq_res->end; irq++) {
		ret = devm_request_irq(&pdev->dev, irq,
				       rza1dma_irq_handler, 0, "RZA1DMA", rza1dma);
		if (ret) {
			dev_warn(rza1dma->dev, "Can't register IRQ for DMA\n");
			goto err;
		}
	}

	/* Register interrupt handler for error */
	eirq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!eirq_res)
		return -ENODEV;

	irq_err = eirq_res->start;
	ret = devm_request_irq(&pdev->dev, irq_err,
				rza1dma_irq_handler, 0, "RZA1DMA_E", rza1dma);
	if (ret) {
		dev_warn(rza1dma->dev, "Can't register ERRIRQ for DMA\n");
		goto err;
	}

	INIT_LIST_HEAD(&rza1dma->dma_device.channels);
	dma_cap_set(DMA_SLAVE, rza1dma->dma_device.cap_mask);
	dma_cap_set(DMA_MEMCPY, rza1dma->dma_device.cap_mask);
	spin_lock_init(&rza1dma->lock);

	rza1dma->channel = devm_kzalloc(&pdev->dev,
				sizeof(struct rza1dma_channel) * channel_num,
				GFP_KERNEL);

	/* Initialize channel parameters */
	for (i = 0; i < channel_num; i++) {
		struct rza1dma_channel *rza1dmac = &rza1dma->channel[i];

		rza1dmac->rza1dma = rza1dma;

		INIT_LIST_HEAD(&rza1dmac->ld_queue);
		INIT_LIST_HEAD(&rza1dmac->ld_free);
		INIT_LIST_HEAD(&rza1dmac->ld_active);

		tasklet_init(&rza1dmac->dma_tasklet, rza1dma_tasklet,
			     (unsigned long)rza1dmac);
		rza1dmac->chan.device = &rza1dma->dma_device;
		dma_cookie_init(&rza1dmac->chan);
		rza1dmac->channel = i;

		/* Set io base address for each channel */
		if (i < 8) {
			rza1dmac->ch_base = rza1dma->base + CHANNEL_0_7_OFFSET +
						EACH_CHANNEL_OFFSET * i;
			rza1dmac->ch_cmn_base = rza1dma->base +
						CHANNEL_0_7_COMMON_BASE;
		} else {
			rza1dmac->ch_base = rza1dma->base + CHANNEL_8_15_OFFSET	+
						EACH_CHANNEL_OFFSET * (i - 8);
			rza1dmac->ch_cmn_base = rza1dma->base +
						CHANNEL_8_15_COMMON_BASE;
		}
		/* Allocate descriptors */
		rza1dmac->desc_base = dma_alloc_coherent(NULL,
					sizeof(struct format_desc) * FORMAT_DESC_NUM,
					&rza1dmac->desc_base_dma,
					GFP_KERNEL);
		/* Add the channel to the DMAC list */
		list_add_tail(&rza1dmac->chan.device_node,
			      &rza1dma->dma_device.channels);

		/* Initialize register for each channel */
		rza1dma_ch_writel(rza1dmac, CHCTRL_DEFAULT, CHCTRL, 1);
	}

	/* Initialize register for all channels */
	rza1dma_writel(rza1dma, DCTRL_DEFAULT, CHANNEL_0_7_COMMON_BASE	+ DCTRL);
	rza1dma_writel(rza1dma, DCTRL_DEFAULT, CHANNEL_8_15_COMMON_BASE + DCTRL);

	rza1dma->pdata = pdata;
	rza1dma->dev = &pdev->dev;
	rza1dma->dma_device.dev = &pdev->dev;

	rza1dma->dma_device.device_alloc_chan_resources = rza1dma_alloc_chan_resources;
	rza1dma->dma_device.device_free_chan_resources = rza1dma_free_chan_resources;
	rza1dma->dma_device.device_tx_status = rza1dma_tx_status;
	rza1dma->dma_device.device_prep_slave_sg = rza1dma_prep_slave_sg;
	rza1dma->dma_device.device_prep_dma_memcpy = rza1dma_prep_dma_memcpy;
	rza1dma->dma_device.device_control = rza1dma_control;
	rza1dma->dma_device.device_issue_pending = rza1dma_issue_pending;

	platform_set_drvdata(pdev, rza1dma);

	rza1dma->dma_device.copy_align = 0; /* Set copy_align to zero for net_dma_find_channel
					     * func to run well. But it might cause problems.
					     */
	rza1dma->dma_device.copy_align = 0;
	rza1dma->dma_device.dev->dma_parms = &rza1dma->dma_parms;
	dma_set_max_seg_size(rza1dma->dma_device.dev, 0xffffff);

	ret = dma_async_device_register(&rza1dma->dma_device);
	if (ret) {
		dev_err(&pdev->dev, "unable to register\n");
		goto err;
	}
	return 0;
err:
	return ret;
}

static int __exit rza1dma_remove(struct platform_device *pdev)
{
	struct rza1dma_engine *rza1dma = platform_get_drvdata(pdev);
	struct rza1_dma_pdata *pdata = rza1dma->pdata;
	int i, channel_num = pdata->channel_num;

	/* free allocated resources */
	for (i = 0; i < channel_num; i++) {
		struct rza1dma_channel *rza1dmac = &rza1dma->channel[i];

		dma_free_coherent(NULL,
				sizeof(struct format_desc) * FORMAT_DESC_NUM,
				rza1dmac->desc_base,
				rza1dmac->desc_base_dma);
	}

	dma_async_device_unregister(&rza1dma->dma_device);
	return 0;
}

static struct platform_driver rza1dma_driver = {
	.driver		= {
		.name	= "rza1-dma",
	},
	.remove		= __exit_p(rza1dma_remove),
};

static int __init rza1dma_module_init(void)
{
	return platform_driver_probe(&rza1dma_driver, rza1dma_probe);
}
subsys_initcall(rza1dma_module_init);

MODULE_AUTHOR("RSO");
MODULE_DESCRIPTION("Renesas RZA1 DMA Engine driver");
MODULE_LICENSE("GPL");
