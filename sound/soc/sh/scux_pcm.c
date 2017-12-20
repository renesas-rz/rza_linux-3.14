/*
 * sound/soc/sh/scux_pcm.c
 *     This file is ALSA SoC driver for SCUX peripheral.
 *
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Copyright (C) 2013 Renesas Electronics Corporation
 *
 * This file is based on the sound/soc/sh/siu_pcm.c
 *
 * siu_pcm.c - ALSA driver for Renesas SH7343, SH7722 SIU peripheral.
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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/dma-rza1.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
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

static u64 dma_mask = DMA_BIT_MASK(32);
static unsigned int codec_powerup_wait;
module_param(codec_powerup_wait, uint, 0644);

static struct snd_soc_dai *scu_get_dai(struct snd_pcm_substream *ss)
{
	struct snd_soc_pcm_runtime *rtd = ss->private_data;

	return  rtd->cpu_dai;
}

static void scu_avoid_end_noise(int dir, u32 *st_ptr, int size)
{
	if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
		while (0 < size) {
			*st_ptr = 0;
			st_ptr++;
			size--;
		}
	}
}

static void scu_dma_callback(struct snd_pcm_substream *ss)
{
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;
	snd_pcm_uframes_t period, buf, tran, remain, threshold, tran_piece;
	u32 *buf_st_vadr, *buf_end_vadr;
	snd_pcm_uframes_t st_point;
	int dir = ss->stream;

	FNC_ENTRY

	period = runtime->period_size;
	buf = runtime->buffer_size;
	tran = pcminfo->tran_size;
	remain = buf - (tran % buf);
	threshold = period + (period / 2);
	tran_piece = pcminfo->tran_period_piece;
	buf_st_vadr = (u32 *)runtime->dma_area;
	buf_end_vadr = (u32 *)(runtime->dma_area + runtime->dma_bytes);
	st_point = tran % buf;

	if ((0 < remain) && (remain <= threshold)) {
		tran += remain;
		if (remain <= period)
			tran_piece = period - remain;
		else
			tran_piece = (period * 2) - remain;
		scu_avoid_end_noise(dir, buf_end_vadr - (period / 2),
								period / 2);
	} else if (tran_piece > 0) {
		if ((tran_piece + period) < threshold)
			tran += (tran_piece + period);
		else
			tran += tran_piece;
		tran_piece = 0;
		scu_avoid_end_noise(dir, buf_st_vadr, tran % buf);
	} else {
		tran += period;
		if ((0 < st_point) && (st_point < threshold) &&
							((buf % period) != 0))
			scu_avoid_end_noise(dir, buf_st_vadr + st_point,
						(threshold - 1) - st_point);
	}

	pcminfo->tran_size = tran;
	pcminfo->tran_period_piece = tran_piece;

	/* Notify alsa */
	snd_pcm_period_elapsed(ss);

	/* stop dma */
	if (pcminfo->flag_start == 0)
		return;

	queue_work(pcminfo->workq, &pcminfo->work);

	FNC_EXIT
}

static int scu_dmae_req_chan(int sid, struct snd_pcm_substream *ss)
{
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;
	struct rza1_dma_slave *param = &pcminfo->de_param[sid];
	struct dma_slave_config cfg;
	dma_cap_mask_t mask;
	int ret = 0;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	FNC_ENTRY

	/* set dma slave id */
	param->rza1dma_slaveid.slave_id = sid;

	/* request dma channel */
	if (pcminfo->de_chan[sid] == NULL) {
		pcminfo->de_chan[sid] = dma_request_channel(mask,
					      rza1dma_chan_filter, (void *)sid);
		if (!pcminfo->de_chan[sid]) {
			snd_printk(KERN_ERR "DMA channel request error\n");
			return -EBUSY;
		}

		cfg.slave_id = sid;
		ret = dmaengine_slave_config(pcminfo->de_chan[sid], &cfg);
		if (ret < 0) {
			dma_release_channel(pcminfo->de_chan[sid]);
			return ret;
		}
	}

	DBG_MSG("chan=0x%08x\n", (int)pcminfo->de_chan[sid]);

	FNC_EXIT
	return ret;
}

static void scu_dmae_rel_chan(int sid, struct snd_pcm_substream *ss)
{
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;

	FNC_ENTRY

	/* release dma channel */
	if (pcminfo->de_chan[sid]) {
		dma_release_channel(pcminfo->de_chan[sid]);
		pcminfo->de_chan[sid] = NULL;
	}

	FNC_EXIT
	return;
}

static int scu_dmae_request(struct snd_pcm_substream *ss)
{
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;
	int dir = ss->stream == SNDRV_PCM_STREAM_CAPTURE;
	int route = 0;
	int audma_slave_id = 0;
	int ret = 0;

	FNC_ENTRY

	if (!dir) { /* playback */
		route = pcminfo->routeinfo->p_route;
	} else { /* capture */
		route = pcminfo->routeinfo->c_route;
	}

	audma_slave_id = scu_find_data(route, pcminfo->pdata->audma_slave,
					pcminfo->pdata->audma_slave_num);
	if (audma_slave_id != -1) {
		ret = scu_dmae_req_chan(audma_slave_id, ss);
		if (ret < 0)
			return ret;
	}

	FNC_EXIT
	return ret;
}

static int scu_dmae_release(struct snd_pcm_substream *ss)
{
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;
	int dir = ss->stream == SNDRV_PCM_STREAM_CAPTURE;
	int route = 0;
	int audma_slave_id = 0;
	int ret = 0;

	FNC_ENTRY

	if (!dir) /* playback */
		route = pcminfo->routeinfo->p_route;
	else /* capture */
		route = pcminfo->routeinfo->c_route;

	audma_slave_id = scu_find_data(route, pcminfo->pdata->audma_slave,
					pcminfo->pdata->audma_slave_num);
	if (audma_slave_id != -1)
		scu_dmae_rel_chan(audma_slave_id, ss);

	FNC_EXIT
	return ret;
}

static int scu_audma_start(int sid, struct snd_pcm_substream *ss)
{
	int dir = ss->stream == SNDRV_PCM_STREAM_CAPTURE;
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct scu_pcm_info *pcminfo = runtime->private_data;
	struct device *dev = ss->pcm->card->dev;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	u32 dma_size;
	u32 dma_paddr;
	struct snd_soc_dai *dai;
	snd_pcm_uframes_t period, buf, offset, remain, threshold, period_piece;

	FNC_ENTRY

	dai = scu_get_dai(ss);

	/* DMA physical adddress */
	dma_paddr = runtime->dma_addr
			+ frames_to_bytes(runtime, pcminfo->buf_offset);
	DBG_MSG("dma_paddr=0x%08x\n", dma_paddr);

	period = runtime->period_size;
	buf = runtime->buffer_size;
	offset = pcminfo->buf_offset;
	remain = buf - offset;
	threshold = period + (period / 2);
	period_piece = pcminfo->period_piece;

	/* DMA size */
	if ((0 < remain) && (remain <= threshold)) {
		dma_size = frames_to_bytes(runtime, remain);
		offset = 0;
		if (remain <= period)
			period_piece = period - remain;
		else
			period_piece = (period * 2) - remain;
	} else if (period_piece > 0) {
		if (period_piece + period < threshold) {
			dma_size = frames_to_bytes(runtime,
						period_piece + period);
			offset += (period_piece + period);
		} else {
			dma_size = frames_to_bytes(runtime, period_piece);
			offset += period_piece;
		}
		period_piece = 0;
	} else {
		dma_size = frames_to_bytes(runtime, period);
		offset += period;
		if (offset >= buf)
			offset -= buf;
	}
	DBG_MSG("dma_size=%d\n", dma_size);

	pcminfo->buf_offset = offset;
	pcminfo->period_piece = period_piece;

	dma_sync_single_for_device(dai->dev, dma_paddr, dma_size, DMA_DIR(dir));

	desc = dmaengine_prep_slave_single(pcminfo->de_chan[sid], dma_paddr,
		dma_size, DMA_DIR(dir), DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(dai->dev, "dmaengine_prep_slave_sg_single() fail\n");
		return -ENOMEM;
	}

	desc->callback = (dma_async_tx_callback)scu_dma_callback;
	desc->callback_param = ss;

	cookie = dmaengine_submit(desc);
	if (cookie < 0) {
		dev_err(dev, "Failed to submit a dma transfer\n");
		FNC_EXIT
		return cookie;
	}

	dma_async_issue_pending(pcminfo->de_chan[sid]);

	FNC_EXIT
	return 0;
}

static int scu_audma_stop(int sid, struct snd_pcm_substream *ss)
{
	FNC_ENTRY

	FNC_EXIT
	return 0;
}

static void scu_pcm_start(struct snd_pcm_substream *ss, int first_flag)
{
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;
	int dir = ss->stream == SNDRV_PCM_STREAM_CAPTURE;
	int route = 0;
	int audma_slave_id = 0;
	int ssi_depend = 0;
	int ssi_mode = 0;
	int src_ch = 0;
	int src_mode = 0;
	int dvc_ch = 0;
	struct scu_pcm_callback callback;

	FNC_ENTRY

	if (!dir) { /* playback */
		route = pcminfo->routeinfo->p_route;
		callback = pcminfo->routeinfo->pcb;
	} else { /* capture */
		route = pcminfo->routeinfo->c_route;
		callback = pcminfo->routeinfo->ccb;
	}

	audma_slave_id = scu_find_data(route, pcminfo->pdata->audma_slave,
					pcminfo->pdata->audma_slave_num);
	ssi_depend = scu_find_data(route, pcminfo->pdata->ssi_depend,
					pcminfo->pdata->ssi_depend_num);
	ssi_mode = scu_find_data(route, pcminfo->pdata->ssi_mode,
					pcminfo->pdata->ssi_mode_num);
	src_ch = scu_find_data(route, pcminfo->pdata->src_ch,
					pcminfo->pdata->src_ch_num);
	src_mode = scu_find_data(route, pcminfo->pdata->src_mode,
					pcminfo->pdata->src_mode_num);
	dvc_ch = scu_find_data(route, pcminfo->pdata->dvc_ch,
					pcminfo->pdata->dvc_ch_num);

	/* start dma */
	scu_audma_start(audma_slave_id, ss);

	if (first_flag) {
		/* Four Descripters are registered first */

		/* start dma */
		scu_audma_start(audma_slave_id, ss);

		/* start dma */
		scu_audma_start(audma_slave_id, ss);

		/* start dma */
		scu_audma_start(audma_slave_id, ss);

		/* start ssi */
		if (callback.init_ssi)
			callback.init_ssi(pcminfo->pdata->ssi_master,
				pcminfo->pdata->ssi_slave,
				ssi_mode, ssi_depend, dir);

		/* start dvc */
		if (callback.init_dvc)
			callback.init_dvc(dvc_ch);

		/* start src */
		if (callback.init_src)
			callback.init_src(src_ch, ss->runtime->rate, src_mode);
	}

	FNC_EXIT
	return;
}

static void scu_pcm_stop(struct snd_pcm_substream *ss)
{
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;
	int dir = ss->stream == SNDRV_PCM_STREAM_CAPTURE;
	int route = 0;
	int audma_slave_id = 0;
	int ssi_ch = 0;
	int ssi_depend = 0;
	int ssi_mode = 0;
	int src_ch = 0;
	int dvc_ch = 0;
	struct scu_pcm_callback callback;

	FNC_ENTRY

	if (!dir) { /* playback */
		route = pcminfo->routeinfo->p_route;
		callback = pcminfo->routeinfo->pcb;
	} else { /* capture */
		route = pcminfo->routeinfo->c_route;
		callback = pcminfo->routeinfo->ccb;
	}

	audma_slave_id = scu_find_data(route, pcminfo->pdata->audma_slave,
					pcminfo->pdata->audma_slave_num);
	ssi_ch = scu_find_data(route, pcminfo->pdata->ssi_ch,
					pcminfo->pdata->ssi_ch_num);
	ssi_depend = scu_find_data(route, pcminfo->pdata->ssi_depend,
					pcminfo->pdata->ssi_depend_num);
	ssi_mode = scu_find_data(route, pcminfo->pdata->ssi_mode,
					pcminfo->pdata->ssi_mode_num);
	src_ch = scu_find_data(route, pcminfo->pdata->src_ch,
					pcminfo->pdata->src_ch_num);
	dvc_ch = scu_find_data(route, pcminfo->pdata->dvc_ch,
					pcminfo->pdata->dvc_ch_num);
	/* stop src */
	if (callback.deinit_src)
		callback.deinit_src(src_ch);

	/* stop dvc */
	if (callback.deinit_dvc)
		callback.deinit_dvc(dvc_ch);

	/* stop ssi */
	if (callback.deinit_ssi)
		callback.deinit_ssi(ssi_ch, ssi_mode, ssi_depend, dir);

	/* stop dma */
	scu_audma_stop(audma_slave_id, ss);

	FNC_EXIT
	return;
}

static void scu_dma_do_work(struct work_struct *work)
{
	struct scu_pcm_info *pcminfo =
			container_of(work, struct scu_pcm_info, work);
	struct snd_pcm_substream *ss = pcminfo->ss;

	FNC_ENTRY

	/* start pcm process */
	scu_pcm_start(ss, pcminfo->flag_first);
	if (pcminfo->flag_first == 1)
		pcminfo->flag_first = 0;

	FNC_EXIT
	return;
}

static int scu_audio_start(struct snd_pcm_substream *ss)
{
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;
	int ret = 0;

	FNC_ENTRY

	/* dma channel request */
	ret = scu_dmae_request(ss);
	if (ret < 0) {
		pr_info("scu_dmae_request faild\n");
		FNC_EXIT
		return ret;
	}

	/* DMA control */
	pcminfo->flag_start = 1;
	/* PCM 1st process */
	pcminfo->flag_first = 1;

	pcminfo->buf_offset = 0;
	pcminfo->tran_size = 0;
	pcminfo->period_piece = 0;
	pcminfo->tran_period_piece = 0;

	mdelay(codec_powerup_wait);
	queue_work(pcminfo->workq, &pcminfo->work);

	FNC_EXIT
	return ret;
}

static int scu_audio_stop(struct snd_pcm_substream *ss)
{
	int ret = 0;
	struct scu_pcm_info *pcminfo = ss->runtime->private_data;

	FNC_ENTRY

	/* stop dma */
	pcminfo->flag_start = 0;

	/* stop pcm process */
	scu_pcm_stop(ss);

	/* dma channel release */
	ret = scu_dmae_release(ss);

	/* Cancel work queue */
	cancel_work_sync(&pcminfo->work);

	FNC_EXIT
	return ret;
}

static struct scu_pcm_info *scu_pcm_new_stream(struct snd_pcm_substream *ss)
{
	struct scu_pcm_info *pcminfo;

	FNC_ENTRY

	/* allocate scu_pcm_info structure */
	pcminfo = kzalloc(sizeof(struct scu_pcm_info), GFP_KERNEL);
	if (!pcminfo)
		return pcminfo;

	/* initialize rcar_pcm_info structure */
	pcminfo->routeinfo   = scu_get_route_info();
	pcminfo->ss          = ss;
	pcminfo->pdata       = scu_get_platform_data();

	/* allocate dma_chan structure */
	pcminfo->de_chan = kzalloc((sizeof(struct dma_chan) *
					pcminfo->pdata->dma_slave_maxnum),
					GFP_KERNEL);
	if (!pcminfo->de_chan) {
		kfree(pcminfo);
		return NULL;
	}

	/* allocate sh_dmadesc_slave structure */
	pcminfo->de_param = kzalloc((sizeof(struct rza1_dma_slave) *
					pcminfo->pdata->dma_slave_maxnum),
					GFP_KERNEL);
	if (!pcminfo->de_param) {
		kfree(pcminfo->de_chan);
		kfree(pcminfo);
		return NULL;
	}
	spin_lock_init(&pcminfo->pcm_lock);
	pcminfo->workq = alloc_ordered_workqueue("sh_scu_pcm", 0);
	INIT_WORK(&pcminfo->work, scu_dma_do_work);

	FNC_EXIT
	return pcminfo;
}

static void scu_pcm_free_stream(struct snd_pcm_runtime *runtime)
{
	struct scu_pcm_info *pcminfo = runtime->private_data;

	FNC_ENTRY

	/* post process */
	cancel_work_sync(&pcminfo->work);
	destroy_workqueue(pcminfo->workq);
	kfree(pcminfo->de_param);
	kfree(pcminfo->de_chan);
	kfree(runtime->private_data);	/* free pcminfo structure */

	FNC_EXIT
	return;
}

static int scu_pcm_open(struct snd_pcm_substream *ss)
{
	struct scu_pcm_info *pcminfo;
	int dir = ss->stream == SNDRV_PCM_STREAM_CAPTURE;
	int ret = 0;

	FNC_ENTRY

	pcminfo = scu_pcm_new_stream(ss);
	if (pcminfo == NULL)
		return -ENOMEM;

	ret = scu_check_route(dir, pcminfo->routeinfo);
	if (ret < 0)
		return ret;

	ss->runtime->private_data = pcminfo;
	ss->runtime->private_free = scu_pcm_free_stream;

	FNC_EXIT
	return 0;
}

static int scu_pcm_close(struct snd_pcm_substream *ss)
{
	FNC_ENTRY

	FNC_EXIT
	return 0;
}

static int scu_pcm_hw_params(struct snd_pcm_substream *ss,
			     struct snd_pcm_hw_params *hw_params)
{
	struct device *dev = ss->pcm->card->dev;
	int ret;

	FNC_ENTRY

	ret = snd_pcm_lib_malloc_pages(ss, params_buffer_bytes(hw_params));
	if (ret < 0)
		dev_err(dev, "snd_pcm_lib_malloc_pages() failed\n");

	FNC_EXIT
	return ret;
}

static int scu_pcm_hw_free(struct snd_pcm_substream *ss)
{
	struct device *dev = ss->pcm->card->dev;
	int ret;

	FNC_ENTRY

	ret = snd_pcm_lib_free_pages(ss);
	if (ret < 0)
		dev_err(dev, "snd_pcm_lib_free_pages() failed\n");

	FNC_EXIT
	return ret;
}

static int scu_pcm_prepare(struct snd_pcm_substream *ss)
{
	FNC_ENTRY

	FNC_EXIT
	return 0;
}

static int scu_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct scu_pcm_info *pcminfo = runtime->private_data;

	spin_lock(&pcminfo->pcm_lock);

	FNC_ENTRY

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = scu_audio_start(ss);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = scu_audio_stop(ss);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock(&pcminfo->pcm_lock);

	FNC_EXIT
	return ret;
}

static snd_pcm_uframes_t scu_pcm_pointer_dma(struct snd_pcm_substream *ss)
{
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct scu_pcm_info *pcminfo = runtime->private_data;
	snd_pcm_uframes_t position = 0;

	position = pcminfo->tran_size % runtime->buffer_size;

	DBG_MSG("\tposition = %d\n", (u32)position);

	return position;
}

static int scu_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_card *card = rtd->card->snd_card;

	FNC_ENTRY

	ret = scu_dai_add_control(card);

	if (ret)
		return ret;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &dma_mask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
	ret = snd_pcm_lib_preallocate_pages_for_all(
		rtd->pcm,
		SNDRV_DMA_TYPE_DEV,
		rtd->card->snd_card->dev,
		SCU_BUFFER_BYTES_MAX, SCU_BUFFER_BYTES_MAX);

	FNC_EXIT
	return ret;
}

static void scu_pcm_free(struct snd_pcm *pcm)
{
	FNC_ENTRY

	/* free dma buffer */
	snd_pcm_lib_preallocate_free_for_all(pcm);

	FNC_EXIT
}

static struct snd_pcm_ops scu_pcm_ops = {
	.open		= scu_pcm_open,
	.close		= scu_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= scu_pcm_hw_params,
	.hw_free	= scu_pcm_hw_free,
	.prepare	= scu_pcm_prepare,
	.trigger	= scu_pcm_trigger,
	.pointer	= scu_pcm_pointer_dma,
};

struct snd_soc_platform_driver scu_platform = {
	.ops		= &scu_pcm_ops,
	.pcm_new	= scu_pcm_new,
	.pcm_free	= scu_pcm_free,
};
EXPORT_SYMBOL_GPL(scu_platform);
