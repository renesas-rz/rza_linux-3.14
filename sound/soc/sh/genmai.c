/*
 * sound/soc/sh/genmai.c
 *     This file is ALSA SoC driver for genmai.
 *
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Copyright (C) 2013 Renesas Electronics Corporation
 *
 * This file is based on the sound/soc/sh/migor.c
 *
 * ALSA SoC driver for Migo-R
 *
 * Copyright (C) 2009-2010 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/module.h>

#include <sound/core.h>
#include <sound/pcm.h>
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
static struct scu_route_info *routeinfo;

int scu_check_route(int dir, struct scu_route_info *routeinfo)
{
	if (!dir) { /* playback */
		if (routeinfo->p_route != RP_MEM_SSI0 &&
		    routeinfo->p_route != RP_MEM_SRC1_SSI0 &&
		    routeinfo->p_route != RP_MEM_SRC1_DVC1_SSI0) {
			pr_info("scu playback route is invalid.\n");
			return -EPERM;
		}
	} else { /* capture */
		if (routeinfo->c_route != RC_SSI0_MEM &&
		    routeinfo->c_route != RC_SSI0_SRC0_MEM) {
			pr_info("scu capture route is invalid.\n");
			return -EPERM;
		}
	}

	return 0;
}
EXPORT_SYMBOL(scu_check_route);

/************************************************************************

	DAPM

************************************************************************/
#undef EV_PRINT
#ifdef EV_PRINT
static void event_print(int event, char *evt_str)
{

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_printk(KERN_INFO "%s SND_SOC_DAPM_PRE_PMU\n", evt_str);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_printk(KERN_INFO "%s SND_SOC_DAPM_POST_PMU\n", evt_str);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_printk(KERN_INFO "%s SND_SOC_DAPM_PRE_PMD\n", evt_str);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_printk(KERN_INFO "%s SND_SOC_DAPM_POST_PMD\n", evt_str);
		break;
	default:
		snd_printk(KERN_INFO "%s unknown event\n", evt_str);
	}
}
#else
#define event_print(a, b)
#endif

static int event_ssi0(struct snd_soc_dapm_widget *w,
		      struct snd_kcontrol *kcontrol, int event)
{
	event_print(event, "ssi0");
	if (event == SND_SOC_DAPM_POST_PMU) {
		/* playback */
		routeinfo->pcb.init_ssi = scu_init_ssi;
		routeinfo->pcb.deinit_ssi = scu_deinit_ssi;
		/* add capture */
		routeinfo->ccb.init_ssi = scu_init_ssi;
		routeinfo->ccb.deinit_ssi = scu_deinit_ssi;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		/* playback */
		routeinfo->pcb.init_ssi = NULL;
		routeinfo->pcb.deinit_ssi = NULL;
		/* add capture */
		routeinfo->ccb.init_ssi = NULL;
		routeinfo->ccb.deinit_ssi = NULL;
	}
	return 0;
}

static int event_ssi0_src0(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	event_print(event, "ssi0_src0");
	if (event == SND_SOC_DAPM_POST_PMU) {
		routeinfo->ccb.init_ssi = scu_init_ssi;
		routeinfo->ccb.deinit_ssi = scu_deinit_ssi;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		routeinfo->ccb.init_ssi = NULL;
		routeinfo->ccb.deinit_ssi = NULL;
	}
	return 0;
}

static int event_ssi0_src1(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	event_print(event, "ssi0_src1");
	if (event == SND_SOC_DAPM_POST_PMU) {
		routeinfo->pcb.init_ssi = scu_init_ssi;
		routeinfo->pcb.deinit_ssi = scu_deinit_ssi;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		routeinfo->pcb.init_ssi = NULL;
		routeinfo->pcb.deinit_ssi = NULL;
	}
	return 0;
}

static int event_ssi0_dvc1(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	event_print(event, "ssi0_dvc1");
	if (event == SND_SOC_DAPM_POST_PMU) {
		routeinfo->pcb.init_ssi = scu_init_ssi;
		routeinfo->pcb.deinit_ssi = scu_deinit_ssi;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		routeinfo->pcb.init_ssi = NULL;
		routeinfo->pcb.deinit_ssi = NULL;
	}
	return 0;
}


static int event_src0(struct snd_soc_dapm_widget *w,
		      struct snd_kcontrol *kcontrol, int event)
{
	event_print(event, "src0");
	if (event == SND_SOC_DAPM_POST_PMU) {
		routeinfo->ccb.init_src = scu_init_src;
		routeinfo->ccb.deinit_src = scu_deinit_src;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		routeinfo->ccb.init_src = NULL;
		routeinfo->ccb.deinit_src = NULL;
	}
	return 0;
}

static int event_src1(struct snd_soc_dapm_widget *w,
		      struct snd_kcontrol *kcontrol, int event)
{
	event_print(event, "src1");
	if (event == SND_SOC_DAPM_POST_PMU) {
		routeinfo->pcb.init_src = scu_init_src;
		routeinfo->pcb.deinit_src = scu_deinit_src;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		routeinfo->pcb.init_src = NULL;
		routeinfo->pcb.deinit_src = NULL;
	}
	return 0;
}

static int event_src1_dvc1(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	event_print(event, "src1_dvc1");
	if (event == SND_SOC_DAPM_POST_PMU) {
		routeinfo->pcb.init_src = scu_init_src;
		routeinfo->pcb.deinit_src = scu_deinit_src;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		routeinfo->pcb.init_src = NULL;
		routeinfo->pcb.deinit_src = NULL;
	}
	return 0;
}

static int event_dvc1(struct snd_soc_dapm_widget *w,
		      struct snd_kcontrol *kcontrol, int event)
{
	event_print(event, "dvc1");
	if (event == SND_SOC_DAPM_POST_PMU) {
		routeinfo->pcb.init_dvc = scu_init_dvc;
		routeinfo->pcb.deinit_dvc = scu_deinit_dvc;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		routeinfo->pcb.init_dvc = NULL;
		routeinfo->pcb.deinit_dvc = NULL;
	}
	return 0;
}

static void scu_playback_route_control(struct snd_soc_dapm_context *dapm)
{
	snd_soc_dapm_disable_pin(dapm, "SSI0_OUT0");
	snd_soc_dapm_disable_pin(dapm, "SSI0_OUT1");
	snd_soc_dapm_disable_pin(dapm, "SSI0_OUT2");

	switch (routeinfo->p_route) {
	case RP_MEM_SSI0:
		snd_soc_dapm_enable_pin(dapm, "SSI0_OUT0");
		break;
	case RP_MEM_SRC1_SSI0:
		snd_soc_dapm_enable_pin(dapm, "SSI0_OUT1");
		break;
	case RP_MEM_SRC1_DVC1_SSI0:
		snd_soc_dapm_enable_pin(dapm, "SSI0_OUT2");
		break;
	default:
		break;
	};
}

static void scu_capture_route_control(struct snd_soc_dapm_context *dapm)
{
	snd_soc_dapm_disable_pin(dapm, "SSI0_IN0");
	snd_soc_dapm_disable_pin(dapm, "SSI0_IN1");

	switch (routeinfo->c_route) {
	case RC_SSI0_MEM:
		snd_soc_dapm_enable_pin(dapm, "SSI0_IN0");
		break;
	case RC_SSI0_SRC0_MEM:
		snd_soc_dapm_enable_pin(dapm, "SSI0_IN1");
		break;
	default:
		break;
	};
}

static int scu_get_ssi0_route(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = routeinfo->route_ssi[0];

	return 0;
}

static int scu_set_ssi0_route(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);

	if (routeinfo->route_ssi[0] == ucontrol->value.integer.value[0])
		return 0;

	routeinfo->route_ssi[0] = ucontrol->value.integer.value[0];

	if (routeinfo->route_ssi[0])
		routeinfo->p_route |= RP_MEM_SSI0;
	else
		routeinfo->p_route &= ~RP_MEM_SSI0;

	scu_playback_route_control(&card->dapm);

	return 1;
}

static int scu_set_ssi0_caproute(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	DBG_MSG("start:scu_set_ssi0_caproute");
	if (routeinfo->route_ssi[0] == ucontrol->value.integer.value[0])
		return 0;

	routeinfo->route_ssi[0] = ucontrol->value.integer.value[0];

	if (routeinfo->route_ssi[0])
		routeinfo->c_route |= RC_SSI0_MEM;
	else
		routeinfo->c_route &= ~RC_SSI0_MEM;

	scu_capture_route_control(&card->dapm);
	return 1;
}

static int scu_get_src0_route(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = routeinfo->route_src[SRC0];

	return 0;
}

static int scu_set_src0_route(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);

	if (routeinfo->route_src[SRC0] == ucontrol->value.integer.value[0])
		return 0;

	routeinfo->route_src[SRC0] = ucontrol->value.integer.value[0];
	if (routeinfo->route_src[SRC0])
		routeinfo->c_route |= W_SRC0;
	else
		routeinfo->c_route &= ~W_SRC0;

	scu_capture_route_control(&card->dapm);

	return 1;
}

static int scu_get_src1_route(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = routeinfo->route_src[SRC1];

	return 0;
}
static int scu_set_src1_route(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);

	if (routeinfo->route_src[SRC1] == ucontrol->value.integer.value[0])
		return 0;

	routeinfo->route_src[SRC1] = ucontrol->value.integer.value[0];
	if (routeinfo->route_src[SRC1])
		routeinfo->p_route |= W_SRC1;
	else
		routeinfo->p_route &= ~W_SRC1;

	scu_playback_route_control(&card->dapm);

	return 1;
}

static int scu_get_dvc1_route(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = routeinfo->route_dvc[DVC1];

	return 0;
}

static int scu_set_dvc1_route(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);

	if (routeinfo->route_dvc[DVC1] == ucontrol->value.integer.value[0])
		return 0;

	routeinfo->route_dvc[DVC1] = ucontrol->value.integer.value[0];
	if (routeinfo->route_dvc[DVC1])
		routeinfo->p_route |= W_DVC1;
	else
		routeinfo->p_route &= ~W_DVC1;

	scu_playback_route_control(&card->dapm);

	return 1;
}

static const char * const widget_switch[] = {"Off", "On"};

static const struct soc_enum widget_switch_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(widget_switch), widget_switch);

static const struct snd_kcontrol_new playback_controls[] = {
	SOC_ENUM_EXT("SSI0 Control", widget_switch_enum,
	scu_get_ssi0_route, scu_set_ssi0_route),
	SOC_ENUM_EXT("SRC1 Control", widget_switch_enum,
	scu_get_src1_route, scu_set_src1_route),
	SOC_ENUM_EXT("DVC1 Control", widget_switch_enum,
	scu_get_dvc1_route, scu_set_dvc1_route),
};

static const struct snd_kcontrol_new capture_controls[] = {
	SOC_ENUM_EXT("SSI0 CapControl", widget_switch_enum,
	scu_get_ssi0_route, scu_set_ssi0_caproute),
	SOC_ENUM_EXT("SRC0 Control", widget_switch_enum,
	scu_get_src0_route, scu_set_src0_route),
};

static const struct snd_soc_dapm_widget genmai_dapm_widgets[] = {
	/* Playback */
	SND_SOC_DAPM_OUTPUT("SSI0_OUT0"),
	SND_SOC_DAPM_OUTPUT("SSI0_OUT1"),
	SND_SOC_DAPM_OUTPUT("SSI0_OUT2"),
	SND_SOC_DAPM_DAC("MEM_OUT", "HiFi Playback", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MIXER_E("SSI0", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_ssi0,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("SSI0_SRC1", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_ssi0_src1,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("SSI0_DVC1", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_ssi0_dvc1,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("SRC1", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_src1,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("DVC1", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_dvc1,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("src1_dvc1", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_src1_dvc1,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* Capture */
	SND_SOC_DAPM_ADC("MEM_IN", "HiFi Capture", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_INPUT("SSI0_IN0"),
	SND_SOC_DAPM_INPUT("SSI0_IN1"),
	SND_SOC_DAPM_MIXER_E("SSI0", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_ssi0,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("SSI0_SRC0", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_ssi0_src0,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("SRC0", SND_SOC_NOPM, 0, 0, NULL, 0,
	event_src0,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Playback route */
	/* SSI<-MEM */
	{"SSI0", NULL, "MEM_OUT"},
	{"SSI0_OUT0", NULL, "SSI0"},
	/* SSI<-SRC<-MEM */
	{"SRC1", NULL, "MEM_OUT"},
	{"SSI0_SRC1", NULL, "SRC1"},
	{"SSI0_OUT1", NULL, "SSI0_SRC1"},
	/* SSI<-DVC<-SRC<-MEM */
	{"DVC1", NULL, "SRC1"},
	{"SSI0_DVC1", NULL, "DVC1"},
	{"SSI0_OUT2", NULL, "SSI0_DVC1"},

	/* Capture route */
	/* MEM<-SSI */
	{"SSI0", NULL, "SSI0_IN0"},
	{"MEM_IN", NULL, "SSI0"},
	/* MEM<-SRC<-SSI */
	{"SSI0_SRC0", NULL, "SSI0_IN1"},
	{"SRC0", NULL, "SSI0_SRC0"},
	{"MEM_IN", NULL, "SRC0"},
};

/************************************************************************

	ALSA SoC

************************************************************************/
static int genmai_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	/* set PLL clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 11289600, SND_SOC_CLOCK_IN);
	if (ret) {
		pr_err("snd_soc_dai_set_sysclk err=%d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_CBS_CFS |
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF);
	if (ret) {
		pr_err("snd_soc_dai_set_fmt err=%d\n", ret);
		return ret;
	}

	return ret;
}

static int genmai_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops genmai_dai_ops = {
	.hw_params = genmai_hw_params,
	.hw_free = genmai_hw_free,
};

static int genmai_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	FNC_ENTRY

	/* Add controls */
	ret = snd_soc_add_card_controls(rtd->card, playback_controls,
					ARRAY_SIZE(playback_controls));
	if (ret) {
		pr_err("snd_soc_add_card_controls(playback) err=%d\n", ret);
		return ret;
	}

	ret = snd_soc_add_card_controls(rtd->card, capture_controls,
					ARRAY_SIZE(capture_controls));
	if (ret) {
		pr_err("snd_soc_add_card_controls(capture) err=%d\n", ret);
		return ret;
	}

	/* Add widget and route for scu */
	ret = snd_soc_dapm_new_controls(dapm, genmai_dapm_widgets,
					ARRAY_SIZE(genmai_dapm_widgets));
	if (ret) {
		pr_err("snd_soc_dapm_new_controls err=%d\n", ret);
		return ret;
	}

	ret = snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	if (ret) {
		pr_err("snd_soc_dapm_add_routes err=%d\n", ret);
		return ret;
	}

	scu_playback_route_control(dapm);
	scu_capture_route_control(dapm);

	FNC_EXIT
	return ret;
}
/* genmai digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link genmai_dai = {
	.name		= "wm8978",		/* Codec name */
	.stream_name	= "WM8978",		/* Stream name */
	.cpu_dai_name	= "scu-ssi-dai",	/* DAI name of the CPU DAI*/
	.codec_dai_name	= "wm8978-hifi",	/* DAI name within the codec */
	.platform_name	= "scux-pcm-audio.0",	/* device name */
#if (defined(CONFIG_SPI_MASTER) && defined(CONFIG_MACH_RSKRZA1))
	.codec_name	= "spi4.0",		/* device name */
#else
	.codec_name	= "wm8978.0-001a",	/* device name */
#endif
	.ops		= &genmai_dai_ops,	/* machine stream operations */
	.init		= genmai_dai_init,	/* machine specific init */
};

/* genmai audio machine driver */
static struct snd_soc_card snd_soc_genmai = {
	.name = "genmai-wm8978",
	.owner = THIS_MODULE,
	.dai_link = &genmai_dai,
	.num_links = 1,
};

static int genmai_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;

	FNC_ENTRY

	routeinfo = scu_get_route_info();

	snd_soc_genmai.dev = &pdev->dev;
	ret = snd_soc_register_card(&snd_soc_genmai);

	if (ret)
		pr_err("Unable to register sourd card\n");

	FNC_EXIT
	return ret;
}

static struct platform_driver genmai_alsa_driver = {
	.driver = {
		.name = "genmai_alsa_soc_platform",
		.owner = THIS_MODULE,
	},
	.probe = genmai_probe,
};

module_platform_driver(genmai_alsa_driver);

MODULE_AUTHOR(" Renesas Solutions Corp.");
MODULE_DESCRIPTION("ALSA SoC GENMAI");
MODULE_LICENSE("GPL v2");
