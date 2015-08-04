/*
 * max9856.c  --  codec driver for max9856
 *
 * Copyright (C) 2011 taskit GmbH
 *
 * Author: Christian Glindkamp <christian.glindkamp@taskit.de>
 *
 * Initial development of this code was funded by
 * MICRONIC Computer Systeme GmbH, http://www.mcsberlin.de/
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "max9856.h"

struct max9856_priv {
	struct regmap *regmap;
	unsigned int sysclk;
};

/* max9856 register cache */
static const struct reg_default max9856_reg[] = {
	{ 0x02, 0x00 },
	{ 0x03, 0x00 },
	{ 0x04, 0x00 },
	{ 0x05, 0x00 },
	{ 0x06, 0x00 },
	{ 0x07, 0x00 },
	{ 0x08, 0x00 },
	{ 0x09, 0x00 },
	{ 0x0a, 0x00 },
	{ 0x0b, 0x00 },
	{ 0x0c, 0x00 },
	{ 0x0d, 0x00 },
	{ 0x0e, 0x08 },
	{ 0x0f, 0x04 },
	{ 0x10, 0x11 },
	{ 0x11, 0x00 },
	{ 0x12, 0x00 },
	{ 0x13, 0x00 },
	{ 0x14, 0x00 },
	{ 0x15, 0x00 },
	{ 0x16, 0x00 },
	{ 0x17, 0x00 },
	{ 0x18, 0x00 },
	{ 0x19, 0x00 },
	{ 0x1a, 0x00 },
	{ 0x1b, 0x00 },
	{ 0x1c, 0xbf },
};

/* these registers are not used at the moment but provided for the sake of
 * completeness */
static bool max9856_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX9856_STATUSA:
	case MAX9856_STATUSB:
		return 1;
	default:
		return 0;
	}
}

static const struct regmap_config max9856_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = MAX9856_PM,
	.volatile_reg = max9856_volatile_register,
	.cache_type = REGCACHE_RBTREE,
};

static const DECLARE_TLV_DB_SCALE(max9856_adc_gain_tlv, -1200, 100, 0);
static const DECLARE_TLV_DB_SCALE(max9856_pgal_tlv, -3200, 200, 0);
static const unsigned int max9856_pgam_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0xB, 0x1f, TLV_DB_SCALE_ITEM(0, 100, 0),
};

static const unsigned int max9856_pgads_tlv[] = {
	TLV_DB_RANGE_HEAD(6),
	0x1A, 0x2F, TLV_DB_SCALE_ITEM(-4000, 50, 0),
	0x30, 0x5F, TLV_DB_SCALE_ITEM(-2900, 25, 0),
	0x60, 0x7F, TLV_DB_SCALE_ITEM(-1700, 15, 0),
	0x80, 0xCF, TLV_DB_SCALE_ITEM(-1200, 10, 0),
	0xD0, 0xFE, TLV_DB_SCALE_ITEM(-350, 7, 0),
	0xFF, 0xFF, TLV_DB_SCALE_ITEM(0, 0, 0),
};

static const unsigned int max9856_hpvol_tlv[] = {
	TLV_DB_RANGE_HEAD(4),
	0x18, 0x1f, TLV_DB_SCALE_ITEM(-7400, 400, 0),
	0x20, 0x33, TLV_DB_SCALE_ITEM(-4200, 200, 0),
	0x34, 0x37, TLV_DB_SCALE_ITEM(-200, 100, 0),
	0x38, 0x3f, TLV_DB_SCALE_ITEM(200, 50, 0),
};

static const struct snd_kcontrol_new max9856_controls[] = {
SOC_SINGLE_TLV("ADC Out Volume", MAX9856_ADC_LEVEL, 4, 0xf, 1, max9856_adc_gain_tlv),
SOC_SINGLE_TLV("Digital Audio Input Volume", MAX9856_DIGITAL_GAIN, 0, 0xff, 1, max9856_pgads_tlv),
SOC_SINGLE_TLV("PGAAUX Volume", MAX9856_AUXIN_GAIN, 0, 0x1f, 1, max9856_pgal_tlv),
SOC_SINGLE_TLV("LINEIN1 Volume", MAX9856_LINEIN1_GAIN, 0, 0x1f, 1, max9856_pgal_tlv),
SOC_SINGLE_TLV("LINEIN2 Volume", MAX9856_LINEIN2_GAIN, 0, 0x1f, 1, max9856_pgal_tlv),
SOC_SINGLE_TLV("PGAML Volume", MAX9856_MICL_GAIN, 0, 0x1f, 1, max9856_pgam_tlv),
SOC_SINGLE_TLV("Headphone Left Volume", MAX9856_HPL_VOL, 0, 0x3f, 1, max9856_hpvol_tlv),
SOC_SINGLE_TLV("Headphone Right Volume", MAX9856_HPR_VOL, 0, 0x3f, 1, max9856_hpvol_tlv),
};

/* Left Output Mixer Switch */
static const struct snd_kcontrol_new max9856_lout_mixer_controls[] = {
	SOC_DAPM_SINGLE("DAC Switch", MAX9856_MIX_OUT,
		MAX9856_MIX_OUT_MXOUTL_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("LINEIN2 Switch", MAX9856_MIX_OUT,
		MAX9856_MIX_OUT_MXOUTL_SHIFT + 1, 1, 0),
	SOC_DAPM_SINGLE("LINEIN1 Switch", MAX9856_MIX_OUT,
		MAX9856_MIX_OUT_MXOUTL_SHIFT + 2, 1, 0),
	SOC_DAPM_SINGLE("MIC PGA Switch", MAX9856_MIX_OUT,
		MAX9856_MIX_OUT_MXOUTL_SHIFT + 3, 1, 0),
};

/* Right Output Mixer Switch */
static const struct snd_kcontrol_new max9856_rout_mixer_controls[] = {
	SOC_DAPM_SINGLE("DAC Switch", MAX9856_MIX_OUT,
		MAX9856_MIX_OUT_MXOUTR_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("LINEIN2 Switch", MAX9856_MIX_OUT,
		MAX9856_MIX_OUT_MXOUTR_SHIFT + 1, 1, 0),
	SOC_DAPM_SINGLE("LINEIN1 Switch", MAX9856_MIX_OUT,
		MAX9856_MIX_OUT_MXOUTR_SHIFT + 2, 1, 0),
	SOC_DAPM_SINGLE("MIC PGA Switch", MAX9856_MIX_OUT,
		MAX9856_MIX_OUT_MXOUTR_SHIFT + 3, 1, 0),
};

/* Left Input Mixer Switch */
static const struct snd_kcontrol_new max9856_lin_mixer_controls[] = {
	SOC_DAPM_SINGLE("MICR Switch", MAX9856_MIX_IN_L, 0, 1, 0),
	SOC_DAPM_SINGLE("MICL Switch", MAX9856_MIX_IN_L, 1, 1, 0),
	SOC_DAPM_SINGLE("LINEIN2 Switch", MAX9856_MIX_IN_L, 2, 1, 0),
	SOC_DAPM_SINGLE("LINEIN1 Switch", MAX9856_MIX_IN_L, 3, 1, 0),
	SOC_DAPM_SINGLE("AUXIN Switch", MAX9856_MIX_IN_L, 4, 1, 0),
};

/* Right Input Mixer Switch */
static const struct snd_kcontrol_new max9856_rin_mixer_controls[] = {
	SOC_DAPM_SINGLE("MICR Switch", MAX9856_MIX_IN_R, 0, 1, 0),
	SOC_DAPM_SINGLE("MICL Switch", MAX9856_MIX_IN_R, 1, 1, 0),
	SOC_DAPM_SINGLE("LINEIN2 Switch", MAX9856_MIX_IN_R, 2, 1, 0),
	SOC_DAPM_SINGLE("LINEIN1 Switch", MAX9856_MIX_IN_R, 3, 1, 0),
	SOC_DAPM_SINGLE("AUXIN Switch", MAX9856_MIX_IN_R, 4, 1, 0),
};


static const struct snd_soc_dapm_widget max9856_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DACL", "HiFi Playback", MAX9856_PM, MAX9856_PM_DALEN_SHIFT, 0),
	SND_SOC_DAPM_DAC("DACR", "HiFi Playback", MAX9856_PM, MAX9856_PM_DAREN_SHIFT, 0),
	SND_SOC_DAPM_ADC("ADCL", "HiFi Capture", MAX9856_PM, MAX9856_PM_ADLEN_SHIFT, 0),
	SND_SOC_DAPM_ADC("ADCR", "HiFi Capture", MAX9856_PM, MAX9856_PM_ADREN_SHIFT, 0),
	SND_SOC_DAPM_OUTPUT("OUTL"),
	SND_SOC_DAPM_OUTPUT("OUTR"),
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_INPUT("LINEIN1"),
	SND_SOC_DAPM_INPUT("LINEIN2"),
	SND_SOC_DAPM_INPUT("AUXIN"),
	SND_SOC_DAPM_INPUT("MICL"),
	SND_SOC_DAPM_INPUT("MICR"),
	SND_SOC_DAPM_INPUT("INR"),
	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
		&max9856_lin_mixer_controls[0],
		ARRAY_SIZE(max9856_lin_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
		&max9856_rin_mixer_controls[0],
		ARRAY_SIZE(max9856_rin_mixer_controls)),
	SND_SOC_DAPM_MIXER("Left Output Mixer", SND_SOC_NOPM, 0, 0,
		&max9856_lout_mixer_controls[0],
		ARRAY_SIZE(max9856_lout_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Output Mixer", SND_SOC_NOPM, 0, 0,
		&max9856_rout_mixer_controls[0],
		ARRAY_SIZE(max9856_rout_mixer_controls)),
};

static const struct snd_soc_dapm_route max9856_dapm_routes[] = {
	/* Left Output mixer */
	{"Left Output Mixer", "MIC PGA Switch", "MICL"},
	{"Left Output Mixer", "LINEIN1 Switch", "LINEIN1"},
	{"Left Output Mixer", "LINEIN2 Switch", "LINEIN2"},
	{"Left Output Mixer", "DAC Switch", "DACL"},

	/* Right Output mixer */
	{"Right Output Mixer", "MIC PGA Switch", "MICR"},
	{"Right Output Mixer", "LINEIN1 Switch", "LINEIN1"},
	{"Right Output Mixer", "LINEIN2 Switch", "LINEIN2"},
	{"Right Output Mixer", "DAC Switch", "DACR"},

	/* Left Input mixer */
	{"Left Input Mixer", "AUXIN Switch", "AUXIN"},
	{"Left Input Mixer", "LINEIN1 Switch", "LINEIN1"},
	{"Left Input Mixer", "LINEIN2 Switch", "LINEIN2"},
	{"Left Input Mixer", "MICL Switch", "MICL"},
	{"Left Input Mixer", "MICR Switch", "MICR"},

	/* Right Input mixer */
	{"Right Input Mixer", "AUXIN Switch", "AUXIN"},
	{"Right Input Mixer", "LINEIN1 Switch", "LINEIN1"},
	{"Right Input Mixer", "LINEIN2 Switch", "LINEIN2"},
	{"Right Input Mixer", "MICL Switch", "MICL"},
	{"Right Input Mixer", "MICR Switch", "MICR"},

	/* Output Pins */
	{"HPL", NULL, "Left Output Mixer"},
	{"HPR", NULL, "Right Output Mixer"},
	{"OUTL", NULL, "Left Output Mixer"},
	{"OUTR", NULL, "Right Output Mixer"},

	/* ADC */
	{"ADCL", NULL, "Left Input Mixer"},
	{"ADCR", NULL, "Right Input Mixer"},
};

static int max9856_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct max9856_priv *max9856 = snd_soc_codec_get_drvdata(codec);
	u64 lrclk_div;
	u8 mclk_div, clock, dac;

	if (!max9856->sysclk)
		return -EINVAL;

	/* DACNI = (65536 x 96 x LRCLK_D ) / PSCLK */
	/* PSCLK = MCLK / mclk_div */
	clock = snd_soc_read(codec, MAX9856_CLOCK);
	switch(clock & MAX9856_CLOCK_PSCLK(7)) {
	case MAX9856_CLOCK_PSCLK(1):
	case MAX9856_CLOCK_PSCLK(2):
		mclk_div = 1;
		break;
	case MAX9856_CLOCK_PSCLK(3):
	case MAX9856_CLOCK_PSCLK(4):
		mclk_div = 2;
		break;
	case MAX9856_CLOCK_PSCLK(5):
		mclk_div = 4;
		break;
	default:
		return -EINVAL;
	}

	lrclk_div = 6291456;
	lrclk_div *= params_rate(params);
	lrclk_div *= mclk_div;
	do_div(lrclk_div, max9856->sysclk);

	/* Since we use the same LRCLK for both ADC and DAC, only change DACNI */
	snd_soc_write(codec, MAX9856_DAC_INTA, (lrclk_div >> 8) & 0x7f);
	snd_soc_write(codec, MAX9856_DAC_INTB, lrclk_div & 0xff);

	switch (params_width(params)) {
	case 16:
		dac = 0;
		break;
	case 18:
	case 20:
		dac = MAX9856_DAC_SYS_WS;
		break;
	default:
		return -EINVAL;
	}
	snd_soc_update_bits(codec, MAX9856_DAC_SYS, MAX9856_DAC_SYS_WS, dac);

	return 0;
}

static int max9856_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct max9856_priv *max9856 = snd_soc_codec_get_drvdata(codec);
	u8 clock = 0;

	/* MCLK to PSCLK divider */
	if (freq <= 10000000)
		return -EINVAL;
	else if (freq <= 16000000)
		clock = MAX9856_CLOCK_PSCLK(1);
	else if (freq <= 20000000)
		clock = MAX9856_CLOCK_PSCLK(2);
	else if (freq <= 32000000)
		clock = MAX9856_CLOCK_PSCLK(3);
	else if (freq <= 40000000)
		clock = MAX9856_CLOCK_PSCLK(4);
	else if (freq <= 60000000)
		clock = MAX9856_CLOCK_PSCLK(5);
	else
		return -EINVAL;

	snd_soc_update_bits(codec, MAX9856_CLOCK, MAX9856_CLOCK_PSCLK(7), clock);

	max9856->sysclk = freq;
	return 0;
}

static int max9856_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 dac_sys = 0;
	u8 adc_sys = 0;
	u8 clock = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		clock |= MAX9856_CLOCK_MAS;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dac_sys |= MAX9856_DAC_SYS_DDLY;
		adc_sys |= MAX9856_ADC_SYS_ADLY;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		dac_sys |= MAX9856_DAC_SYS_DBCI | MAX9856_DAC_SYS_DWCI;
		adc_sys |= MAX9856_ADC_SYS_ABCI | MAX9856_ADC_SYS_AWCI;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dac_sys |= MAX9856_DAC_SYS_DBCI;
		adc_sys |= MAX9856_ADC_SYS_ABCI;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dac_sys |= MAX9856_DAC_SYS_DWCI;
		adc_sys |= MAX9856_ADC_SYS_AWCI;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, MAX9856_DAC_SYS, 0xFC, dac_sys);
	snd_soc_update_bits(codec, MAX9856_ADC_SYS, 0xC8, adc_sys);
	snd_soc_update_bits(codec, MAX9856_CLOCK, MAX9856_CLOCK_MAS, clock);

	return 0;
}

static int max9856_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct max9856_priv *max9856 = snd_soc_codec_get_drvdata(codec);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			ret = regcache_sync(max9856->regmap);
			if (ret) {
				dev_err(codec->dev,
					"Failed to sync cache: %d\n", ret);
				return ret;
			}
		}
		break;
	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

#define MAX9856_RATES SNDRV_PCM_RATE_8000_48000

#define MAX9856_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE)

static const struct snd_soc_dai_ops max9856_dai_ops = {
	.hw_params	= max9856_hw_params,
	.set_sysclk	= max9856_set_dai_sysclk,
	.set_fmt	= max9856_set_dai_fmt,
};

static struct snd_soc_dai_driver max9856_dai = {
	.name = "max9856-hifi",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = MAX9856_RATES,
		.formats = MAX9856_FORMATS
	},
	.capture = {
		.stream_name = "HiFi Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = MAX9856_RATES,
		.formats = MAX9856_FORMATS
	},
	.ops = &max9856_dai_ops,
	.symmetric_rates = 1,
};

#ifdef CONFIG_PM
static int max9856_suspend(struct snd_soc_codec *codec)
{
	max9856_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int max9856_resume(struct snd_soc_codec *codec)
{
	max9856_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}
#else
#define max9856_suspend NULL
#define max9856_resume NULL
#endif

static int max9856_probe(struct snd_soc_codec *codec)
{
	int ret;

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_REGMAP);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	snd_soc_write(codec, MAX9856_PM, 0xbf);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_max9856 = {
	.probe =	max9856_probe,
	.suspend =	max9856_suspend,
	.resume =	max9856_resume,
	.set_bias_level = max9856_set_bias_level,

	.controls = max9856_controls,
	.num_controls = ARRAY_SIZE(max9856_controls),
	.dapm_widgets = max9856_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(max9856_dapm_widgets),
	.dapm_routes = max9856_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(max9856_dapm_routes),
};

static int max9856_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct max9856_priv *max9856;
	int ret;

	max9856 = devm_kzalloc(&i2c->dev, sizeof(struct max9856_priv),
			       GFP_KERNEL);
	if (max9856 == NULL)
		return -ENOMEM;

	max9856->regmap = devm_regmap_init_i2c(i2c, &max9856_regmap);
	if (IS_ERR(max9856->regmap))
		return PTR_ERR(max9856->regmap);

	i2c_set_clientdata(i2c, max9856);

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_max9856, &max9856_dai, 1);

	return ret;
}

static int max9856_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id max9856_i2c_id[] = {
	{ "max9856", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9856_i2c_id);

static struct i2c_driver max9856_i2c_driver = {
	.driver = {
		.name = "max9856",
		.owner = THIS_MODULE,
	},
	.probe = max9856_i2c_probe,
	.remove = max9856_i2c_remove,
	.id_table = max9856_i2c_id,
};

module_i2c_driver(max9856_i2c_driver);

MODULE_AUTHOR("Christian Glindkamp <christian.glindkamp@taskit.de>");
MODULE_DESCRIPTION("ASoC MAX9856 codec driver");
MODULE_LICENSE("GPL");
