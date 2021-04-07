/*
 * taotdm.c  -- ALSA SoC support for TAO3530/TDM3730
 *
 * TechNexion <linuxfae@technexion.com>
 *
 * Based on sound/soc/omap/omap3evm.c by Anuj Aggarwal
 *
 * Copyright (C) 2012 TechNexion, Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

static int taotdm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops taotdm_ops = {
	.hw_params = taotdm_hw_params,
};

#ifdef CONFIG_SND_SOC_WL1271BT
static int taotdm_wl1271bt_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set cpu DAI configuration for WL1271 Bluetooth codec */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_DSP_B |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu DAI configuration for "\
				"WL1271 Bluetooth codec\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops taotdm_wl1271bt_pcm_ops = {
	.hw_params = taotdm_wl1271bt_pcm_hw_params,
};
#endif

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link taotdm_dai[] = {
{
	.name		= "TWL4030",
	.stream_name 	= "TWL4030",
	.cpu_dai_name	= "omap-mcbsp-dai.1",
	.codec_dai_name	= "twl4030-hifi",
	.platform_name	= "omap-pcm-audio",
	.codec_name	= "twl4030-codec",
	.ops		= &taotdm_ops,
},
#ifdef CONFIG_SND_SOC_WL1271BT
{
	.name		= "WL1271BT",
	.stream_name	= "WL1271BT",
	.cpu_dai_name	= "omap-mcbsp-dai.0",
	.codec_dai_name	= "wl1271bt",
	.platform_name	= "omap-pcm-audio",
	.codec_name	= "wl1271bt-dummy-codec",
	.ops		= &taotdm_wl1271bt_pcm_ops,
},
#endif
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_taotdm = {
	.name = "taotdm",
	.dai_link = taotdm_dai,
	.num_links = ARRAY_SIZE(taotdm_dai),
};

static struct platform_device *taotdm_snd_device;

static int __init taotdm_soc_init(void)
{
	int ret;

	pr_info("TAOTDM SoC init\n");

	taotdm_snd_device = platform_device_alloc("soc-audio", -1);
	if (!taotdm_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(taotdm_snd_device, &snd_soc_taotdm);
	ret = platform_device_add(taotdm_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(taotdm_snd_device);

	return ret;
}

static void __exit taotdm_soc_exit(void)
{
	platform_device_unregister(taotdm_snd_device);
}

module_init(taotdm_soc_init);
module_exit(taotdm_soc_exit);

MODULE_AUTHOR("TechNexion <linuxfae@technexion.com>");
MODULE_DESCRIPTION("ALSA SoC support for TAO3530/TDM3730");
MODULE_LICENSE("GPL v2");
