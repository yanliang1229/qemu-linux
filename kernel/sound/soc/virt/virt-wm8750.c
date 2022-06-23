/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>

#include "../codecs/wm8750.h"

#define DAI_NAME_SIZE	32

struct virt_wm8750_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *codec_clk;
	unsigned int clk_frequency;
};

static int virt_wm8750_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct virt_wm8750_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct device *dev = rtd->card->dev;
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, WM8750_SYSCLK,
				     data->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params\n");
		return ret;
	}

	return 0;
}

static const struct snd_soc_dapm_widget virt_wm8750_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static int virt_wm8750_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np;
	struct platform_device *cpu_pdev;
	struct i2c_client *codec_dev;
	struct virt_wm8750_data *data = NULL;
	struct snd_soc_dai_link_component *comp;
	int ret;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!cpu_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find CPU platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		return -EPROBE_DEFER;
	}

	comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
	if (!comp) {
		ret = -ENOMEM;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->clk_frequency = 18432000;

	data->dai.cpus		= &comp[0];
	data->dai.codecs	= &comp[1];
	data->dai.platforms	= &comp[2];

	data->dai.num_cpus	= 1;
	data->dai.num_codecs	= 1;
	data->dai.num_platforms	= 1;

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codecs->dai_name = "wm8750-hifi";
	data->dai.codecs->of_node = codec_np;
	data->dai.cpus->of_node = cpu_np;
	data->dai.platforms->of_node = cpu_np;
	data->dai.init = &virt_wm8750_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = virt_wm8750_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(virt_wm8750_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	of_node_put(cpu_np);
	of_node_put(codec_np);

	return 0;

fail:
	of_node_put(cpu_np);
	of_node_put(codec_np);

	return ret;
}

static int virt_wm8750_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct virt_wm8750_data *data = snd_soc_card_get_drvdata(card);

	clk_put(data->codec_clk);

	return 0;
}

static const struct of_device_id virt_wm8750_dt_ids[] = {
	{ .compatible = "qemu,virt-audio-wm8750", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, virt_wm8750_dt_ids);

static struct platform_driver virt_wm8750_driver = {
	.driver = {
		.name = "virt-wm8750",
		.pm = &snd_soc_pm_ops,
		.of_match_table = virt_wm8750_dt_ids,
	},
	.probe = virt_wm8750_probe,
	.remove = virt_wm8750_remove,
};
module_platform_driver(virt_wm8750_driver);

MODULE_AUTHOR("yanl1229 <yanl1229@163.com>");
MODULE_DESCRIPTION("Virt-A53 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:virt-wm8750");
