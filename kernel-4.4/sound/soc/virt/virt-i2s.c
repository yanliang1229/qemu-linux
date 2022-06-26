/*
 * ALSA SoC I2S Audio Layer for VIRT A53 SoC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

/* I2S registers */
#define VIRT_I2S_CS_A_REG		0x00
#define VIRT_I2S_FIFO_A_REG		0x04
#define VIRT_I2S_MODE_A_REG		0x08
#define VIRT_I2S_RXC_A_REG		0x0c
#define VIRT_I2S_TXC_A_REG		0x10
#define VIRT_I2S_DREQ_A_REG		0x14
#define VIRT_I2S_INTEN_A_REG		0x18
#define VIRT_I2S_INTSTC_A_REG		0x1c
#define VIRT_I2S_GRAY_REG		0x20

/* I2S register settings */
#define VIRT_I2S_STBY		BIT(25)
#define VIRT_I2S_SYNC		BIT(24)
#define VIRT_I2S_RXSEX		BIT(23)
#define VIRT_I2S_RXF		BIT(22)
#define VIRT_I2S_TXE		BIT(21)
#define VIRT_I2S_RXD		BIT(20)
#define VIRT_I2S_TXD		BIT(19)
#define VIRT_I2S_RXR		BIT(18)
#define VIRT_I2S_TXW		BIT(17)
#define VIRT_I2S_CS_RXERR	BIT(16)
#define VIRT_I2S_CS_TXERR	BIT(15)
#define VIRT_I2S_RXSYNC		BIT(14)
#define VIRT_I2S_TXSYNC		BIT(13)
#define VIRT_I2S_DMAEN		BIT(9)
#define VIRT_I2S_RXTHR(v)	((v) << 7)
#define VIRT_I2S_TXTHR(v)	((v) << 5)
#define VIRT_I2S_RXCLR		BIT(4)
#define VIRT_I2S_TXCLR		BIT(3)
#define VIRT_I2S_TXON		BIT(2)
#define VIRT_I2S_RXON		BIT(1)
#define VIRT_I2S_EN		(1)

#define VIRT_I2S_CLKDIS		BIT(28)
#define VIRT_I2S_PDMN		BIT(27)
#define VIRT_I2S_PDME		BIT(26)
#define VIRT_I2S_FRXP		BIT(25)
#define VIRT_I2S_FTXP		BIT(24)
#define VIRT_I2S_CLKM		BIT(23)
#define VIRT_I2S_CLKI		BIT(22)
#define VIRT_I2S_FSM		BIT(21)
#define VIRT_I2S_FSI		BIT(20)
#define VIRT_I2S_FLEN(v)	((v) << 10)
#define VIRT_I2S_FSLEN(v)	(v)

#define VIRT_I2S_CHWEX		BIT(15)
#define VIRT_I2S_CHEN		BIT(14)
#define VIRT_I2S_CHPOS(v)	((v) << 4)
#define VIRT_I2S_CHWID(v)	(v)
#define VIRT_I2S_CH1(v)		((v) << 16)
#define VIRT_I2S_CH2(v)		(v)

#define VIRT_I2S_TX_PANIC(v)	((v) << 24)
#define VIRT_I2S_RX_PANIC(v)	((v) << 16)
#define VIRT_I2S_TX(v)		((v) << 8)
#define VIRT_I2S_RX(v)		(v)

#define VIRT_I2S_INT_RXERR	BIT(3)
#define VIRT_I2S_INT_TXERR	BIT(2)
#define VIRT_I2S_INT_RXR	BIT(1)
#define VIRT_I2S_INT_TXW	BIT(0)


#define VIRT_DMA_SHIFT		(VIRT_I2S_GRAY_REG + 4)

/* General device struct */
struct virt_i2s_dev {
	struct device				*dev;
	struct snd_dmaengine_dai_dma_data	dma_data[2];
	unsigned int				fmt;
	unsigned int				bclk_ratio;

	struct regmap *i2s_regmap;
};

static void virt_i2s_clear_fifos(struct virt_i2s_dev *dev,
				    bool tx, bool rx)
{
	int timeout = 1000;
	uint32_t syncval;
	uint32_t csreg;
	uint32_t i2s_active_state;
	uint32_t off;
	uint32_t clr;

	off =  tx ? VIRT_I2S_TXON : 0;
	off |= rx ? VIRT_I2S_RXON : 0;

	clr =  tx ? VIRT_I2S_TXCLR : 0;
	clr |= rx ? VIRT_I2S_RXCLR : 0;

	/* Backup the current state */
	regmap_read(dev->i2s_regmap, VIRT_I2S_CS_A_REG, &csreg);
	i2s_active_state = csreg & (VIRT_I2S_RXON | VIRT_I2S_TXON);
	/* Stop I2S module */
	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_CS_A_REG, off, 0);
	/*
	 * Clear the FIFOs
	 * Requires at least 2 PCM clock cycles to take effect
	 */
	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_CS_A_REG, clr, clr);
	/*
	 * Toggle the SYNC flag. After 2 PCM clock cycles it can be read back
	 * FIXME: This does not seem to work for slave mode!
	 */
	regmap_read(dev->i2s_regmap, VIRT_I2S_CS_A_REG, &syncval);
	syncval &= VIRT_I2S_SYNC;

	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_CS_A_REG,
			VIRT_I2S_SYNC, ~syncval);

	/* Wait for the SYNC flag changing it's state */
	while (--timeout) {
		regmap_read(dev->i2s_regmap, VIRT_I2S_CS_A_REG, &csreg);
		if ((csreg & VIRT_I2S_SYNC) != syncval)
			break;
	}

	if (!timeout)
		dev_err(dev->dev, "I2S SYNC error!\n");

	/* Restore I2S state */
	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_CS_A_REG,
			VIRT_I2S_RXON | VIRT_I2S_TXON, i2s_active_state);
}

static int virt_i2s_set_dai_fmt(struct snd_soc_dai *dai,
				      unsigned int fmt)
{
	struct virt_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	dev->fmt = fmt;
	return 0;
}

static int virt_i2s_set_dai_bclk_ratio(struct snd_soc_dai *dai,
				      unsigned int ratio)
{
	struct virt_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	dev->bclk_ratio = ratio;
	return 0;
}

static int virt_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct virt_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	unsigned int data_length, data_delay, bclk_ratio;
	unsigned int ch1pos, ch2pos, mode, format;
	uint32_t csreg;

	/*
	 * If a stream is already enabled,
	 * the registers are already set properly.
	 */
	regmap_read(dev->i2s_regmap, VIRT_I2S_CS_A_REG, &csreg);

	if (csreg & (VIRT_I2S_TXON | VIRT_I2S_RXON))
		return 0;

	/*
	 * Adjust the data length according to the format.
	 * We prefill the half frame length with an integer
	 * divider of 2400 as explained at the clock settings.
	 * Maybe it is overwritten there, if the Integer mode
	 * does not apply.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		data_length = 16;
		bclk_ratio = 40;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data_length = 32;
		bclk_ratio = 80;
		break;
	default:
		return -EINVAL;
	}

	/* If bclk_ratio already set, use that one. */
	if (dev->bclk_ratio)
		bclk_ratio = dev->bclk_ratio;

	/* Setup the frame format */
	format = VIRT_I2S_CHEN;

	if (data_length > 24)
		format |= VIRT_I2S_CHWEX;

	format |= VIRT_I2S_CHWID((data_length-8)&0xf);

	switch (dev->fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		data_delay = 1;
		break;
	default:
		/*
		 * TODO
		 * Others are possible but are not implemented at the moment.
		 */
		dev_err(dev->dev, "%s:bad format\n", __func__);
		return -EINVAL;
	}

	ch1pos = data_delay;
	ch2pos = bclk_ratio / 2 + data_delay;

	switch (params_channels(params)) {
	case 2:
		format = VIRT_I2S_CH1(format) | VIRT_I2S_CH2(format);
		format |= VIRT_I2S_CH1(VIRT_I2S_CHPOS(ch1pos));
		format |= VIRT_I2S_CH2(VIRT_I2S_CHPOS(ch2pos));
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Set format for both streams.
	 * We cannot set another frame length
	 * (and therefore word length) anyway,
	 * so the format will be the same.
	 */
	regmap_write(dev->i2s_regmap, VIRT_I2S_RXC_A_REG, format);
	regmap_write(dev->i2s_regmap, VIRT_I2S_TXC_A_REG, format);

	/* Setup the I2S mode */
	mode = 0;

	if (data_length <= 16) {
		/*
		 * Use frame packed mode (2 channels per 32 bit word)
		 * We cannot set another frame length in the second stream
		 * (and therefore word length) anyway,
		 * so the format will be the same.
		 */
		mode |= VIRT_I2S_FTXP | VIRT_I2S_FRXP;
	}

	mode |= VIRT_I2S_FLEN(bclk_ratio - 1);
	mode |= VIRT_I2S_FSLEN(bclk_ratio / 2);

	/* Master or slave? */
	switch (dev->fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is master */
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		/*
		 * CODEC is bit clock master
		 * CPU is frame master
		 */
		mode |= VIRT_I2S_CLKM;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		/*
		 * CODEC is frame master
		 * CPU is bit clock master
		 */
		mode |= VIRT_I2S_FSM;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* CODEC is master */
		mode |= VIRT_I2S_CLKM;
		mode |= VIRT_I2S_FSM;
		break;
	default:
		dev_err(dev->dev, "%s:bad master\n", __func__);
		return -EINVAL;
	}

	/*
	 * Invert clocks?
	 *
	 * The BCM approach seems to be inverted to the classical I2S approach.
	 */
	switch (dev->fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		/* None. Therefore, both for BCM */
		mode |= VIRT_I2S_CLKI;
		mode |= VIRT_I2S_FSI;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		/* Both. Therefore, none for BCM */
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/*
		 * Invert only frame sync. Therefore,
		 * invert only bit clock for BCM
		 */
		mode |= VIRT_I2S_CLKI;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		/*
		 * Invert only bit clock. Therefore,
		 * invert only frame sync for BCM
		 */
		mode |= VIRT_I2S_FSI;
		break;
	default:
		return -EINVAL;
	}

	regmap_write(dev->i2s_regmap, VIRT_I2S_MODE_A_REG, mode);

	/* Setup the DMA parameters */
	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_CS_A_REG,
			VIRT_I2S_RXTHR(1)
			| VIRT_I2S_TXTHR(1)
			| VIRT_I2S_DMAEN, 0xffffffff);

	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_DREQ_A_REG,
			  VIRT_I2S_TX_PANIC(0x10)
			| VIRT_I2S_RX_PANIC(0x30)
			| VIRT_I2S_TX(0x30)
			| VIRT_I2S_RX(0x20), 0xffffffff);

	/* Clear FIFOs */
	virt_i2s_clear_fifos(dev, true, true);

	return 0;
}

static int virt_i2s_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct virt_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	uint32_t cs_reg;
	/*
	 * Clear both FIFOs if the one that should be started
	 * is not empty at the moment. This should only happen
	 * after overrun. Otherwise, hw_params would have cleared
	 * the FIFO.
	 */
	regmap_read(dev->i2s_regmap, VIRT_I2S_CS_A_REG, &cs_reg);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK
			&& !(cs_reg & VIRT_I2S_TXE))
		virt_i2s_clear_fifos(dev, true, false);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE
			&& (cs_reg & VIRT_I2S_RXD))
		virt_i2s_clear_fifos(dev, false, true);

	return 0;
}

static void virt_i2s_stop(struct virt_i2s_dev *dev,
		struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	uint32_t mask;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		mask = VIRT_I2S_RXON;
	else
		mask = VIRT_I2S_TXON;

	regmap_update_bits(dev->i2s_regmap,
			VIRT_I2S_CS_A_REG, mask, 0);
}

static int virt_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct virt_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	uint32_t mask;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			mask = VIRT_I2S_RXON;
		else
			mask = VIRT_I2S_TXON;

		regmap_update_bits(dev->i2s_regmap,
				VIRT_I2S_CS_A_REG, mask, mask);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		virt_i2s_stop(dev, substream, dai);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int virt_i2s_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct virt_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	if (dai->active)
		return 0;

	/* Enable PCM block */
	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_CS_A_REG,
			VIRT_I2S_EN, VIRT_I2S_EN);

	/*
	 * Disable STBY.
	 * Requires at least 4 PCM clock cycles to take effect.
	 */
	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_CS_A_REG,
			VIRT_I2S_STBY, VIRT_I2S_STBY);

	return 0;
}

static void virt_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct virt_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	virt_i2s_stop(dev, substream, dai);

	/* If both streams are stopped, disable module and clock */
	if (dai->active)
		return;

	/* Disable the module */
	regmap_update_bits(dev->i2s_regmap, VIRT_I2S_CS_A_REG,
			VIRT_I2S_EN, 0);
}

static const struct snd_soc_dai_ops virt_i2s_dai_ops = {
	.startup	= virt_i2s_startup,
	.shutdown	= virt_i2s_shutdown,
	.prepare	= virt_i2s_prepare,
	.trigger	= virt_i2s_trigger,
	.hw_params	= virt_i2s_hw_params,
	.set_fmt	= virt_i2s_set_dai_fmt,
	.set_bclk_ratio	= virt_i2s_set_dai_bclk_ratio
};

static int virt_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct virt_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,
			&dev->dma_data[SNDRV_PCM_STREAM_PLAYBACK],
			&dev->dma_data[SNDRV_PCM_STREAM_CAPTURE]);

	return 0;
}

static struct snd_soc_dai_driver virt_i2s_dai = {
	.name	= "virt-i2s",
	.probe	= virt_i2s_dai_probe,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates =	SNDRV_PCM_RATE_8000_192000,
		.formats =	SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S32_LE
		},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates =	SNDRV_PCM_RATE_8000_192000,
		.formats =	SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S32_LE
		},
	.ops = &virt_i2s_dai_ops,
	.symmetric_rates = 1
};

static bool virt_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case VIRT_I2S_CS_A_REG:
	case VIRT_I2S_FIFO_A_REG:
	case VIRT_I2S_INTSTC_A_REG:
	case VIRT_I2S_GRAY_REG:
		return true;
	default:
		return false;
	};
}

static bool virt_i2s_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case VIRT_I2S_FIFO_A_REG:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config virt_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = VIRT_I2S_GRAY_REG,
	.precious_reg = virt_i2s_precious_reg,
	.volatile_reg = virt_i2s_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

static const struct snd_soc_component_driver virt_i2s_component = {
	.name	= "virt-i2s-comp",
};

static int virt_i2s_probe(struct platform_device *pdev)
{
	struct virt_i2s_dev *dev;
	int ret;
	struct regmap *regmap;
	struct resource *mem;
	void __iomem *base;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(&pdev->dev, base,
					&virt_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev),
			   GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->i2s_regmap = regmap;

	/* Set the DMA address */
	dev->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr =
		(dma_addr_t)mem->start + VIRT_DMA_SHIFT;

	dev->dma_data[SNDRV_PCM_STREAM_CAPTURE].addr =
		(dma_addr_t)mem->start + VIRT_DMA_SHIFT;
	/* Set the bus width */
	dev->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr_width =
		DMA_SLAVE_BUSWIDTH_4_BYTES;
	dev->dma_data[SNDRV_PCM_STREAM_CAPTURE].addr_width =
		DMA_SLAVE_BUSWIDTH_4_BYTES;

	/* Set burst */
	dev->dma_data[SNDRV_PCM_STREAM_PLAYBACK].maxburst = 2;
	dev->dma_data[SNDRV_PCM_STREAM_CAPTURE].maxburst = 2;

	/* BCLK ratio - use default */
	dev->bclk_ratio = 0;

	/* Store the pdev */
	dev->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, dev);

	ret = devm_snd_soc_register_component(&pdev->dev,
			&virt_i2s_component, &virt_i2s_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI: %d\n", ret);
		return ret;
	}

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		return ret;
	}
	printk(KERN_INFO "%s probe done\n", __func__);
	return 0;
}

static const struct of_device_id virt_i2s_of_match[] = {
	{ .compatible = "virt-i2s", },
	{},
};

MODULE_DEVICE_TABLE(of, virt_i2s_of_match);

static struct platform_driver virt_i2s_driver = {
	.probe		= virt_i2s_probe,
	.driver		= {
		.name	= "virt-i2s",
		.of_match_table = virt_i2s_of_match,
	},
};

module_platform_driver(virt_i2s_driver);

MODULE_ALIAS("platform:virt-i2s");
MODULE_DESCRIPTION("VIRT I2S interface");
MODULE_AUTHOR("yanl1229 <yanl1229@163.com>");
MODULE_LICENSE("GPL v2");
