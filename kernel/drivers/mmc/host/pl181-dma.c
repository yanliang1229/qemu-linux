// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/drivers/mmc/host/pl181.c - ARM PrimeCell pl181 PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *  Copyright (C) 2010 ST-Ericsson SA
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/pm.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/amba/pl181.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/pinctrl/consumer.h>
#include <linux/reset.h>

#include <asm/div64.h>
#include <asm/io.h>

#include "pl181.h"

#define DRIVER_NAME "pl181"

#define FIFISZIE	16 *4
#define FIFIHALSIZE	8 * 4

static unsigned int fmax = 515633;


static void pl181_reg_delay(struct pl181_host *host)
{
	/*
	 * According to the spec, at least three feedback clock cycles
	 * of max 52 MHz must pass between two writes to the MMCICLOCK reg.
	 * Three MCLK clock cycles must pass between two MMCIPOWER reg writes.
	 * Worst delay time during card init is at 100 kHz => 30 us.
	 * Worst delay time when up and running is at 25 MHz => 120 ns.
	 */
	if (host->cclk < 25000000)
		udelay(30);
	else
		ndelay(120);
}

/*
 * This must be called with host->lock held
 */
static void pl181_write_clkreg(struct pl181_host *host, u32 clk)
{
	if (host->clk_reg != clk) {
		host->clk_reg = clk;
		writel(clk, host->base + MMCICLOCK);
	}
}

/*
 * This must be called with host->lock held
 */
static void pl181_write_pwrreg(struct pl181_host *host, u32 pwr)
{
	if (host->pwr_reg != pwr) {
		host->pwr_reg = pwr;
		writel(pwr, host->base + MMCIPOWER);
	}
}

/*
 * This must be called with host->lock held
 */
static void pl181_write_datactrlreg(struct pl181_host *host, u32 datactrl)
{
	if (host->datactrl_reg != datactrl) {
		host->datactrl_reg = datactrl;
		writel(datactrl, host->base + MMCIDATACTRL);
	}
}

/*
 * This must be called with host->lock held
 */
static void pl181_set_clkreg(struct pl181_host *host, unsigned int desired)
{
	u32 clk;

	/* Make sure cclk reflects the current calculated clock */
	host->cclk = 0;

	if (desired) {
		if (desired >= host->mclk) {
			clk = MCI_CLK_BYPASS;
			host->cclk = host->mclk;
		} else {
			/*
			 * PL180 TRM says f = mclk / (2 * (clkdiv + 1))
			 * => clkdiv = mclk / (2 * f) - 1
			 */
			clk = host->mclk / (2 * desired) - 1;
			if (clk >= 256)
				clk = 255;
			host->cclk = host->mclk / (2 * (clk + 1));
		}
		clk |= MCI_CLK_ENABLE;
		/* This hasn't proven to be worthwhile */
		/* clk |= MCI_CLK_PWRSAVE; */
	}

	/* Set actual clock for debug */
	host->mmc->actual_clock = host->cclk;

	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_4)
		clk |= MCI_4BIT_BUS;

	pl181_write_clkreg(host, clk);
}

/*
 * Validate mmc prerequisites
 */
static int pl181_validate_data(struct pl181_host *host,
			      struct mmc_data *data)
{
	if (!data)
		return 0;

	if (!is_power_of_2(data->blksz)) {
		dev_err(mmc_dev(host->mmc),
			"unsupported block size (%d bytes)\n", data->blksz);
		return -EINVAL;
	}

	return 0;
}

static void
pl181_request_end(struct pl181_host *host, struct mmc_request *mrq)
{
	writel(0, host->base + MMCICOMMAND);

	BUG_ON(host->data);

	host->mrq = NULL;
	host->cmd = NULL;

	mmc_request_done(host->mmc, mrq);
}

static void pl181_set_mask1(struct pl181_host *host, unsigned int mask)
{
	void __iomem *base = host->base;
	writel(mask, base + MMCIMASK1);
	host->mask1_reg = mask;
}

static void pl181_stop_data(struct pl181_host *host)
{
	pl181_write_datactrlreg(host, 0);
	pl181_set_mask1(host, 0);
	host->data = NULL;
}

static void pl181_init_sg(struct pl181_host *host, struct mmc_data *data)
{
	unsigned int flags = SG_MITER_ATOMIC;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;

	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
}

static u32 pl181_dctrl_blksz(struct pl181_host *host)
{
	return (ffs(host->data->blksz) - 1) << 4;
}

static u32 pl181_get_dctrl_cfg(struct pl181_host *host)
{
	return MCI_DPSM_ENABLE | pl181_dctrl_blksz(host);
}
/*
 * All the DMA operation mode stuff goes inside this ifdef.
 * This assumes that you have a generic DMA device interface,
 * no custom DMA interfaces are supported.
 */
#ifdef CONFIG_DMA_ENGINE
struct pl181_dmae_next {
	struct dma_async_tx_descriptor *desc;
	struct dma_chan	*chan;
};

struct pl181_dmae_priv {
	struct dma_chan	*cur_chan;
	struct dma_chan	*rx_channel;
	struct dma_chan	*tx_channel;
	struct dma_async_tx_descriptor	*desc_current;
	struct pl181_dmae_next next_data;
};


static void pl181_dma_unmap(struct pl181_host *host, struct mmc_data *data)
{
	struct pl181_dmae_priv *dmae = host->dma_priv;
	struct dma_chan *chan;

	if (data->flags & MMC_DATA_READ)
		chan = dmae->rx_channel;
	else
		chan = dmae->tx_channel;

	dma_unmap_sg(chan->device->dev, data->sg, data->sg_len,
		     mmc_get_dma_dir(data));
}

void pl181_dmae_error(struct pl181_host *host)
{
	struct pl181_dmae_priv *dmae = host->dma_priv;

	if (!dma_inprogress(host))
		return;

	dev_err(mmc_dev(host->mmc), "error during DMA transfer!\n");
	dmaengine_terminate_all(dmae->cur_chan);
	host->dma_in_progress = false;
	dmae->cur_chan = NULL;
	dmae->desc_current = NULL;
	host->data->host_cookie = 0;

	pl181_dma_unmap(host, host->data);
}

void pl181_dma_error(struct pl181_host *host)
{
	if (!host->use_dma)
		return;

	pl181_dmae_error(host);
}

void pl181_dma_release(struct pl181_host *host)
{
	struct pl181_dmae_priv *dmae = host->dma_priv;

	if (dmae->rx_channel)
		dma_release_channel(dmae->rx_channel);
	if (dmae->tx_channel)
		dma_release_channel(dmae->tx_channel);
	dmae->rx_channel = dmae->tx_channel = NULL;
	host->use_dma = false;
}

int pl181_dmae_setup(struct pl181_host *host)
{
	const char *rxname, *txname;
	struct pl181_dmae_priv *dmae;

	dmae = devm_kzalloc(mmc_dev(host->mmc), sizeof(*dmae), GFP_KERNEL);
	if (!dmae)
		return -ENOMEM;

	host->dma_priv = dmae;
	/* 申请dma传输通道 */
	dmae->rx_channel = dma_request_slave_channel(mmc_dev(host->mmc),
						     "rx");
	dmae->tx_channel = dma_request_slave_channel(mmc_dev(host->mmc),
						     "tx");

	/*
	 * If only an RX channel is specified, the driver will
	 * attempt to use it bidirectionally, however if it is
	 * is specified but cannot be located, DMA will be disabled.
	 */
	if (dmae->rx_channel && !dmae->tx_channel)
		dmae->tx_channel = dmae->rx_channel;

	if (dmae->rx_channel)
		rxname = dma_chan_name(dmae->rx_channel);
	else
		rxname = "none";

	if (dmae->tx_channel)
		txname = dma_chan_name(dmae->tx_channel);
	else
		txname = "none";

	dev_info(mmc_dev(host->mmc), "DMA channels RX %s, TX %s\n",
		 rxname, txname);

	/*
	 * Limit the maximum segment size in any SG entry according to
	 * the parameters of the DMA engine device.
	 * 设置dma 通道传输参数
	 */
	if (dmae->tx_channel) {
		struct device *dev = dmae->tx_channel->device->dev;
		unsigned int max_seg_size = dma_get_max_seg_size(dev);

		if (max_seg_size < host->mmc->max_seg_size)
			host->mmc->max_seg_size = max_seg_size;
	}
	if (dmae->rx_channel) {
		struct device *dev = dmae->rx_channel->device->dev;
		unsigned int max_seg_size = dma_get_max_seg_size(dev);

		if (max_seg_size < host->mmc->max_seg_size)
			host->mmc->max_seg_size = max_seg_size;
	}

	if (!dmae->tx_channel || !dmae->rx_channel) {
		pl181_dma_release(host);
		return -EINVAL;
	}

	return 0;
}

void pl181_dmae_finalize(struct pl181_host *host, struct mmc_data *data)
{
	struct pl181_dmae_priv *dmae = host->dma_priv;
	u32 status;
	int i;

	if (!dma_inprogress(host))
		return;

	/* Wait up to 1ms for the DMA to complete */
	for (i = 0; ; i++) {
		status = readl(host->base + MMCISTATUS);
		if (!(status & MCI_RXDATAAVLBLMASK) || i >= 100)
			break;
		udelay(10);
	}

	/*
	 * Check to see whether we still have some data left in the FIFO -
	 * this catches DMA controllers which are unable to monitor the
	 * DMALBREQ and DMALSREQ signals while allowing us to DMA to non-
	 * contiguous buffers.  On TX, we'll get a FIFO underrun error.
	 */
	if (status & MCI_RXDATAAVLBLMASK) {
		pl181_dma_error(host);
		if (!data->error)
			data->error = -EIO;
	} else if (!data->host_cookie) {
		pl181_dma_unmap(host, data);
	}

	/*
	 * Use of DMA with scatter-gather is impossible.
	 * Give up with DMA and switch back to PIO mode.
	 */
	if (status & MCI_RXDATAAVLBLMASK) {
		dev_err(mmc_dev(host->mmc), "buggy DMA detected. Taking evasive action.\n");
		pl181_dma_release(host);
	}

	host->dma_in_progress = false;
	dmae->cur_chan = NULL;
	dmae->desc_current = NULL;
}

/* prepares DMA channel and DMA descriptor, returns non-zero on failure */
static int _pl181_dmae_prep_data(struct pl181_host *host, struct mmc_data *data,
				struct dma_chan **dma_chan,
				struct dma_async_tx_descriptor **dma_desc)
{
	struct pl181_dmae_priv *dmae = host->dma_priv;
	/* 配置dma传输的通道参数 */
	struct dma_slave_config conf = {
		.src_addr = host->phybase + MMCIFIFO,
		.dst_addr = host->phybase + MMCIFIFO,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = 8, /* # of words */
		.dst_maxburst = 8, /* # of words */
		.device_fc = false,
	};
	struct dma_chan *chan;
	struct dma_device *device;
	struct dma_async_tx_descriptor *desc;
	int nr_sg;
	unsigned long flags = DMA_CTRL_ACK;

	if (data->flags & MMC_DATA_READ) {
		conf.direction = DMA_DEV_TO_MEM;
		chan = dmae->rx_channel;
	} else {
		conf.direction = DMA_MEM_TO_DEV;
		chan = dmae->tx_channel;
	}

	/* If there's no DMA channel, fall back to PIO */
	if (!chan)
		return -EINVAL;

	device = chan->device;
	nr_sg = dma_map_sg(device->dev, data->sg, data->sg_len,
			   mmc_get_dma_dir(data));
	if (nr_sg == 0)
		return -EINVAL;

	/* 设置dma通道传输参数 */
	dmaengine_slave_config(chan, &conf);
	/* 获取sg传输的dma描述符 */
	desc = dmaengine_prep_slave_sg(chan, data->sg, nr_sg,
					    conf.direction, flags);
	if (!desc)
		goto unmap_exit;

	*dma_chan = chan;
	*dma_desc = desc;

	return 0;

 unmap_exit:
	dma_unmap_sg(device->dev, data->sg, data->sg_len,
		     mmc_get_dma_dir(data));
	return -ENOMEM;
}

int pl181_dmae_prep_data(struct pl181_host *host,
			struct mmc_data *data,
			bool next)
{
	struct pl181_dmae_priv *dmae = host->dma_priv;
	struct pl181_dmae_next *nd = &dmae->next_data;

	if (!host->use_dma)
		return -EINVAL;

	if (next)
		return _pl181_dmae_prep_data(host, data, &nd->chan, &nd->desc);
	/* Check if next job is already prepared. */
	if (dmae->cur_chan && dmae->desc_current)
		return 0;

	/* No job were prepared thus do it now. */
	return _pl181_dmae_prep_data(host, data, &dmae->cur_chan,
				    &dmae->desc_current);
}

int pl181_dmae_start(struct pl181_host *host, unsigned int *datactrl)
{
	unsigned int irqmask;
	struct pl181_dmae_priv *dmae = host->dma_priv;

	host->dma_in_progress = true;
	/* 触发dma传输 */
	dmaengine_submit(dmae->desc_current);
	dma_async_issue_pending(dmae->cur_chan);
	return 0;
}

void pl181_unprep_data(struct pl181_host *host,
			   struct mmc_data *data, int err)

{
	struct pl181_dmae_priv *dmae = host->dma_priv;

	if (!host->use_dma)
		return;

	pl181_dma_unmap(host, data);

	if (err) {
		struct pl181_dmae_next *next = &dmae->next_data;
		struct dma_chan *chan;
		if (data->flags & MMC_DATA_READ)
			chan = dmae->rx_channel;
		else
			chan = dmae->tx_channel;
		dmaengine_terminate_all(chan);

		if (dmae->desc_current == next->desc)
			dmae->desc_current = NULL;

		if (dmae->cur_chan == next->chan) {
			host->dma_in_progress = false;
			dmae->cur_chan = NULL;
		}

		next->desc = NULL;
		next->chan = NULL;
	}
	data->host_cookie = 0;
}

#endif


void pl181_dma_setup(struct pl181_host *host)
{
	if (pl181_dmae_setup(host))
		return;

	host->next_cookie = 1;
	host->use_dma = true;
}

void pl181_dma_finalize(struct pl181_host *host, struct mmc_data *data)
{
	if (!host->use_dma)
		return;

	pl181_dmae_finalize(host, data);
}

int pl181_prep_data(struct pl181_host *host, struct mmc_data *data, bool next)
{
	int err;

	if (!host->use_dma)
		return -EINVAL;

	err = pl181_dmae_prep_data(host, data, next);

	if (next && !err)
		data->host_cookie = ++host->next_cookie < 0 ?
			1 : host->next_cookie;

	return err;
}

void pl181_get_next_data(struct pl181_host *host, struct mmc_data *data)
{
	struct pl181_dmae_priv *dmae = host->dma_priv;
	struct pl181_dmae_next *next = &dmae->next_data;

	WARN_ON(data->host_cookie && data->host_cookie != host->next_cookie);

	if (!host->use_dma)
		return;

	WARN_ON(!data->host_cookie && (next->desc || next->chan));

	dmae->desc_current = next->desc;
	dmae->cur_chan = next->chan;
	next->desc = NULL;
	next->chan = NULL;
}

int pl181_dma_start(struct pl181_host *host, unsigned int datactrl)
{
	struct mmc_data *data = host->data;
	int ret;

	if (!host->use_dma)
		return -EINVAL;

	/* 准备好dma传输的缓存地址 */
	ret = pl181_prep_data(host, data, false);
	if (ret)
		return ret;

	/* Okay, go for it. */
	dev_vdbg(mmc_dev(host->mmc),
		 "Submit pl181 DMA job, sglen %d blksz %04x blks %04x flags %08x\n",
		 data->sg_len, data->blksz, data->blocks, data->flags);

	/* Trigger the DMA transfer */
	datactrl |= MCI_DPSM_DMAENABLE;
	pl181_write_datactrlreg(host, datactrl);

	pl181_dmae_start(host, &datactrl);

	/*
	 * Let the pl181 say when the data is ended and it's time
	 * to fire next DMA request. When that happens, pl181 will
	 * call pl181_data_end()
	 */
	writel(readl(host->base + MMCIMASK0) | MCI_DATAENDMASK,
	       host->base + MMCIMASK0);
	return 0;
}

static void pl181_start_data(struct pl181_host *host, struct mmc_data *data)
{
	unsigned int datactrl, timeout, irqmask;
	unsigned long long clks;
	void __iomem *base;

	dev_dbg(mmc_dev(host->mmc), "blksz %04x blks %04x flags %08x\n",
		data->blksz, data->blocks, data->flags);

	host->data = data;
	host->size = data->blksz * data->blocks;
	data->bytes_xfered = 0;

	clks = (unsigned long long)data->timeout_ns * host->cclk;
	do_div(clks, NSEC_PER_SEC);

	timeout = data->timeout_clks + (unsigned int)clks;

	base = host->base;
	writel(timeout, base + MMCIDATATIMER);
	writel(host->size, base + MMCIDATALENGTH);

	datactrl = pl181_get_dctrl_cfg(host);
	datactrl |= host->data->flags & MMC_DATA_READ ? MCI_DPSM_DIRECTION : 0;

	if (host->mmc->card && mmc_card_sdio(host->mmc->card)) {
		pl181_write_clkreg(host, host->clk_reg);
	}

	/*
	 * Attempt to use DMA operation mode, if this
	 * should fail, fall back to PIO mode
	 */
	if (!pl181_dma_start(host, datactrl))
		return;

	/* IRQ mode, map the SG list for CPU reading/writing */
	pl181_init_sg(host, data);

	if (data->flags & MMC_DATA_READ) {
		irqmask = MCI_RXFIFOHALFFULLMASK;

		/*
		 * If we have less than the fifo 'half-full' threshold to
		 * transfer, trigger a PIO interrupt as soon as any data
		 * is available.
		 */
		if (host->size < FIFIHALSIZE)
			irqmask |= MCI_RXDATAAVLBLMASK;
	} else {
		/*
		 * We don't actually need to include "FIFO empty" here
		 * since its implicit in "FIFO half empty".
		 */
		irqmask = MCI_TXFIFOHALFEMPTYMASK;
	}
	/* 使能数据传输 */
	pl181_write_datactrlreg(host, datactrl);
	writel(readl(base + MMCIMASK0) & ~MCI_DATAENDMASK, base + MMCIMASK0);
	/* 打开数据传输FIFO相关中断 */
	pl181_set_mask1(host, irqmask);
}

static void
pl181_start_command(struct pl181_host *host, struct mmc_command *cmd, u32 c)
{
	void __iomem *base = host->base;

	dev_dbg(mmc_dev(host->mmc), "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + MMCICOMMAND) & MCI_CPSM_ENABLE) {
		writel(0, base + MMCICOMMAND);
		pl181_reg_delay(host);
	}

	c |= cmd->opcode | MCI_CPSM_ENABLE;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			c |=  MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP;
		else if (cmd->flags & MMC_RSP_CRC)
			c |= MCI_CPSM_RESPONSE;
		else
			c |= MCI_CPSM_RESPONSE;
	}

	host->cmd = cmd;
	/* 发送命令 */
	writel(cmd->arg, base + MMCIARGUMENT);
	writel(c, base + MMCICOMMAND);
}

static void pl181_stop_command(struct pl181_host *host)
{
	host->stop_abort.error = 0;
	pl181_start_command(host, &host->stop_abort, 0);
}

static void pl181_pre_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct pl181_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	WARN_ON(data->host_cookie);

	if (pl181_validate_data(host, data))
		return;

	pl181_prep_data(host, data, true);
}

static void pl181_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct pl181_host *host = mmc_priv(mmc);
	unsigned long flags;

	WARN_ON(host->mrq != NULL);

	mrq->cmd->error = pl181_validate_data(host, mrq->data);
	if (mrq->cmd->error) {
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	host->mrq = mrq;

	if (mrq->data)
		pl181_get_next_data(host, mrq->data);

	if (mrq->sbc)
		pl181_start_command(host, mrq->sbc, 0);
	else
		pl181_start_command(host, mrq->cmd, 0);

	if (mrq->data && (mrq->data->flags & MMC_DATA_READ))
		pl181_start_data(host, mrq->data);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void pl181_post_request(struct mmc_host *mmc, struct mmc_request *mrq,
			      int err)
{
	struct pl181_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data || !data->host_cookie)
		return;

	pl181_unprep_data(host, data, err);
}

static void
pl181_data_irq(struct pl181_host *host, struct mmc_data *data,
	      unsigned int status)
{
	unsigned int status_err;

	/* Make sure we have data to handle */
	if (!data)
		return;

	/* First check for errors */
	status_err = status & (MCI_STARTBITERR |
			       MCI_DATACRCFAIL | MCI_DATATIMEOUT |
			       MCI_TXUNDERRUN | MCI_RXOVERRUN);

	if (status_err) {
		u32 remain, success = 0;

		/* Terminate the DMA transfer */
		pl181_dma_error(host);

		dev_dbg(mmc_dev(host->mmc), "MCI ERROR IRQ, status 0x%08x at 0x%08x\n",
			status_err, success);
		if (status_err & MCI_DATACRCFAIL) {
			/* Last block was not successful */
			success -= 1;
			data->error = -EILSEQ;
		} else if (status_err & MCI_DATATIMEOUT) {
			data->error = -ETIMEDOUT;
		} else if (status_err & MCI_STARTBITERR) {
			data->error = -ECOMM;
		} else if (status_err & MCI_TXUNDERRUN) {
			data->error = -EIO;
		} else if (status_err & MCI_RXOVERRUN) {
			if (success > FIFISZIE)
				success -= FIFISZIE;
			else
				success = 0;
			data->error = -EIO;
		}
		data->bytes_xfered = round_down(success, data->blksz);
	}

	if (status & MCI_DATABLOCKEND)
		dev_err(mmc_dev(host->mmc), "stray MCI_DATABLOCKEND interrupt\n");

	if (status & MCI_DATAEND || data->error) {
		pl181_dma_finalize(host, data);

		pl181_stop_data(host);

		if (!data->error)
			/* The error clause is handled above, success! */
			data->bytes_xfered = data->blksz * data->blocks;

		if (!data->stop) {
			pl181_request_end(host, data->mrq);
		} else if (host->mrq->sbc && !data->error) {
			pl181_request_end(host, data->mrq);
		} else {
			pl181_start_command(host, data->stop, 0);
		}
	}
}

static void
pl181_cmd_irq(struct pl181_host *host, struct mmc_command *cmd,
	     unsigned int status)
{
	void __iomem *base = host->base;
	bool sbc, busy_resp;

	if (!cmd)
		return;

	sbc = (cmd == host->mrq->sbc);
	busy_resp = !!(cmd->flags & MMC_RSP_BUSY);

	/*
	 * We need to be one of these interrupts to be considered worth
	 * handling. Note that we tag on any latent IRQs postponed
	 * due to waiting for busy status.
	 */
	if (!((status|host->busy_status) &
	      (MCI_CMDCRCFAIL|MCI_CMDTIMEOUT|MCI_CMDSENT|MCI_CMDRESPEND)))
		return;

	host->cmd = NULL;

	if (status & MCI_CMDTIMEOUT) {
		cmd->error = -ETIMEDOUT; /* 命令超时 */
	} else if (status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		cmd->error = -EILSEQ; /* 命令响应错误 */
	} else {
		cmd->resp[0] = readl(base + MMCIRESPONSE0);
		cmd->resp[1] = readl(base + MMCIRESPONSE1);
		cmd->resp[2] = readl(base + MMCIRESPONSE2);
		cmd->resp[3] = readl(base + MMCIRESPONSE3);
	}

	if ((!sbc && !cmd->data) || cmd->error) {
		if (host->data) {
			/* Terminate the DMA transfer */
			pl181_dma_error(host);
			pl181_stop_data(host);
		}
		pl181_request_end(host, host->mrq);
	} else if (sbc) {
		pl181_start_command(host, host->mrq->cmd, 0);
	} else if (!(cmd->data->flags & MMC_DATA_READ)) {
		pl181_start_data(host, cmd->data);
	}
}

static int pl181_get_rx_fifocnt(struct pl181_host *host, u32 status, int remain)
{
	return remain - (readl(host->base + MMCIFIFOCNT) << 2);
}

static int pl181_pio_read(struct pl181_host *host, char *buffer, unsigned int remain)
{
	void __iomem *base = host->base;
	char *ptr = buffer;
	u32 status = readl(host->base + MMCISTATUS);
	int host_remain = host->size;

	do {
		int count = pl181_get_rx_fifocnt(host, status, host_remain);

		if (count > remain)
			count = remain;

		if (count <= 0)
			break;

		/*
		 * SDIO especially may want to send something that is
		 * not divisible by 4 (as opposed to card sectors
		 * etc). Therefore make sure to always read the last bytes
		 * while only doing full 32-bit reads towards the FIFO.
		 */
		if (unlikely(count & 0x3)) {
			if (count < 4) {
				unsigned char buf[4];
				ioread32_rep(base + MMCIFIFO, buf, 1);
				memcpy(ptr, buf, count);
			} else {
				ioread32_rep(base + MMCIFIFO, ptr, count >> 2);
				count &= ~0x3;
			}
		} else {
			ioread32_rep(base + MMCIFIFO, ptr, count >> 2);
		}

		ptr += count;
		remain -= count;
		host_remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_RXDATAAVLBL);

	return ptr - buffer;
}

static int pl181_pio_write(struct pl181_host *host, char *buffer, unsigned int remain, u32 status)
{
	void __iomem *base = host->base;
	char *ptr = buffer;

	do {
		unsigned int count, maxcnt;

		maxcnt = status & MCI_TXFIFOEMPTY ?
			 FIFISZIE : FIFIHALSIZE;
		count = min(remain, maxcnt);

		/*
		 * SDIO especially may want to send something that is
		 * not divisible by 4 (as opposed to card sectors
		 * etc), and the FIFO only accept full 32-bit writes.
		 * So compensate by adding +3 on the count, a single
		 * byte become a 32bit write, 7 bytes will be two
		 * 32bit writes etc.
		 */
		iowrite32_rep(base + MMCIFIFO, ptr, (count + 3) >> 2);

		ptr += count;
		remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_TXFIFOHALFEMPTY);

	return ptr - buffer;
}

/*
 * PIO data transfer IRQ handler.
 * 处理数据传输中与FIFO操作相关的中断
 */
static irqreturn_t pl181_pio_irq(int irq, void *dev_id)
{
	struct pl181_host *host = dev_id;
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	void __iomem *base = host->base;
	u32 status;

	status = readl(base + MMCISTATUS);

	dev_dbg(mmc_dev(host->mmc), "irq1 (pio) %08x\n", status);

	do {
		unsigned int remain, len;
		char *buffer;

		/*
		 *
		 * For read, check for data available.
		 * 检查fifo中是否有数据，如果没有，直接结束
		 */
		if (!(status & (MCI_TXFIFOHALFEMPTY|MCI_RXDATAAVLBL)))
			break;

		if (!sg_miter_next(sg_miter))
			break;

		buffer = sg_miter->addr;
		remain = sg_miter->length;

		len = 0;
		/* fifo中有可读的数据 */
		if (status & MCI_RXACTIVE)
			len = pl181_pio_read(host, buffer, remain);
		/* fifo有中待发送的数据 */
		if (status & MCI_TXACTIVE)
			len = pl181_pio_write(host, buffer, remain, status);

		sg_miter->consumed = len;

		host->size -= len;
		remain -= len;

		if (remain)
			break;

		status = readl(base + MMCISTATUS);
	} while (1);

	sg_miter_stop(sg_miter);

	/*
	 * If we have less than the fifo 'half-full' threshold to transfer,
	 * trigger a PIO interrupt as soon as any data is available.
	 */
	if (status & MCI_RXACTIVE && host->size < FIFIHALSIZE)
		pl181_set_mask1(host, MCI_RXDATAAVLBLMASK);

	/*
	 * If we run out of data, disable the data IRQs; this
	 * prevents a race where the FIFO becomes empty before
	 * the chip itself has disabled the data path, and
	 * stops us racing with our data end IRQ.
	 */
	if (host->size == 0) {
		pl181_set_mask1(host, 0);
		writel(readl(base + MMCIMASK0) | MCI_DATAENDMASK, base + MMCIMASK0);
	}

	return IRQ_HANDLED;
}

/*
 * Handle completion of command and data transfers.
 * 处理命令传输和数据传输中断
 */
static irqreturn_t pl181_irq(int irq, void *dev_id)
{
	struct pl181_host *host = dev_id;
	u32 status;
	int ret = 0;

	spin_lock(&host->lock);

	do {
		status = readl(host->base + MMCISTATUS);
		/*
		 * Busy detection is managed by pl181_cmd_irq(), including to
		 * clear the corresponding IRQ.
		 */
		status &= readl(host->base + MMCIMASK0);
		writel(status, host->base + MMCICLEAR);

		dev_dbg(mmc_dev(host->mmc), "irq0 (data+cmd) %08x\n", status);
		pl181_data_irq(host, host->data, status);
		pl181_cmd_irq(host, host->cmd, status);
		ret = 1;
	} while (status);

	spin_unlock(&host->lock);

	return IRQ_RETVAL(ret);
}

static void pl181_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct pl181_host *host = mmc_priv(mmc);
	u32 pwr = 0;
	unsigned long flags;
	int ret;

	if (host->plat->ios_handler &&
		host->plat->ios_handler(mmc_dev(mmc), ios))
			dev_err(mmc_dev(mmc), "platform ios_handler failed\n");

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

		if (!IS_ERR(mmc->supply.vqmmc) && host->vqmmc_enabled) {
			regulator_disable(mmc->supply.vqmmc);
			host->vqmmc_enabled = false;
		}

		break;
	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
		pwr |= MCI_PWR_UP;

		break;
	case MMC_POWER_ON:
		if (!IS_ERR(mmc->supply.vqmmc) && !host->vqmmc_enabled) {
			ret = regulator_enable(mmc->supply.vqmmc);
			if (ret < 0)
				dev_err(mmc_dev(mmc),
					"failed to enable vqmmc regulator\n");
			else
				host->vqmmc_enabled = true;
		}

		pwr |= MCI_PWR_ON;
		break;
	}

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		pwr |= MCI_ROD;
	spin_lock_irqsave(&host->lock, flags);
	pl181_set_clkreg(host, ios->clock);

	pl181_write_pwrreg(host, pwr);

	pl181_reg_delay(host);

	spin_unlock_irqrestore(&host->lock, flags);
}

static int pl181_get_cd(struct mmc_host *mmc)
{
	struct pl181_host *host = mmc_priv(mmc);
	struct pl181_platform_data *plat = host->plat;
	unsigned int status = mmc_gpio_get_cd(mmc);

	if (status == -ENOSYS) {
		if (!plat->status)
			return 1; /* Assume always present */

		status = plat->status(mmc_dev(host->mmc));
	}
	return status;
}

static int pl181_sig_volt_switch(struct mmc_host *mmc, struct mmc_ios *ios)
{
	int ret = 0;

	if (!IS_ERR(mmc->supply.vqmmc)) {

		switch (ios->signal_voltage) {
		case MMC_SIGNAL_VOLTAGE_330:
			ret = regulator_set_voltage(mmc->supply.vqmmc,
						2700000, 3600000);
			break;
		case MMC_SIGNAL_VOLTAGE_180:
			ret = regulator_set_voltage(mmc->supply.vqmmc,
						1700000, 1950000);
			break;
		case MMC_SIGNAL_VOLTAGE_120:
			ret = regulator_set_voltage(mmc->supply.vqmmc,
						1100000, 1300000);
			break;
		}

		if (ret)
			dev_warn(mmc_dev(mmc), "Voltage switch failed\n");
	}

	return ret;
}

static struct mmc_host_ops pl181_ops = {
	.request	= pl181_request,
	.pre_req	= pl181_pre_request,
	.post_req	= pl181_post_request,
	.set_ios	= pl181_set_ios,
	.get_ro		= mmc_gpio_get_ro,
	.get_cd		= pl181_get_cd,
	.start_signal_voltage_switch = pl181_sig_volt_switch,
};

static int pl181_of_parse(struct device_node *np, struct mmc_host *mmc)
{
	int ret = mmc_of_parse(mmc);

	if (ret)
		return ret;

	if (of_get_property(np, "mmc-cap-mmc-highspeed", NULL))
		mmc->caps |= MMC_CAP_MMC_HIGHSPEED;
	if (of_get_property(np, "mmc-cap-sd-highspeed", NULL))
		mmc->caps |= MMC_CAP_SD_HIGHSPEED;

	return 0;
}

static int pl181_probe(struct amba_device *dev,
	const struct amba_id *id)
{
	struct pl181_platform_data *plat = dev->dev.platform_data;
	struct device_node *np = dev->dev.of_node;
	struct pl181_host *host;
	struct mmc_host *mmc;
	int ret;

	/* Must have platform data or Device Tree. */
	if (!plat && !np) {
		dev_err(&dev->dev, "No plat data or DT found\n");
		return -EINVAL;
	}

	if (!plat) {
		plat = devm_kzalloc(&dev->dev, sizeof(*plat), GFP_KERNEL);
		if (!plat)
			return -ENOMEM;
	}

	mmc = mmc_alloc_host(sizeof(struct pl181_host), &dev->dev);
	if (!mmc)
		return -ENOMEM;

	ret = pl181_of_parse(np, mmc);
	if (ret)
		goto host_free;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->hw_designer = amba_manf(dev);
	host->hw_revision = amba_rev(dev);
	dev_dbg(mmc_dev(mmc), "designer ID = 0x%02x\n", host->hw_designer);
	dev_dbg(mmc_dev(mmc), "revision = 0x%01x\n", host->hw_revision);

	host->clk = devm_clk_get(&dev->dev, NULL);
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		goto host_free;
	}

	ret = clk_prepare_enable(host->clk);
	if (ret)
		goto host_free;

	host->plat = plat;
	host->mclk = clk_get_rate(host->clk);
	/*
	 * According to the spec, mclk is max 100 MHz,
	 * so we try to adjust the clock down to this,
	 * (if possible).
	 */
	if (host->mclk > 100000000) {
		ret = clk_set_rate(host->clk, 100000000);
		if (ret < 0)
			goto clk_disable;
		host->mclk = clk_get_rate(host->clk);
		dev_dbg(mmc_dev(mmc), "eventual mclk rate: %u Hz\n",
			host->mclk);
	}

	host->phybase = dev->res.start;
	host->base = devm_ioremap_resource(&dev->dev, &dev->res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto clk_disable;
	}

	mmc->f_min = DIV_ROUND_UP(host->mclk, 512);
	/*
	 * If no maximum operating frequency is supplied, fall back to use
	 * the module parameter, which has a (low) default value in case it
	 * is not specified. Either value must not exceed the clock rate into
	 * the block, of course.
	 */
	if (mmc->f_max)
		mmc->f_max = min(host->mclk, mmc->f_max);
	else
		mmc->f_max = min(host->mclk, fmax);


	dev_dbg(mmc_dev(mmc), "clocking block at %u Hz\n", mmc->f_max);

	host->rst = devm_reset_control_get_optional_exclusive(&dev->dev, NULL);
	if (IS_ERR(host->rst)) {
		ret = PTR_ERR(host->rst);
		goto clk_disable;
	}

	/* Get regulators and the supported OCR mask */
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto clk_disable;

	if (!mmc->ocr_avail)
		mmc->ocr_avail = plat->ocr_mask;
	else if (plat->ocr_mask)
		dev_warn(mmc_dev(mmc), "Platform OCR mask is ignored\n");

	/* We support these capabilities. */
	mmc->caps |= MMC_CAP_CMD23;

	/* Prepare a CMD12 - needed to clear the DPSM on some variants. */
	host->stop_abort.opcode = MMC_STOP_TRANSMISSION;
	host->stop_abort.arg = 0;
	host->stop_abort.flags = MMC_RSP_R1B | MMC_CMD_AC;

	mmc->ops = &pl181_ops;

	/* We support these PM capabilities. */
	mmc->pm_caps |= MMC_PM_KEEP_POWER;

	/*
	 * We can do SGIO
	 */
	mmc->max_segs = NR_SG;

	/*
	 * Since only a certain number of bits are valid in the data length
	 * register, we must ensure that we don't exceed 2^num-1 bytes in a
	 * single request.
	 */
	mmc->max_req_size = (1 << 16) - 1;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */
	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * Block size can be up to 2048 bytes, but must be a power of two.
	 */
	mmc->max_blk_size = 1 << 11;

	/*
	 * Limit the number of blocks transferred so that we don't overflow
	 * the maximum request size.
	 */
	mmc->max_blk_count = mmc->max_req_size >> 11;

	spin_lock_init(&host->lock);

	writel(0, host->base + MMCIMASK0);
	writel(0, host->base + MMCIMASK1);
	writel(0xfff, host->base + MMCICLEAR);

	ret = devm_request_irq(&dev->dev, dev->irq[0], pl181_irq, IRQF_SHARED,
			DRIVER_NAME " (cmd)", host);
	if (ret)
		goto clk_disable;

	if (dev->irq[1]) {
		ret = devm_request_irq(&dev->dev, dev->irq[1], pl181_pio_irq,
				IRQF_SHARED, DRIVER_NAME " (pio)", host);
		if (ret)
			goto clk_disable;
	}

	writel(MCI_IRQENABLE | MCI_STARTBITERR, host->base + MMCIMASK0);

	amba_set_drvdata(dev, mmc);

	dev_info(&dev->dev, "%s: PL%03x manf %x rev%u at 0x%08llx irq %d,%d\n",
		 mmc_hostname(mmc), amba_part(dev), amba_manf(dev),
		 amba_rev(dev), (unsigned long long)dev->res.start,
		 dev->irq[0], dev->irq[1]);

	pl181_dma_setup(host);

	pm_runtime_set_autosuspend_delay(&dev->dev, 50);
	pm_runtime_use_autosuspend(&dev->dev);

	mmc_add_host(mmc);

	pm_runtime_put(&dev->dev);
	return 0;

 clk_disable:
	clk_disable_unprepare(host->clk);
 host_free:
	mmc_free_host(mmc);
	return ret;
}

static int pl181_remove(struct amba_device *dev)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);

	if (mmc) {
		struct pl181_host *host = mmc_priv(mmc);
		/*
		 * Undo pm_runtime_put() in probe.  We use the _sync
		 * version here so that we can access the primecell.
		 */
		pm_runtime_get_sync(&dev->dev);

		mmc_remove_host(mmc);

		writel(0, host->base + MMCIMASK0);
		writel(0, host->base + MMCIMASK1);
		writel(0, host->base + MMCICOMMAND);
		writel(0, host->base + MMCIDATACTRL);

		pl181_dma_release(host);
		clk_disable_unprepare(host->clk);
		mmc_free_host(mmc);
	}

	return 0;
}

#ifdef CONFIG_PM
static void pl181_save(struct pl181_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	writel(0, host->base + MMCIMASK0);
	pl181_reg_delay(host);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void pl181_restore(struct pl181_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	writel(MCI_IRQENABLE | MCI_STARTBITERR,
	       host->base + MMCIMASK0);
	pl181_reg_delay(host);

	spin_unlock_irqrestore(&host->lock, flags);
}

static int pl181_runtime_suspend(struct device *dev)
{
	struct amba_device *adev = to_amba_device(dev);
	struct mmc_host *mmc = amba_get_drvdata(adev);

	if (mmc) {
		struct pl181_host *host = mmc_priv(mmc);
		pinctrl_pm_select_sleep_state(dev);
		pl181_save(host);
		clk_disable_unprepare(host->clk);
	}

	return 0;
}

static int pl181_runtime_resume(struct device *dev)
{
	struct amba_device *adev = to_amba_device(dev);
	struct mmc_host *mmc = amba_get_drvdata(adev);

	if (mmc) {
		struct pl181_host *host = mmc_priv(mmc);
		clk_prepare_enable(host->clk);
		pl181_restore(host);
		pinctrl_pm_select_default_state(dev);
	}

	return 0;
}
#endif

static const struct dev_pm_ops pl181_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(pl181_runtime_suspend, pl181_runtime_resume, NULL)
};

static const struct amba_id pl181_ids[] = {
	{
		.id	= 0x00041181,
		.mask	= 0x000fffff,
	},
	{ 0, 0 },
};

MODULE_DEVICE_TABLE(amba, pl181_ids);

static struct amba_driver pl181_driver = {
	.drv		= {
		.name	= DRIVER_NAME,
		.pm	= &pl181_dev_pm_ops,
	},
	.probe		= pl181_probe,
	.remove		= pl181_remove,
	.id_table	= pl181_ids,
};

module_amba_driver(pl181_driver);

module_param(fmax, uint, 0444);

MODULE_DESCRIPTION("ARM PrimeCell PL181 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
