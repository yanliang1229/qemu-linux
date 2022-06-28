/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/drivers/mmc/host/pl181.h - ARM PrimeCell MMCI PL181 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 */
#define MMCIPOWER		0x000
#define MCI_PWR_OFF		0x00
#define MCI_PWR_UP		0x02
#define MCI_PWR_ON		0x03
#define MCI_OD			(1 << 6)
#define MCI_ROD			(1 << 7)

#define MMCICLOCK		0x004
#define MCI_CLK_ENABLE		(1 << 8)
#define MCI_CLK_PWRSAVE		(1 << 9)
#define MCI_CLK_BYPASS		(1 << 10)
#define MCI_4BIT_BUS		(1 << 11)

/* Modified PL180 on Versatile Express platform */
#define MCI_ARM_HWFCEN		(1 << 12)
#define MMCIARGUMENT		0x008
/* The command register controls the Command Path State Machine (CPSM) */
#define MMCICOMMAND		0x00c
#define MCI_CPSM_RESPONSE	BIT(6)
#define MCI_CPSM_LONGRSP	BIT(7)
#define MCI_CPSM_INTERRUPT	BIT(8)
#define MCI_CPSM_PENDING	BIT(9)
#define MCI_CPSM_ENABLE		BIT(10)

#define MMCIRESPCMD		0x010
#define MMCIRESPONSE0		0x014
#define MMCIRESPONSE1		0x018
#define MMCIRESPONSE2		0x01c
#define MMCIRESPONSE3		0x020
#define MMCIDATATIMER		0x024
#define MMCIDATALENGTH		0x028

/* The data control register controls the Data Path State Machine (DPSM) */
#define MMCIDATACTRL		0x02c
#define MCI_DPSM_ENABLE		BIT(0)
#define MCI_DPSM_DIRECTION	BIT(1)
#define MCI_DPSM_MODE		BIT(2)
#define MCI_DPSM_DMAENABLE	BIT(3)
#define MCI_DPSM_BLOCKSIZE	BIT(4)

#define MMCIDATACNT		0x030
#define MMCISTATUS		0x034
#define MCI_CMDCRCFAIL		(1 << 0)
#define MCI_DATACRCFAIL		(1 << 1)
#define MCI_CMDTIMEOUT		(1 << 2)
#define MCI_DATATIMEOUT		(1 << 3)
#define MCI_TXUNDERRUN		(1 << 4)
#define MCI_RXOVERRUN		(1 << 5)
#define MCI_CMDRESPEND		(1 << 6)
#define MCI_CMDSENT		(1 << 7)
#define MCI_DATAEND		(1 << 8)
#define MCI_STARTBITERR		(1 << 9)
#define MCI_DATABLOCKEND	(1 << 10)
#define MCI_CMDACTIVE		(1 << 11)
#define MCI_TXACTIVE		(1 << 12)
#define MCI_RXACTIVE		(1 << 13)
#define MCI_TXFIFOHALFEMPTY	(1 << 14)
#define MCI_RXFIFOHALFFULL	(1 << 15)
#define MCI_TXFIFOFULL		(1 << 16)
#define MCI_RXFIFOFULL		(1 << 17)
#define MCI_TXFIFOEMPTY		(1 << 18)
#define MCI_RXFIFOEMPTY		(1 << 19)
#define MCI_TXDATAAVLBL		(1 << 20)
#define MCI_RXDATAAVLBL		(1 << 21)

#define MMCICLEAR		0x038
#define MCI_CMDCRCFAILCLR	(1 << 0)
#define MCI_DATACRCFAILCLR	(1 << 1)
#define MCI_CMDTIMEOUTCLR	(1 << 2)
#define MCI_DATATIMEOUTCLR	(1 << 3)
#define MCI_TXUNDERRUNCLR	(1 << 4)
#define MCI_RXOVERRUNCLR	(1 << 5)
#define MCI_CMDRESPENDCLR	(1 << 6)
#define MCI_CMDSENTCLR		(1 << 7)
#define MCI_DATAENDCLR		(1 << 8)
#define MCI_STARTBITERRCLR	(1 << 9)
#define MCI_DATABLOCKENDCLR	(1 << 10)

#define MMCIMASK0		0x03c
#define MCI_CMDCRCFAILMASK	(1 << 0)
#define MCI_DATACRCFAILMASK	(1 << 1)
#define MCI_CMDTIMEOUTMASK	(1 << 2)
#define MCI_DATATIMEOUTMASK	(1 << 3)
#define MCI_TXUNDERRUNMASK	(1 << 4)
#define MCI_RXOVERRUNMASK	(1 << 5)
#define MCI_CMDRESPENDMASK	(1 << 6)
#define MCI_CMDSENTMASK		(1 << 7)
#define MCI_DATAENDMASK		(1 << 8)
#define MCI_STARTBITERRMASK	(1 << 9)
#define MCI_DATABLOCKENDMASK	(1 << 10)
#define MCI_CMDACTIVEMASK	(1 << 11)
#define MCI_TXACTIVEMASK	(1 << 12)
#define MCI_RXACTIVEMASK	(1 << 13)
#define MCI_TXFIFOHALFEMPTYMASK	(1 << 14)
#define MCI_RXFIFOHALFFULLMASK	(1 << 15)
#define MCI_TXFIFOFULLMASK	(1 << 16)
#define MCI_RXFIFOFULLMASK	(1 << 17)
#define MCI_TXFIFOEMPTYMASK	(1 << 18)
#define MCI_RXFIFOEMPTYMASK	(1 << 19)
#define MCI_TXDATAAVLBLMASK	(1 << 20)
#define MCI_RXDATAAVLBLMASK	(1 << 21)

#define MMCIMASK1		0x040
#define MMCIFIFOCNT		0x048
#define MMCIFIFO		0x080 /* to 0x0bc */


#define MCI_IRQENABLE	\
	(MCI_CMDCRCFAILMASK | MCI_DATACRCFAILMASK | MCI_CMDTIMEOUTMASK | \
	MCI_DATATIMEOUTMASK | MCI_TXUNDERRUNMASK | MCI_RXOVERRUNMASK |	\
	MCI_CMDRESPENDMASK | MCI_CMDSENTMASK)

/* These interrupts are directed to IRQ1 when two IRQ lines are available */
#define MCI_IRQ_PIO_MASK \
	(MCI_RXFIFOHALFFULLMASK | MCI_RXDATAAVLBLMASK | \
	 MCI_TXFIFOHALFEMPTYMASK)

#define NR_SG		128

#define MMCI_PINCTRL_STATE_OPENDRAIN "opendrain"

struct clk;
struct dma_chan;
struct pl181_host;

struct pl181_host {
	phys_addr_t		phybase;
	void __iomem		*base;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_command	stop_abort;
	struct mmc_data		*data;
	struct mmc_host		*mmc;
	struct clk		*clk;
	struct reset_control	*rst;
	spinlock_t		lock;
	unsigned int		mclk;
	/* cached value of requested clk in set_ios */
	unsigned int		cclk;
	u32			pwr_reg;
	u32			clk_reg;
	u32			datactrl_reg;
	u32			busy_status;
	u32			mask1_reg;
	u8			vqmmc_enabled:1;
	struct pl181_platform_data *plat;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_opendrain;

	u8			hw_designer;
	u8			hw_revision:4;

	struct timer_list	timer;
	unsigned int		oldstat;

	/* pio stuff */
	struct sg_mapping_iter	sg_miter;
	unsigned int		size;

	u8			use_dma:1;
	u8			dma_in_progress:1;
	void			*dma_priv;

	s32			next_cookie;
};

#define dma_inprogress(host)	((host)->dma_in_progress)