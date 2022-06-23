/* SPDX-License-Identifier: GPL-2.0-only */
/* include/linux/amba/PL080.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      http://armlinux.simtec.co.uk/
 *      Ben Dooks <ben@simtec.co.uk>
 *
 * ARM PrimeCell PL080 DMA controller
*/

/* Note, there are some Samsung updates to this controller block which
 * make it not entierly compatible with the PL080 specification from
 * ARM. When in doubt, check the Samsung documentation first.
 *
 * The Samsung defines are PL080S, and add an extra control register,
 * the ability to move more than 2^11 counts of data and some extra
 * OneNAND features.
*/

#ifndef ASM_PL080_H
#define ASM_PL080_H

#define PL080_INT_STATUS			(0x00)
#define PL080_TC_STATUS				(0x04)
#define PL080_TC_CLEAR				(0x08)
#define PL080_ERR_STATUS			(0x0C)
#define PL080_ERR_CLEAR				(0x10)
#define PL080_RAW_TC_STATUS			(0x14)
#define PL080_RAW_ERR_STATUS			(0x18)
#define PL080_EN_CHAN				(0x1c)
#define PL080_SOFT_BREQ				(0x20)
#define PL080_SOFT_SREQ				(0x24)
#define PL080_SOFT_LBREQ			(0x28)
#define PL080_SOFT_LSREQ			(0x2C)

#define PL080_CONFIG				(0x30)
#define PL080_CONFIG_M2_BE			BIT(2)
#define PL080_CONFIG_M1_BE			BIT(1)
#define PL080_CONFIG_ENABLE			BIT(0)

#define PL080_SYNC				(0x34)

/* Per channel configuration registers */
#define PL080_Cx_BASE(x)			((0x100 + (x * 0x20)))
#define PL080_CH_SRC_ADDR			(0x00)
#define PL080_CH_DST_ADDR			(0x04)
#define PL080_CH_LLI				(0x08)
#define PL080_CH_CONTROL			(0x0C)
#define PL080_CH_CONFIG				(0x10)

#define PL080_LLI_ADDR_MASK			GENMASK(31, 2)
#define PL080_LLI_ADDR_SHIFT			(2)
#define PL080_LLI_LM_AHB2			BIT(0)

#define PL080_CONTROL_TC_IRQ_EN			BIT(31)
#define PL080_CONTROL_PROT_MASK			GENMASK(30, 28)
#define PL080_CONTROL_PROT_SHIFT		(28)
#define PL080_CONTROL_PROT_CACHE		BIT(30)
#define PL080_CONTROL_PROT_BUFF			BIT(29)
#define PL080_CONTROL_PROT_SYS			BIT(28)
#define PL080_CONTROL_DST_INCR			BIT(27)
#define PL080_CONTROL_SRC_INCR			BIT(26)
#define PL080_CONTROL_DST_AHB2			BIT(25)
#define PL080_CONTROL_SRC_AHB2			BIT(24)
#define PL080_CONTROL_DWIDTH_MASK		GENMASK(23, 21)
#define PL080_CONTROL_DWIDTH_SHIFT		(21)
#define PL080_CONTROL_SWIDTH_MASK		GENMASK(20, 18)
#define PL080_CONTROL_SWIDTH_SHIFT		(18)
#define PL080_CONTROL_DB_SIZE_MASK		GENMASK(17, 15)
#define PL080_CONTROL_DB_SIZE_SHIFT		(15)
#define PL080_CONTROL_SB_SIZE_MASK		GENMASK(14, 12)
#define PL080_CONTROL_SB_SIZE_SHIFT		(12)
#define PL080_CONTROL_TRANSFER_SIZE_MASK	GENMASK(11, 0)
#define PL080_CONTROL_TRANSFER_SIZE_SHIFT	(0)

#define PL080_BSIZE_1				(0x0)
#define PL080_BSIZE_4				(0x1)
#define PL080_BSIZE_8				(0x2)
#define PL080_BSIZE_16				(0x3)
#define PL080_BSIZE_32				(0x4)
#define PL080_BSIZE_64				(0x5)
#define PL080_BSIZE_128				(0x6)
#define PL080_BSIZE_256				(0x7)

#define PL080_WIDTH_8BIT			(0x0)
#define PL080_WIDTH_16BIT			(0x1)
#define PL080_WIDTH_32BIT			(0x2)

#define PL080N_CONFIG_ITPROT			BIT(20)
#define PL080N_CONFIG_SECPROT			BIT(19)
#define PL080_CONFIG_HALT			BIT(18)
#define PL080_CONFIG_ACTIVE			BIT(17)  /* RO */
#define PL080_CONFIG_LOCK			BIT(16)
#define PL080_CONFIG_TC_IRQ_MASK		BIT(15)
#define PL080_CONFIG_ERR_IRQ_MASK		BIT(14)
#define PL080_CONFIG_FLOW_CONTROL_MASK		GENMASK(13, 11)
#define PL080_CONFIG_FLOW_CONTROL_SHIFT		(11)
#define PL080_CONFIG_DST_SEL_MASK		GENMASK(9, 6)
#define PL080_CONFIG_DST_SEL_SHIFT		(6)
#define PL080_CONFIG_SRC_SEL_MASK		GENMASK(4, 1)
#define PL080_CONFIG_SRC_SEL_SHIFT		(1)
#define PL080_CONFIG_ENABLE			BIT(0)

#define PL080_FLOW_MEM2MEM			(0x0)
#define PL080_FLOW_MEM2PER			(0x1)
#define PL080_FLOW_PER2MEM			(0x2)
#define PL080_FLOW_SRC2DST			(0x3)
#define PL080_FLOW_SRC2DST_DST			(0x4)
#define PL080_FLOW_MEM2PER_PER			(0x5)
#define PL080_FLOW_PER2MEM_PER			(0x6)
#define PL080_FLOW_SRC2DST_SRC			(0x7)

/* DMA linked list chain structure */

struct pl080_lli {
	u32	src_addr;
	u32	dst_addr;
	u32	next_lli;
	u32	control0;
};

/* We need sizes of structs from this header */
#include <linux/dmaengine.h>
#include <linux/interrupt.h>

struct PL080_driver_data;
struct PL080_phy_chan;
struct PL080_txd;

/* Bitmasks for selecting AHB ports for DMA transfers */
enum {
	PL080_AHB1 = (1 << 0),
	PL080_AHB2 = (1 << 1)
};

/**
 * struct PL080_channel_data - data structure to pass info between
 * platform and PL08x driver regarding channel configuration
 * @bus_id: name of this device channel, not just a device name since
 * devices may have more than one channel e.g. "foo_tx"
 * @min_signal: the minimum DMA signal number to be muxed in for this
 * channel (for platforms supporting muxed signals). If you have
 * static assignments, make sure this is set to the assigned signal
 * number, PL08x have 16 possible signals in number 0 thru 15 so
 * when these are not enough they often get muxed (in hardware)
 * disabling simultaneous use of the same channel for two devices.
 * @max_signal: the maximum DMA signal number to be muxed in for
 * the channel. Set to the same as min_signal for
 * devices with static assignments
 * @muxval: a number usually used to poke into some mux regiser to
 * mux in the signal to this channel
 * @addr: source/target address in physical memory for this DMA channel,
 * can be the address of a FIFO register for burst requests for example.
 * This can be left undefined if the PrimeCell API is used for configuring
 * this.
 * @single: the device connected to this channel will request single DMA
 * transfers, not bursts. (Bursts are default.)
 * @periph_buses: the device connected to this channel is accessible via
 * these buses (use PL080_AHB1 | PL080_AHB2).
 */
struct pl080_channel_data {
	const char *bus_id;
	int min_signal;
	int max_signal;
	u32 muxval;
	dma_addr_t addr;
	bool single;
	u8 periph_buses;
};

enum pl080_burst_size {
	PL080_BURST_SZ_1,
	PL080_BURST_SZ_4,
	PL080_BURST_SZ_8,
	PL080_BURST_SZ_16,
	PL080_BURST_SZ_32,
	PL080_BURST_SZ_64,
	PL080_BURST_SZ_128,
	PL080_BURST_SZ_256,
};

enum pl080_bus_width {
	PL080_BUS_WIDTH_8_BITS,
	PL080_BUS_WIDTH_16_BITS,
	PL080_BUS_WIDTH_32_BITS,
};

/**
 * struct PL080_platform_data - the platform configuration for the PL08x
 * PrimeCells.
 * @slave_channels: the channels defined for the different devices on the
 * platform, all inclusive, including multiplexed channels. The available
 * physical channels will be multiplexed around these signals as they are
 * requested, just enumerate all possible channels.
 * @num_slave_channels: number of elements in the slave channel array
 * @memcpy_burst_size: the appropriate burst size for memcpy operations
 * @memcpy_bus_width: memory bus width
 * @memcpy_prot_buff: whether memcpy DMA is bufferable
 * @memcpy_prot_cache: whether memcpy DMA is cacheable
 * @get_xfer_signal: request a physical signal to be used for a DMA transfer
 * immediately: if there is some multiplexing or similar blocking the use
 * of the channel the transfer can be denied by returning less than zero,
 * else it returns the allocated signal number
 * @put_xfer_signal: indicate to the platform that this physical signal is not
 * running any DMA transfer and multiplexing can be recycled
 * @lli_buses: buses which LLIs can be fetched from: PL080_AHB1 | PL080_AHB2
 * @mem_buses: buses which memory can be accessed from: PL080_AHB1 | PL080_AHB2
 * @slave_map: DMA slave matching table
 * @slave_map_len: number of elements in @slave_map
 */
struct pl080_platform_data {
	struct pl080_channel_data *slave_channels;
	unsigned int num_slave_channels;
	enum pl080_burst_size memcpy_burst_size;
	enum pl080_bus_width memcpy_bus_width;
	bool memcpy_prot_buff;
	bool memcpy_prot_cache;
	int (*get_xfer_signal)(const struct pl080_channel_data *);
	void (*put_xfer_signal)(const struct pl080_channel_data *, int);
	u8 lli_buses;
	u8 mem_buses;
	const struct dma_slave_map *slave_map;
	int slave_map_len;
};

#ifdef CONFIG_AMBA_PL08X
bool pl080_filter_id(struct dma_chan *chan, void *chan_id);
#else
static inline bool pl080_filter_id(struct dma_chan *chan, void *chan_id)
{
	return false;
}
#endif

#endif /* ASM_PL080_H */
