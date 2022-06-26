// SPDX-License-Identifier: GPL-2.0
#include "toy.h"
#include <linux/buffer_head.h>
#include <linux/bitops.h>
#include <linux/sched.h>

static DEFINE_SPINLOCK(bitmap_lock);

/*
 * bitmap consists of blocks filled with 16bit words
 * bit set == busy, bit clear == free
 * endianness is a mess, but for counting zero bits it really doesn't matter...
 */
static __u32 count_free(struct buffer_head *map[], unsigned blocksize, __u32 numbits)
{
	__u32 sum = 0;
	unsigned blocks = DIV_ROUND_UP(numbits, blocksize * 8);

	while (blocks--) {
		unsigned words = blocksize / 2;
		__u16 *p = (__u16 *)(*map++)->b_data;
		while (words--)
			sum += 16 - hweight16(*p++);
	}

	return sum;
}

void toy_free_block(struct inode *inode, unsigned long block)
{
	struct super_block *sb = inode->i_sb;
	struct toy_sb_info *sbi = toy_sb(sb);
	struct buffer_head *bh;
	int k = sb->s_blocksize_bits + 3;
	unsigned long bit, zone;

	if (block < sbi->s_firstdatazone || block >= sbi->s_nzones) {
		printk("Trying to free block not in datazone\n");
		return;
	}
	zone = block - sbi->s_firstdatazone + 1;
	bit = zone & ((1<<k) - 1);
	zone >>= k;
	if (zone >= sbi->s_zmap_blocks) {
		printk("toy_free_block: nonexistent bitmap buffer\n");
		return;
	}
	bh = sbi->s_zmap[zone];
	spin_lock(&bitmap_lock);
	if (!test_and_clear_bit_le(bit, bh->b_data))
		printk("toy_free_block (%s:%lu): bit already cleared\n",
		       sb->s_id, block);
	spin_unlock(&bitmap_lock);
	mark_buffer_dirty(bh);
	return;
}

/**
 * 从文件系统的数据区中申请一个空闲的数据块，并返回其块号
 */
int toy_new_block(struct inode * inode)
{
	struct toy_sb_info *sbi = toy_sb(inode->i_sb);
	/* 每个块可以表示的位数 */
	int bits_per_zone = 8 * inode->i_sb->s_blocksize;
	int i;

	for (i = 0; i < sbi->s_zmap_blocks; i++) {
		struct buffer_head *bh = sbi->s_zmap[i];
		int j;

		spin_lock(&bitmap_lock);
		/* 找到第一个为0的位置 */
		j = find_first_zero_bit_le(bh->b_data, bits_per_zone);
		if (j < bits_per_zone) {
			/* 设置位图已经被占用 */
			set_bit_le(j, bh->b_data);
			spin_unlock(&bitmap_lock);
			mark_buffer_dirty(bh);
			/**
			 * 相对数据块号转变为整个文件系统中的绝对块号
			 */
			j += i * bits_per_zone + sbi->s_firstdatazone-1;
			if (j < sbi->s_firstdatazone || j >= sbi->s_nzones)
				break;
			return j;
		}
		spin_unlock(&bitmap_lock);
	}
	return 0;
}

unsigned long toy_count_free_blocks(struct super_block *sb)
{
	struct toy_sb_info *sbi = toy_sb(sb);
	u32 bits = sbi->s_nzones - sbi->s_firstdatazone + 1;

	return (count_free(sbi->s_zmap, sb->s_blocksize, bits)
		<< sbi->s_log_zone_size);
}

struct toy_inode *
toy_raw_inode(struct super_block *sb, ino_t ino, struct buffer_head **bh)
{
	int block;
	struct toy_sb_info *sbi = toy_sb(sb);
	struct toy_inode *p;

	if (!ino || ino > sbi->s_ninodes) {
		printk("Bad inode number on dev %s: %ld is out of range\n",
		       sb->s_id, (long)ino);
		return NULL;
	}
	ino--;
	block = 2 + sbi->s_imap_blocks + sbi->s_zmap_blocks +
		 ino / TOY_INODES_PER_BLOCK;
	*bh = sb_bread(sb, block);
	if (!*bh) {
		printk("Unable to read inode block\n");
		return NULL;
	}
	p = (void *)(*bh)->b_data;
	return p + ino % TOY_INODES_PER_BLOCK;
}

/**
 * Clear the link count and mode of a deleted inode on disk.
 */
static void toy_clear_inode(struct inode *inode)
{
	struct buffer_head *bh = NULL;

	struct toy_inode *raw_inode;
	raw_inode = toy_raw_inode(inode->i_sb, inode->i_ino, &bh);
	if (raw_inode) {
		raw_inode->i_nlinks = 0;
		raw_inode->i_mode = 0;
	}

	if (bh) {
		mark_buffer_dirty(bh);
		brelse (bh);
	}
}

void toy_free_inode(struct inode * inode)
{
	struct super_block *sb = inode->i_sb;
	struct toy_sb_info *sbi = toy_sb(inode->i_sb);
	struct buffer_head *bh;
	int k = sb->s_blocksize_bits + 3;
	unsigned long ino, bit;

	ino = inode->i_ino;
	if (ino < 1 || ino > sbi->s_ninodes) {
		printk("toy_free_inode: inode 0 or nonexistent inode\n");
		return;
	}
	bit = ino & ((1<<k) - 1);
	ino >>= k;
	if (ino >= sbi->s_imap_blocks) {
		printk("toy_free_inode: nonexistent imap in superblock\n");
		return;
	}

	toy_clear_inode(inode);	/* clear on-disk copy */

	bh = sbi->s_imap[ino];
	spin_lock(&bitmap_lock);
	if (!test_and_clear_bit_le(bit, bh->b_data))
		printk("toy_free_inode: bit %lu already cleared\n", bit);
	spin_unlock(&bitmap_lock);
	mark_buffer_dirty(bh);
}

struct inode *toy_new_inode(const struct inode *dir, umode_t mode, int *error)
{
	struct super_block *sb = dir->i_sb;
	struct toy_sb_info *sbi = toy_sb(sb);
	struct inode *inode = new_inode(sb);
	struct buffer_head * bh;
	/* 单个文件块可以管理的位图 */
	int bits_per_zone = 8 * sb->s_blocksize;
	unsigned long j;
	int i;

	if (!inode) {
		*error = -ENOMEM;
		return NULL;
	}
	j = bits_per_zone;
	bh = NULL;
	*error = -ENOSPC;
	/**
	 * 从索引节点位图中找到一个空闲的索引节点
	 */
	spin_lock(&bitmap_lock);
	for (i = 0; i < sbi->s_imap_blocks; i++) {
		bh = sbi->s_imap[i];
		j = find_first_zero_bit_le(bh->b_data, bits_per_zone);
		if (j < bits_per_zone)
			break;
	}
	if (!bh || j >= bits_per_zone) {
		spin_unlock(&bitmap_lock);
		iput(inode);
		return NULL;
	}
	if (test_and_set_bit_le(j, bh->b_data)) {	/* shouldn't happen */
		spin_unlock(&bitmap_lock);
		printk("toy_new_inode: bit already set\n");
		iput(inode);
		return NULL;
	}
	spin_unlock(&bitmap_lock);
	mark_buffer_dirty(bh);
	j += i * bits_per_zone;
	if (!j || j > sbi->s_ninodes) {
		iput(inode);
		return NULL;
	}
	inode_init_owner(inode, dir, mode);
	/**
	 * 索引节点号
	 */
	inode->i_ino = j;
	inode->i_mtime = current_time(inode);
	inode->i_atime = current_time(inode);
	inode->i_ctime = current_time(inode);
	inode->i_blocks = 0;
	memset(&toy_i(inode)->data, 0, sizeof(toy_i(inode)->data));
	insert_inode_hash(inode);
	mark_inode_dirty(inode);

	*error = 0;
	return inode;
}

unsigned long toy_count_free_inodes(struct super_block *sb)
{
	struct toy_sb_info *sbi = toy_sb(sb);
	u32 bits = sbi->s_ninodes + 1;

	return count_free(sbi->s_imap, sb->s_blocksize, bits);
}
