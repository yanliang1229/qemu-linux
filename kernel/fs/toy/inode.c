// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include "toy.h"
#include <linux/buffer_head.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/highuid.h>
#include <linux/vfs.h>
#include <linux/writeback.h>

static void toy_evict_inode(struct inode *inode)
{
	truncate_inode_pages_final(&inode->i_data);
	if (!inode->i_nlink) {
		inode->i_size = 0;
		toy_truncate(inode);
	}
	invalidate_inode_buffers(inode);
	clear_inode(inode);
	if (!inode->i_nlink)
		toy_free_inode(inode);
}

static void toy_put_super(struct super_block *sb)
{
	int i;
	struct toy_sb_info *sbi = toy_sb(sb);

	if (!sb_rdonly(sb)) {
		sbi->s_ms->s_state = sbi->s_mount_state;
		mark_buffer_dirty(sbi->s_sbh);
	}
	for (i = 0; i < sbi->s_imap_blocks; i++)
		brelse(sbi->s_imap[i]);
	for (i = 0; i < sbi->s_zmap_blocks; i++)
		brelse(sbi->s_zmap[i]);
	brelse (sbi->s_sbh);
	kfree(sbi->s_imap);
	sb->s_fs_info = NULL;
	kfree(sbi);
}

static struct kmem_cache * toy_inode_cachep;

static struct inode *toy_alloc_inode(struct super_block *sb)
{
	struct toy_inode_info *ei;
	ei = kmem_cache_alloc(toy_inode_cachep, GFP_KERNEL);
	if (!ei)
		return NULL;
	return &ei->vfs_inode;
}

static void toy_free_in_core_inode(struct inode *inode)
{
	kmem_cache_free(toy_inode_cachep, toy_i(inode));
}

static void init_once(void *foo)
{
	struct toy_inode_info *ei = (struct toy_inode_info *) foo;

	inode_init_once(&ei->vfs_inode);
}

static int __init init_inodecache(void)
{
	toy_inode_cachep = kmem_cache_create("toy_inode_cache",
					     sizeof(struct toy_inode_info),
					     0, (SLAB_RECLAIM_ACCOUNT|
						SLAB_MEM_SPREAD|SLAB_ACCOUNT),
					     init_once);
	if (toy_inode_cachep == NULL)
		return -ENOMEM;
	return 0;
}

static void destroy_inodecache(void)
{
	/*
	 * Make sure all delayed rcu free inodes are flushed before we
	 * destroy cache.
	 */
	rcu_barrier();
	kmem_cache_destroy(toy_inode_cachep);
}

static int toy_remount(struct super_block * sb, int * flags, char * data)
{
	struct toy_sb_info * sbi = toy_sb(sb);
	struct toy_super_block * ms;

	sync_filesystem(sb);
	ms = sbi->s_ms;
	if ((bool)(*flags & SB_RDONLY) == sb_rdonly(sb))
		return 0;
	if (*flags & SB_RDONLY) {
		if (ms->s_state & TOY_VALID_FS ||
		    !(sbi->s_mount_state & TOY_VALID_FS))
			return 0;
		/* Mounting a rw partition read-only. */
		ms->s_state = sbi->s_mount_state;
		mark_buffer_dirty(sbi->s_sbh);
	} else {
	  	/* Mount a partition which is read-only, read-write. */
		sbi->s_mount_state = ms->s_state;
		ms->s_state &= ~TOY_VALID_FS;
		mark_buffer_dirty(sbi->s_sbh);

		if (!(sbi->s_mount_state & TOY_VALID_FS))
			printk("TOY-fs warning: remounting unchecked fs, "
				"running fsck is recommended\n");
		else if ((sbi->s_mount_state & TOY_ERROR_FS))
			printk("TOY-fs warning: remounting fs with errors, "
				"running fsck is recommended\n");
	}
	return 0;
}

static int toy_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	struct super_block *sb = dentry->d_sb;
	struct toy_sb_info *sbi = toy_sb(sb);
	u64 id = huge_encode_dev(sb->s_bdev->bd_dev);
	buf->f_type = sb->s_magic;
	buf->f_bsize = sb->s_blocksize;
	buf->f_blocks = (sbi->s_nzones - sbi->s_firstdatazone) << sbi->s_log_zone_size;
	buf->f_bfree = toy_count_free_blocks(sb);
	buf->f_bavail = buf->f_bfree;
	buf->f_files = sbi->s_ninodes;
	buf->f_ffree = toy_count_free_inodes(sb);
	buf->f_namelen = sbi->s_namelen;
	buf->f_fsid.val[0] = (u32)id;
	buf->f_fsid.val[1] = (u32)(id >> 32);

	return 0;
}

static struct buffer_head * toy_update_inode(struct inode * inode)
{
	struct buffer_head * bh;
	struct toy_inode * raw_inode;
	struct toy_inode_info *toy_inode = toy_i(inode);
	int i;

	raw_inode = toy_raw_inode(inode->i_sb, inode->i_ino, &bh);
	if (!raw_inode)
		return NULL;
	raw_inode->i_mode = inode->i_mode;
	raw_inode->i_uid = fs_high2lowuid(i_uid_read(inode));
	raw_inode->i_gid = fs_high2lowgid(i_gid_read(inode));
	raw_inode->i_nlinks = inode->i_nlink;
	raw_inode->i_size = inode->i_size;
	raw_inode->i_time = inode->i_mtime.tv_sec;
	if (S_ISCHR(inode->i_mode) || S_ISBLK(inode->i_mode))
		raw_inode->i_zone[0] = old_encode_dev(inode->i_rdev);
	else for (i = 0; i < 9; i++)
		raw_inode->i_zone[i] = toy_inode->data[i];
	mark_buffer_dirty(bh);
	return bh;
}

static int toy_write_inode(struct inode *inode, struct writeback_control *wbc)
{
	int err = 0;
	struct buffer_head *bh;

	bh = toy_update_inode(inode);

	if (!bh)
		return -EIO;
	if (wbc->sync_mode == WB_SYNC_ALL && buffer_dirty(bh)) {
		sync_dirty_buffer(bh);
		if (buffer_req(bh) && !buffer_uptodate(bh)) {
			printk("IO error syncing toy inode [%s:%08lx]\n",
				inode->i_sb->s_id, inode->i_ino);
			err = -EIO;
		}
	}
	brelse (bh);
	return err;
}

static const struct super_operations toy_sops = {
	.alloc_inode	= toy_alloc_inode,
	.free_inode	= toy_free_in_core_inode,
	.write_inode	= toy_write_inode,
	.evict_inode	= toy_evict_inode,
	.put_super	= toy_put_super,
	.statfs		= toy_statfs,
	.remount_fs	= toy_remount,
};

static int toy_fill_super(struct super_block *s, void *data, int silent)
{
	struct buffer_head *bh;
	struct buffer_head **map;
	struct toy_super_block *ms;
	unsigned long i, block;
	struct inode *root_inode;
	struct toy_sb_info *sbi;
	int ret = -EINVAL;

	sbi = kzalloc(sizeof(struct toy_sb_info), GFP_KERNEL);
	if (!sbi)
		return -ENOMEM;
	s->s_fs_info = sbi;

	BUILD_BUG_ON(32 != sizeof(struct toy_inode));

	/**
	 * 设置文件系统块大小
	 */
	if (!sb_set_blocksize(s, BLOCK_SIZE))
		goto out_bad_hblock;

	/**
	 * 从磁盘上读取文件系统超级块信息
	 */
	if (!(bh = sb_bread(s, 1)))
		goto out_bad_sb;

	ms = (struct toy_super_block *)bh->b_data;
	sbi->s_ms = ms;
	sbi->s_sbh = bh;
	sbi->s_mount_state = ms->s_state;
	sbi->s_ninodes = ms->s_ninodes;
	sbi->s_nzones = ms->s_nzones;
	sbi->s_imap_blocks = ms->s_imap_blocks;
	sbi->s_zmap_blocks = ms->s_zmap_blocks;
	sbi->s_firstdatazone = ms->s_firstdatazone;
	sbi->s_log_zone_size = ms->s_log_zone_size;
	sbi->s_max_size = ms->s_max_size;
	s->s_magic = ms->s_magic;
	sbi->s_version = TOY;
	sbi->s_dirsize = DIR_SIZE;
	sbi->s_namelen = NAME_LEN;
	s->s_max_links = TOY_LINK_MAX;
	/*
	 * Allocate the buffer map to keep the superblock small.
	 */
	if (sbi->s_imap_blocks == 0 || sbi->s_zmap_blocks == 0)
		goto out_illegal_sb;
	/**
	 * 计算出存放索引节点和数据块位图需要的内存长度
	 */
	i = (sbi->s_imap_blocks + sbi->s_zmap_blocks) * sizeof(bh);
	map = kzalloc(i, GFP_KERNEL);
	if (!map)
		goto out_no_map;
	/**
	 * 设置保存索引节点和数据块位图的内存指针
	 */
	sbi->s_imap = &map[0];
	sbi->s_zmap = &map[sbi->s_imap_blocks];

	block = 2;
	/**
	 * 从磁盘上读取索引节点位图信息
	 */
	for (i = 0; i < sbi->s_imap_blocks; i++) {
		if (!(sbi->s_imap[i] = sb_bread(s, block)))
			goto out_no_bitmap;
		block++;
	}
	/**
	 * 从磁盘上读取数据块位图信息
	 */
	for (i = 0; i < sbi->s_zmap_blocks; i++) {
		if (!(sbi->s_zmap[i] = sb_bread(s, block)))
			goto out_no_bitmap;
		block++;
	}

	/**
	 * 第一个块由于保存引导程序，作为保留
	 */
	set_bit_le(0, sbi->s_imap[0]->b_data);
	set_bit_le(0, sbi->s_zmap[0]->b_data);

	/* Apparently toy can create filesystems that allocate more blocks for
	 * the bitmaps than needed.  We simply ignore that, but verify it didn't
	 * create one with not enough blocks and bail out if so.
	 * 计算出保存索引节点信息需要的块数
	 */
	block = toy_blocks_needed(sbi->s_ninodes, s->s_blocksize);
	if (sbi->s_imap_blocks < block) {
		printk("TOY-fs: file system does not have enough "
				"imap blocks allocated.  Refusing to mount.\n");
		goto out_no_bitmap;
	}
	/**
	 * 计算出保存数据块信息需要的块数
	 */
	block = toy_blocks_needed(
			(sbi->s_nzones - sbi->s_firstdatazone + 1),
			s->s_blocksize);
	if (sbi->s_zmap_blocks < block) {
		printk("TOY-fs: file system does not have enough "
				"zmap blocks allocated.  Refusing to mount.\n");
		goto out_no_bitmap;
	}

	/* set up enough so that it can read an inode */
	s->s_op = &toy_sops;
	s->s_time_min = 0;
	s->s_time_max = U32_MAX;
	root_inode = toy_iget(s, TOY_ROOT_INO);
	if (IS_ERR(root_inode)) {
		ret = PTR_ERR(root_inode);
		goto out_no_root;
	}

	ret = -ENOMEM;
	s->s_root = d_make_root(root_inode);
	if (!s->s_root)
		goto out_no_root;

	if (!sb_rdonly(s)) {
		ms->s_state &= ~TOY_VALID_FS;
		mark_buffer_dirty(bh);
	}
	if (!(sbi->s_mount_state & TOY_VALID_FS))
		printk("TOY-fs: mounting unchecked file system, "
			"running fsck is recommended\n");
	else if (sbi->s_mount_state & TOY_ERROR_FS)
		printk("TOY-fs: mounting file system with errors, "
			"running fsck is recommended\n");

	return 0;

out_no_root:
	if (!silent)
		printk("TOY-fs: get root inode failed\n");
	goto out_freemap;

out_no_bitmap:
	printk("TOY-fs: bad superblock or unable to read bitmaps\n");
out_freemap:
	for (i = 0; i < sbi->s_imap_blocks; i++)
		brelse(sbi->s_imap[i]);
	for (i = 0; i < sbi->s_zmap_blocks; i++)
		brelse(sbi->s_zmap[i]);
	kfree(sbi->s_imap);
	goto out_release;

out_no_map:
	ret = -ENOMEM;
	if (!silent)
		printk("TOY-fs: can't allocate map\n");
	goto out_release;

out_illegal_sb:
	if (!silent)
		printk("TOY-fs: bad superblock\n");
	goto out_release;

out_release:
	brelse(bh);
	goto out;

out_bad_hblock:
	printk("TOY-fs: blocksize too small for device\n");
	goto out;

out_bad_sb:
	printk("TOY-fs: unable to read superblock\n");
out:
	s->s_fs_info = NULL;
	kfree(sbi);
	return ret;
}


static int toy_writepage(struct page *page, struct writeback_control *wbc)
{
	return block_write_full_page(page, toy_get_block, wbc);
}

static int toy_readpage(struct file *file, struct page *page)
{
	return block_read_full_page(page, toy_get_block);
}

int toy_prepare_chunk(struct page *page, loff_t pos, unsigned len)
{
	return __block_write_begin(page, pos, len, toy_get_block);
}

static void toy_write_failed(struct address_space *mapping, loff_t to)
{
	struct inode *inode = mapping->host;

	if (to > inode->i_size) {
		truncate_pagecache(inode, inode->i_size);
		toy_truncate(inode);
	}
}

static int toy_write_begin(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned flags,
			struct page **pagep, void **fsdata)
{
	int ret;

	ret = block_write_begin(mapping, pos, len, flags, pagep,
				toy_get_block);
	if (unlikely(ret))
		toy_write_failed(mapping, pos + len);

	return ret;
}

static sector_t toy_bmap(struct address_space *mapping, sector_t block)
{
	return generic_block_bmap(mapping,block, toy_get_block);
}

static const struct address_space_operations toy_aops = {
	.readpage = toy_readpage,
	.writepage = toy_writepage,
	.write_begin = toy_write_begin,
	.write_end = generic_write_end,
	.bmap = toy_bmap
};

static const struct inode_operations toy_symlink_inode_operations = {
	.get_link	= page_get_link,
	.getattr	= toy_getattr,
};

static struct inode_operations toy_file_inode_operations = {
	.setattr	= toy_setattr,
	.getattr	= toy_getattr,
};

static struct file_operations toy_file_operations = {
	.llseek		= generic_file_llseek,
	.read_iter	= generic_file_read_iter,
	.write_iter	= generic_file_write_iter,
	.mmap		= generic_file_mmap,
	.fsync		= generic_file_fsync,
	.splice_read	= generic_file_splice_read,
};

/**
 * 设置索引节点相关操作指针
 */
void toy_set_inode(struct inode *inode, dev_t rdev)
{
	if (S_ISREG(inode->i_mode)) { /* 普通文件 */
		inode->i_op = &toy_file_inode_operations;
		inode->i_fop = &toy_file_operations;
		inode->i_mapping->a_ops = &toy_aops;
	} else if (S_ISDIR(inode->i_mode)) {	/* 目录文件 */
		inode->i_op = &toy_dir_inode_operations;
		inode->i_fop = &toy_dir_operations;
		inode->i_mapping->a_ops = &toy_aops;
	} else if (S_ISLNK(inode->i_mode)) {	/* 符号连接 */
		inode->i_op = &toy_symlink_inode_operations;
		inode_nohighmem(inode);
		inode->i_mapping->a_ops = &toy_aops;
	} else { /* 设置文件 */
		init_special_inode(inode, inode->i_mode, rdev);
	}
}

/*
 * The global function to read an inode.
 * 获得索引节点编号为ino的文件系统的索引节点信息
 * @ino: 索引节点号
 */
struct inode *toy_iget(struct super_block *sb, unsigned long ino)
{
	struct inode *inode;
	struct buffer_head * bh;
	struct toy_inode * raw_inode;
	struct toy_inode_info *toy_inode;
	int i;

	inode = iget_locked(sb, ino);
	if (!inode)
		return ERR_PTR(-ENOMEM);
	/**
	 * I_NEW: 表示该索引节点还没有同步磁盘上对应的索引节点的信息
	 */
	if (!(inode->i_state & I_NEW))
		return inode;

	toy_inode = toy_i(inode);
	/**
	 * 从磁盘上读取索引节点信息
	 */
	raw_inode = toy_raw_inode(inode->i_sb, inode->i_ino, &bh);
	if (!raw_inode) {
		iget_failed(inode);
		return ERR_PTR(-EIO);
	}
	inode->i_mode = raw_inode->i_mode;
	i_uid_write(inode, raw_inode->i_uid);
	i_gid_write(inode, raw_inode->i_gid);
	set_nlink(inode, raw_inode->i_nlinks);
	inode->i_size = raw_inode->i_size;
	inode->i_mtime.tv_sec = raw_inode->i_time;
	inode->i_atime.tv_sec = raw_inode->i_time;
	inode->i_ctime.tv_sec = raw_inode->i_time;
	inode->i_mtime.tv_nsec = 0;
	inode->i_atime.tv_nsec = 0;
	inode->i_ctime.tv_nsec = 0;
	inode->i_blocks = 0;
	for (i = 0; i < 9; i++)
		toy_inode->data[i] = raw_inode->i_zone[i];
	toy_set_inode(inode, old_decode_dev(raw_inode->i_zone[0]));
	brelse(bh);
	unlock_new_inode(inode);
	return inode;
}

static unsigned nblocks(loff_t size, struct super_block *sb)
{
	int k = sb->s_blocksize_bits - 10;
	unsigned blocks, res, direct = DIRECT, i = DEPTH;
	blocks = (size + sb->s_blocksize - 1) >> (BLOCK_SIZE_BITS + k);
	res = blocks;
	while (--i && blocks > direct) {
		blocks -= direct;
		blocks += sb->s_blocksize/sizeof(block_t) - 1;
		blocks /= sb->s_blocksize/sizeof(block_t);
		res += blocks;
		direct = 1;
	}
	return res;
}

unsigned toy_blocks(loff_t size, struct super_block *sb)
{
	return nblocks(size, sb);
}

int toy_getattr(const struct path *path, struct kstat *stat,
		  u32 request_mask, unsigned int flags)
{
	struct super_block *sb = path->dentry->d_sb;
	struct inode *inode = d_inode(path->dentry);

	generic_fillattr(inode, stat);
	stat->blocks = (BLOCK_SIZE / 512) * toy_blocks(stat->size, sb);
	stat->blksize = sb->s_blocksize;
	return 0;
}

int toy_setattr(struct dentry *dentry, struct iattr *attr)
{
	struct inode *inode = d_inode(dentry);
	int error;

	error = setattr_prepare(dentry, attr);
	if (error)
		return error;

	if ((attr->ia_valid & ATTR_SIZE) &&
	    attr->ia_size != i_size_read(inode)) {
		error = inode_newsize_ok(inode, attr->ia_size);
		if (error)
			return error;

		truncate_setsize(inode, attr->ia_size);
		toy_truncate(inode);
	}

	setattr_copy(inode, attr);
	mark_inode_dirty(inode);
	return 0;
}

static struct dentry *toy_mount(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data)
{
	return mount_bdev(fs_type, flags, dev_name, data, toy_fill_super);
}

static struct file_system_type toy_fs_type = {
	.owner		= THIS_MODULE,
	.name		= "toy",
	.mount		= toy_mount,
	.kill_sb	= kill_block_super,
	.fs_flags	= FS_REQUIRES_DEV,
};
MODULE_ALIAS_FS("toy");

static int __init init_toy_fs(void)
{
	int err = init_inodecache();
	if (err)
		goto out1;
	err = register_filesystem(&toy_fs_type);
	if (err)
		goto out;
	return 0;
out:
	destroy_inodecache();
out1:
	return err;
}

static void __exit exit_toy_fs(void)
{
        unregister_filesystem(&toy_fs_type);
	destroy_inodecache();
}

module_init(init_toy_fs)
module_exit(exit_toy_fs)
MODULE_LICENSE("GPL");