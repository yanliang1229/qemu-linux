/* SPDX-License-Identifier: GPL-2.0 */
#ifndef FS_MINIX_H
#define FS_MINIX_H

#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/types.h>
#include <linux/magic.h>

/*
 * The minix filesystem constants/structures
 */

#define MINIX_ROOT_INO 1

/* Not the same as the bogus LINK_MAX in <linux/limits.h>. Oh well. */
#define MINIX_LINK_MAX	250

#define MINIX_I_MAP_SLOTS	8
#define MINIX_Z_MAP_SLOTS	64
#define MINIX_VALID_FS		0x0001		/* Clean fs. */
#define MINIX_ERROR_FS		0x0002		/* fs has errors. */

#define MINIX_INODES_PER_BLOCK ((BLOCK_SIZE)/(sizeof (struct minix_inode)))

/*
 * This is the original minix inode layout on disk.
 * Note the 8-bit gid and atime and ctime.
 */
struct minix_inode {
	__u16 i_mode;			/* 文件类型和属性 */
	__u16 i_uid;			/* 文件拥有者用户id */
	__u32 i_size;			/* 文件长度(字节大小) */
	__u32 i_time;			/* 文件修改时间 */
	__u8  i_gid;			/* 文件拥有者组id */
	__u8  i_nlinks;			/* 链接数 */
	__u16 i_zone[9];		/* 文件占用的数据块管理数组 */
};

/*
 * minix super-block data on disk
 */
struct minix_super_block {
	__u16 s_ninodes;		/* inode个数*/
	__u16 s_nzones;			/* 文件系统块个数*/
	__u16 s_imap_blocks;		/* inode索引节点位图占用的块个数*/
	__u16 s_zmap_blocks;		/* 数据区位图占用的块个数 */
	__u16 s_firstdatazone;		/* 数据区中第一个块编号 */
	__u16 s_log_zone_size;		/* 块个数的对数表示 */
	__u32 s_max_size;		/* 最大文件长度 */
	__u16 s_magic;			/* 文件系统幻数 */
	__u16 s_state;			/* 文件系统挂载状态 */
};

struct minix_dir_entry {
	__u16 inode;			/* 目录项inode索引节点编号 */
	char name[0];			/* 文件名 */
};

#define INODE_VERSION(inode)	minix_sb(inode->i_sb)->s_version
#define MINIX		0x0001		/* original minix fs */

/*
 * minix fs inode data in memory
 */
struct minix_inode_info {
	__u16 data[16];
	struct inode vfs_inode;
};

/*
 * minix super-block data in memory
 */
struct minix_sb_info {
	unsigned long s_ninodes;
	unsigned long s_nzones;
	unsigned long s_imap_blocks;
	unsigned long s_zmap_blocks;
	unsigned long s_firstdatazone;
	unsigned long s_log_zone_size;
	unsigned long s_max_size;
	int s_dirsize;
	int s_namelen;
	struct buffer_head ** s_imap;
	struct buffer_head ** s_zmap;
	struct buffer_head * s_sbh;
	struct minix_super_block * s_ms;
	unsigned short s_mount_state;
	unsigned short s_version;
};

extern struct inode *minix_iget(struct super_block *, unsigned long);
extern struct minix_inode * minix_raw_inode(struct super_block *, ino_t, struct buffer_head **);
extern struct inode * minix_new_inode(const struct inode *, umode_t, int *);
extern void minix_free_inode(struct inode * inode);
extern unsigned long minix_count_free_inodes(struct super_block *sb);
extern int minix_new_block(struct inode * inode);
extern void minix_free_block(struct inode *inode, unsigned long block);
extern unsigned long minix_count_free_blocks(struct super_block *sb);
extern int minix_getattr(const struct path *, struct kstat *, u32, unsigned int);
extern int minix_prepare_chunk(struct page *page, loff_t pos, unsigned len);

extern void minix_truncate(struct inode *);
extern void minix_set_inode(struct inode *, dev_t);
extern int minix_get_block(struct inode *, sector_t, struct buffer_head *, int);
extern unsigned minix_blocks(loff_t, struct super_block *);

extern struct minix_dir_entry *minix_find_entry(struct dentry*, struct page**);
extern int minix_add_link(struct dentry*, struct inode*);
extern int minix_delete_entry(struct minix_dir_entry*, struct page*);
extern int minix_make_empty(struct inode*, struct inode*);
extern int minix_empty_dir(struct inode*);
extern void minix_set_link(struct minix_dir_entry*, struct page*, struct inode*);
extern struct minix_dir_entry *minix_dotdot(struct inode*, struct page**);
extern ino_t minix_inode_by_name(struct dentry*);

extern const struct inode_operations minix_file_inode_operations;
extern const struct inode_operations minix_dir_inode_operations;
extern const struct file_operations minix_file_operations;
extern const struct file_operations minix_dir_operations;

static inline struct minix_sb_info *minix_sb(struct super_block *sb)
{
	return sb->s_fs_info;
}

static inline struct minix_inode_info *minix_i(struct inode *inode)
{
	return container_of(inode, struct minix_inode_info, vfs_inode);
}

static inline unsigned minix_blocks_needed(unsigned bits, unsigned blocksize)
{
	return DIV_ROUND_UP(bits, blocksize * 8);
}


#define minix_test_and_set_bit	__test_and_set_bit_le
#define minix_set_bit		__set_bit_le
#define minix_test_and_clear_bit	__test_and_clear_bit_le
#define minix_test_bit	test_bit_le
#define minix_find_first_zero_bit	find_first_zero_bit_le


#endif /* FS_MINIX_H */
