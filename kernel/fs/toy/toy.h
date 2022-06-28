// SPDX-License-Identifier: GPL-2.0

#ifndef FS_TOY_H
#define FS_TOY_H

#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/types.h>
#include <linux/magic.h>

/*
 * The TOY filesystem constants/structures
 */

#define TOY_ROOT_INO 1

/* Not the same as the bogus LINK_MAX in <linux/limits.h>. Oh well. */
#define TOY_LINK_MAX	250

#define TOY_I_MAP_SLOTS	8
#define TOY_Z_MAP_SLOTS	64
#define TOY_VALID_FS		0x0001		/* Clean fs. */
#define TOY_ERROR_FS		0x0002		/* fs has errors. */

#define TOY_INODES_PER_BLOCK ((BLOCK_SIZE)/(sizeof (struct toy_inode)))

/*
 * This is the original toy inode layout on disk.
 * Note the 8-bit gid and atime and ctime.
 */
struct toy_inode {
	__u16 i_mode;			/* 文件类型和属性 */
	__u16 i_uid;			/* 文件拥有者用户id */
	__u32 i_size;			/* 文件长度(字节大小) */
	__u32 i_time;			/* 文件修改时间 */
	__u8  i_gid;			/* 文件拥有者组id */
	__u8  i_nlinks;			/* 链接数 */
	__u16 i_zone[9];		/* 文件占用的数据块管理数组 */
};

/*
 * toy super-block data on disk
 */
struct toy_super_block {
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


struct toy_dir_entry {
	__u16 inode;			/* 目录项inode索引节点编号 */
	char name[0];			/* 文件名 */
};

#define NAME_LEN 30
#define DIR_SIZE 32

#define INODE_VERSION(inode)	toy_sb(inode->i_sb)->s_version
#define TOY		0x0001		/* original toy fs */

/*
 * toy fs inode data in memory
 */
struct toy_inode_info {
	__u16 data[16];
	struct inode vfs_inode;
};

/*
 * toy super-block data in memory
 */
struct toy_sb_info {
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
	struct toy_super_block * s_ms;
	unsigned short s_mount_state;
	unsigned short s_version;
};

enum {DEPTH = 3, DIRECT = 7};	/* Only double indirect */

typedef u16 block_t;	/* 16 bit, host order */

typedef struct {
	block_t	*p;		/* 指向保存块号的指针 */
	block_t	key;		/* 保存块号(文件系统中的块号) */
	struct buffer_head *bh;
} Indirect;

extern struct inode *toy_iget(struct super_block *, unsigned long);
extern struct toy_inode * toy_raw_inode(struct super_block *, ino_t, struct buffer_head **);
extern struct inode * toy_new_inode(const struct inode *, umode_t, int *);
extern void toy_free_inode(struct inode * inode);
extern unsigned long toy_count_free_inodes(struct super_block *sb);
extern int toy_new_block(struct inode * inode);
extern void toy_free_block(struct inode *inode, unsigned long block);
extern unsigned long toy_count_free_blocks(struct super_block *sb);
extern int toy_getattr(const struct path *, struct kstat *, u32, unsigned int);
extern int toy_setattr(struct dentry *dentry, struct iattr *attr);
extern int toy_prepare_chunk(struct page *page, loff_t pos, unsigned len);

extern void toy_truncate(struct inode *);
extern void toy_set_inode(struct inode *, dev_t);
extern int toy_get_block(struct inode *, sector_t, struct buffer_head *, int);

extern struct toy_dir_entry *toy_find_entry(struct dentry*, struct page**);
extern int toy_add_link(struct dentry*, struct inode*);
extern int toy_delete_entry(struct toy_dir_entry*, struct page*);
extern int toy_make_empty(struct inode*, struct inode*);
extern int toy_empty_dir(struct inode*);
extern void toy_set_link(struct toy_dir_entry*, struct page*, struct inode*);
extern struct toy_dir_entry *toy_dotdot(struct inode*, struct page**);
extern ino_t toy_inode_by_name(struct dentry*);

extern const struct inode_operations toy_dir_inode_operations;
extern const struct file_operations toy_dir_operations;

static inline struct toy_sb_info *toy_sb(struct super_block *sb)
{
	return sb->s_fs_info;
}

static inline struct toy_inode_info *toy_i(struct inode *inode)
{
	return container_of(inode, struct toy_inode_info, vfs_inode);
}

static inline unsigned toy_blocks_needed(unsigned bits, unsigned blocksize)
{
	return DIV_ROUND_UP(bits, blocksize * 8);
}

#endif /* FS_TOY_H */
