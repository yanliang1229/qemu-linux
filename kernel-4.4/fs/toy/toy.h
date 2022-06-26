#ifndef FS_TOY_H
#define FS_TOY_H

#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/toy_fs.h>
#include <linux/buffer_head.h>

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

/* Generic part */

typedef struct {
	block_t	*p;
	block_t	key;		/* 保存文件系统的数据块号(相对于文件系统的绝对块号?) */
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
extern int toy_getattr(struct vfsmount *, struct dentry *, struct kstat *);
extern int toy_prepare_chunk(struct page *page, loff_t pos, unsigned len);

extern void toy_truncate(struct inode *);
extern void toy_set_inode(struct inode *, dev_t);
extern unsigned toy_blocks(loff_t, struct super_block *);
extern int toy_get_block(struct inode * inode, sector_t block,
			struct buffer_head *bh, int create);

extern struct toy_dir_entry *toy_find_entry(struct dentry*, struct page**);
extern int toy_add_link(struct dentry*, struct inode*);
extern int toy_delete_entry(struct toy_dir_entry*, struct page*);
extern int toy_make_empty(struct inode*, struct inode*);
extern int toy_empty_dir(struct inode*);
extern void toy_set_link(struct toy_dir_entry*, struct page*, struct inode*);
extern struct toy_dir_entry *toy_dotdot(struct inode*, struct page**);
extern ino_t toy_inode_by_name(struct dentry*);

extern const struct inode_operations toy_file_inode_operations;
extern const struct inode_operations toy_dir_inode_operations;
extern const struct file_operations toy_file_operations;
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

#define toy_test_and_set_bit	__test_and_set_bit_le
#define toy_set_bit		__set_bit_le
#define toy_test_and_clear_bit	__test_and_clear_bit_le
#define toy_test_bit	test_bit_le
#define toy_find_first_zero_bit	find_first_zero_bit_le

#endif /* FS_TOY_H */
