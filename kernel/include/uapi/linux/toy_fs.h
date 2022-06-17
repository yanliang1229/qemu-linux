#ifndef _LINUX_TOY_FS_H
#define _LINUX_TOY_FS_H

#include <linux/types.h>
#include <linux/magic.h>

/*
 * The minix filesystem constants/structures
 */

/*
 * Thanks to Kees J Bot for sending me the definitions of the new
 * minix filesystem (aka V2) with bigger inodes and 32-bit block
 * pointers.
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
 * This is the original minix inode layout on disk.
 * Note the 8-bit gid and atime and ctime.
 */
struct toy_inode {
	__u16 i_mode;
	__u16 i_uid;
	__u32 i_size;
	__u32 i_time;
	__u8  i_gid;
	__u8  i_nlinks;
	__u16 i_zone[9];
};

/*
 * toy super-block data on disk
 */
struct toy_super_block {
	__u16 s_ninodes;
	__u16 s_nzones;
	__u16 s_imap_blocks;
	__u16 s_zmap_blocks;
	__u16 s_firstdatazone;
	__u16 s_log_zone_size;
	__u32 s_max_size;
	__u16 s_magic;
	__u16 s_state;
	__u32 s_zones;
};

struct toy_dir_entry {
	__u16 inode;
	char name[0];
};

#endif
