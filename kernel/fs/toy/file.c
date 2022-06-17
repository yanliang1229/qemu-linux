/*
 *  linux/fs/toy/file.c
 *
 *  Copyright (C) 1991, 1992 Linus Torvalds
 *
 *  toy regular file handling primitives
 */

#include "toy.h"

/*
 * We have mostly NULLs here: the current defaults are OK for
 * the toy filesystem.
 */
const struct file_operations toy_file_operations = {
	.llseek		= generic_file_llseek,
	.read_iter	= generic_file_read_iter,
	.write_iter	= generic_file_write_iter,
	.mmap		= generic_file_mmap,
	.fsync		= generic_file_fsync,
	.splice_read	= generic_file_splice_read,
};

static int toy_setattr(struct dentry *dentry, struct iattr *attr)
{
	struct inode *inode = d_inode(dentry);
	int error;

	error = inode_change_ok(inode, attr);
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

const struct inode_operations toy_file_inode_operations = {
	.setattr	= toy_setattr,
	.getattr	= toy_getattr,
};
