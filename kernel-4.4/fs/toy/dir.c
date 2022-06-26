/*
 *  linux/fs/toy/dir.c
 *
 *  Copyright (C) 1991, 1992 Linus Torvalds
 *
 *  toy directory handling functions
 *
 *  Updated to filesystem version 3 by Daniel Aragones
 */

#include "toy.h"
#include <linux/buffer_head.h>
#include <linux/highmem.h>
#include <linux/swap.h>

typedef struct toy_dir_entry toy_dirent;

static int toy_readdir(struct file *, struct dir_context *);

const struct file_operations toy_dir_operations = {
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.iterate	= toy_readdir,
	.fsync		= generic_file_fsync,
};

static inline void dir_put_page(struct page *page)
{
	kunmap(page);
	page_cache_release(page);
}

/*
 * Return the offset into page `page_nr' of the last valid
 * byte in that page, plus one.
 */
static unsigned
toy_last_byte(struct inode *inode, unsigned long page_nr)
{
	unsigned last_byte = PAGE_CACHE_SIZE;

	if (page_nr == (inode->i_size >> PAGE_CACHE_SHIFT))
		last_byte = inode->i_size & (PAGE_CACHE_SIZE - 1);
	return last_byte;
}

static int dir_commit_chunk(struct page *page, loff_t pos, unsigned len)
{
	struct address_space *mapping = page->mapping;
	struct inode *dir = mapping->host;
	int err = 0;
	block_write_end(NULL, mapping, pos, len, len, page, NULL);

	if (pos+len > dir->i_size) {
		i_size_write(dir, pos+len);
		mark_inode_dirty(dir);
	}
	if (IS_DIRSYNC(dir))
		err = write_one_page(page, 1);
	else
		unlock_page(page);
	return err;
}

static struct page * dir_get_page(struct inode *dir, unsigned long n)
{
	struct address_space *mapping = dir->i_mapping;
	struct page *page = read_mapping_page(mapping, n, NULL);
	if (!IS_ERR(page))
		kmap(page);
	return page;
}

static inline void *toy_next_entry(void *de, struct toy_sb_info *sbi)
{
	return (void*)((char*)de + sbi->s_dirsize);
}

static int toy_readdir(struct file *file, struct dir_context *ctx)
{
	struct inode *inode = file_inode(file);
	struct super_block *sb = inode->i_sb;
	struct toy_sb_info *sbi = toy_sb(sb);
	unsigned chunk_size = sbi->s_dirsize;
	unsigned long npages = dir_pages(inode);
	unsigned long pos = ctx->pos;
	unsigned offset;
	unsigned long n;

	ctx->pos = pos = ALIGN(pos, chunk_size);
	if (pos >= inode->i_size)
		return 0;

	offset = pos & ~PAGE_CACHE_MASK;
	n = pos >> PAGE_CACHE_SHIFT;

	for ( ; n < npages; n++, offset = 0) {
		char *p, *kaddr, *limit;
		struct page *page = dir_get_page(inode, n);

		if (IS_ERR(page))
			continue;
		kaddr = (char *)page_address(page);
		p = kaddr+offset;
		limit = kaddr + toy_last_byte(inode, n) - chunk_size;
		for ( ; p <= limit; p = toy_next_entry(p, sbi)) {
			const char *name;
			__u32 inumber;
			toy_dirent *de = (toy_dirent *)p;
			name = de->name;
			inumber = de->inode;
			if (inumber) {
				unsigned l = strnlen(name, sbi->s_namelen);
				if (!dir_emit(ctx, name, l,
					      inumber, DT_UNKNOWN)) {
					dir_put_page(page);
					return 0;
				}
			}
			ctx->pos += chunk_size;
		}
		dir_put_page(page);
	}
	return 0;
}

static inline int namecompare(int len, int maxlen,
	const char * name, const char * buffer)
{
	if (len < maxlen && buffer[len])
		return 0;
	return !memcmp(name, buffer, len);
}

/*
 *	toy_find_entry()
 *
 * finds an entry in the specified directory with the wanted name. It
 * returns the cache buffer in which the entry was found, and the entry
 * itself (as a parameter - res_dir). It does NOT read the inode of the
 * entry - you'll have to do that yourself if you want to.
 */
toy_dirent *toy_find_entry(struct dentry *dentry, struct page **res_page)
{
	const char * name = dentry->d_name.name;
	int namelen = dentry->d_name.len;
	struct inode * dir = d_inode(dentry->d_parent);
	struct super_block * sb = dir->i_sb;
	struct toy_sb_info * sbi = toy_sb(sb);
	unsigned long n;
	unsigned long npages = dir_pages(dir);
	struct page *page = NULL;
	char *p;

	char *namx;
	__u32 inumber;
	*res_page = NULL;

	for (n = 0; n < npages; n++) {
		char *kaddr, *limit;

		page = dir_get_page(dir, n);
		if (IS_ERR(page))
			continue;

		kaddr = (char*)page_address(page);
		limit = kaddr + toy_last_byte(dir, n) - sbi->s_dirsize;
		for (p = kaddr; p <= limit; p = toy_next_entry(p, sbi)) {
			toy_dirent *de = (toy_dirent *)p;
			namx = de->name;
			inumber = de->inode;
			if (!inumber)
				continue;
			if (namecompare(namelen, sbi->s_namelen, name, namx))
				goto found;
		}
		dir_put_page(page);
	}
	return NULL;

found:
	*res_page = page;
	return (toy_dirent *)p;
}

int toy_add_link(struct dentry *dentry, struct inode *inode)
{
	struct inode *dir = d_inode(dentry->d_parent);
	const char * name = dentry->d_name.name;
	int namelen = dentry->d_name.len;
	struct super_block * sb = dir->i_sb;
	struct toy_sb_info * sbi = toy_sb(sb);
	struct page *page = NULL;
	unsigned long npages = dir_pages(dir);
	unsigned long n;
	char *kaddr, *p;
	toy_dirent *de;
	loff_t pos;
	int err;
	char *namx = NULL;
	__u32 inumber;

	/*
	 * We take care of directory expansion in the same loop
	 * This code plays outside i_size, so it locks the page
	 * to protect that region.
	 */
	for (n = 0; n <= npages; n++) {
		char *limit, *dir_end;

		page = dir_get_page(dir, n);
		err = PTR_ERR(page);
		if (IS_ERR(page))
			goto out;
		lock_page(page);
		kaddr = (char*)page_address(page);
		dir_end = kaddr + toy_last_byte(dir, n);
		limit = kaddr + PAGE_CACHE_SIZE - sbi->s_dirsize;
		for (p = kaddr; p <= limit; p = toy_next_entry(p, sbi)) {
			de = (toy_dirent *)p;
  			namx = de->name;
			inumber = de->inode;
			if (p == dir_end) {
				de->inode = 0;
				goto got_it;
			}
			if (!inumber)
				goto got_it;
			err = -EEXIST;
			if (namecompare(namelen, sbi->s_namelen, name, namx))
				goto out_unlock;
		}
		unlock_page(page);
		dir_put_page(page);
	}
	BUG();
	return -EINVAL;

got_it:
	pos = page_offset(page) + p - (char *)page_address(page);
	err = toy_prepare_chunk(page, pos, sbi->s_dirsize);
	if (err)
		goto out_unlock;
	memcpy (namx, name, namelen);
	memset (namx + namelen, 0, sbi->s_dirsize - namelen - 2);
	de->inode = inode->i_ino;
	err = dir_commit_chunk(page, pos, sbi->s_dirsize);
	dir->i_mtime = dir->i_ctime = CURRENT_TIME_SEC;
	mark_inode_dirty(dir);
out_put:
	dir_put_page(page);
out:
	return err;
out_unlock:
	unlock_page(page);
	goto out_put;
}

int toy_delete_entry(struct toy_dir_entry *de, struct page *page)
{
	struct inode *inode = page->mapping->host;
	char *kaddr = page_address(page);
	loff_t pos = page_offset(page) + (char*)de - kaddr;
	struct toy_sb_info *sbi = toy_sb(inode->i_sb);
	unsigned len = sbi->s_dirsize;
	int err;

	lock_page(page);
	err = toy_prepare_chunk(page, pos, len);
	if (err == 0) {
		de->inode = 0;
		err = dir_commit_chunk(page, pos, len);
	} else {
		unlock_page(page);
	}
	dir_put_page(page);
	inode->i_ctime = inode->i_mtime = CURRENT_TIME_SEC;
	mark_inode_dirty(inode);
	return err;
}

int toy_make_empty(struct inode *inode, struct inode *dir)
{
	struct page *page = grab_cache_page(inode->i_mapping, 0);
	struct toy_sb_info *sbi = toy_sb(inode->i_sb);
	char *kaddr;
	toy_dirent *de;
	int err;

	if (!page)
		return -ENOMEM;
	err = toy_prepare_chunk(page, 0, 2 * sbi->s_dirsize);
	if (err) {
		unlock_page(page);
		goto fail;
	}

	kaddr = kmap_atomic(page);
	memset(kaddr, 0, PAGE_CACHE_SIZE);

	de = (toy_dirent *)kaddr;

	de->inode = inode->i_ino;
	strcpy(de->name, ".");
	de = toy_next_entry(de, sbi);
	de->inode = dir->i_ino;
	strcpy(de->name, "..");
	kunmap_atomic(kaddr);

	err = dir_commit_chunk(page, 0, 2 * sbi->s_dirsize);
fail:
	page_cache_release(page);
	return err;
}

/*
 * routine to check that the specified directory is empty (for rmdir)
 */
int toy_empty_dir(struct inode * inode)
{
	struct page *page = NULL;
	unsigned long i, npages = dir_pages(inode);
	struct toy_sb_info *sbi = toy_sb(inode->i_sb);
	char *name;
	__u32 inumber;

	for (i = 0; i < npages; i++) {
		char *p, *kaddr, *limit;

		page = dir_get_page(inode, i);
		if (IS_ERR(page))
			continue;

		kaddr = (char *)page_address(page);
		limit = kaddr + toy_last_byte(inode, i) - sbi->s_dirsize;
		for (p = kaddr; p <= limit; p = toy_next_entry(p, sbi)) {
			toy_dirent *de = (toy_dirent *)p;
			name = de->name;
			inumber = de->inode;

			if (inumber != 0) {
				/* check for . and .. */
				if (name[0] != '.')
					goto not_empty;
				if (!name[1]) {
					if (inumber != inode->i_ino)
						goto not_empty;
				} else if (name[1] != '.')
					goto not_empty;
				else if (name[2])
					goto not_empty;
			}
		}
		dir_put_page(page);
	}
	return 1;

not_empty:
	dir_put_page(page);
	return 0;
}

/* Releases the page */
void toy_set_link(struct toy_dir_entry *de, struct page *page,
	struct inode *inode)
{
	struct inode *dir = page->mapping->host;
	struct toy_sb_info *sbi = toy_sb(dir->i_sb);
	loff_t pos = page_offset(page) +
			(char *)de-(char*)page_address(page);
	int err;

	lock_page(page);

	err = toy_prepare_chunk(page, pos, sbi->s_dirsize);
	if (err == 0) {
		de->inode = inode->i_ino;
		err = dir_commit_chunk(page, pos, sbi->s_dirsize);
	} else {
		unlock_page(page);
	}
	dir_put_page(page);
	dir->i_mtime = dir->i_ctime = CURRENT_TIME_SEC;
	mark_inode_dirty(dir);
}

struct toy_dir_entry * toy_dotdot(struct inode *dir, struct page **p)
{
	struct page *page = dir_get_page(dir, 0);
	struct toy_sb_info *sbi = toy_sb(dir->i_sb);
	struct toy_dir_entry *de = NULL;

	if (!IS_ERR(page)) {
		de = toy_next_entry(page_address(page), sbi);
		*p = page;
	}
	return de;
}

ino_t toy_inode_by_name(struct dentry *dentry)
{
	struct page *page;
	struct toy_dir_entry *de = toy_find_entry(dentry, &page);
	ino_t res = 0;

	if (de) {
		res = de->inode;
		dir_put_page(page);
	}
	return res;
}
