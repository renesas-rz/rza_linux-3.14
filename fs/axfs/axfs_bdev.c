/*
 * Advanced XIP File System for Linux - AXFS
 *   Readonly, compressed, and XIP filesystem for Linux systems big and small
 *
 * Copyright(c) 2008 Numonyx
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Authors:
 *  Jared Hulbert <jaredeh@gmail.com>
 *
 * Project url: http://axfs.sourceforge.net
 *
 * axfs_bdev.c -
 *   Allows axfs to use block devices or has dummy functions if block
 *   device support is compiled out of the kernel.
 *
 */
#include "axfs.h"

#include <linux/mount.h>
#ifdef CONFIG_BLOCK
#include <linux/buffer_head.h>
#include <linux/namei.h>

struct block_device *axfs_bdev(struct super_block *sb)
{
	return sb->s_bdev;
}

int axfs_has_bdev(struct super_block *sb)
{
	if (axfs_bdev(sb) == NULL)
		return false;

	return true;
}

struct dentry *axfs_mount_bdev(struct file_system_type *fs_type, int flags,
		     const char *dev_name, struct axfs_super *sbi)
{
	return mount_bdev(fs_type, flags, dev_name, sbi, axfs_fill_super);
}

void axfs_kill_block_super(struct super_block *sb)
{
	kill_block_super(sb);
}

void axfs_copy_block(struct super_block *sb, void *dst_addr, u64 fsoffset,
		     u64 len)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	u64 boffset = axfs_fsoffset_to_devoffset(sbi, fsoffset);
	u64 blksize = sb->s_blocksize;
	unsigned long dst;
	unsigned long src;
	sector_t block;
	size_t bytes;
	struct buffer_head *bh;
	u64 copied = 0;

	if (len == 0)
		return;

	while (copied < len) {
		block = boffset + copied;
		block = do_div(block, blksize);
		bh = sb_bread(sb, block);
		src = (unsigned long)bh->b_data;
		dst = (unsigned long)dst_addr;
		if (copied == 0) {
			/* Explicit casting for ARM linker errors. */
			bytes = (size_t) blksize;
			bytes -= (size_t) boffset % (size_t) blksize;
			if (bytes > len)
				bytes = len;
			/* Explicit casting for ARM linker errors. */
			src += (unsigned long)boffset % (unsigned long)blksize;
		} else {
			dst += copied;
			if ((len - copied) < blksize)
				bytes = len - copied;
			else
				bytes = blksize;
		}
		memcpy((void *)dst, (void *)src, bytes);
		copied += bytes;
		brelse(bh);
	}
}

int axfs_verify_bdev_sizes(struct super_block *sb, int *err)
{
	u64 io_dev_size;
	loff_t bdev_size;

	*err = 0;

	if (!axfs_has_bdev(sb))
		return false;

	io_dev_size = axfs_get_io_dev_size(sb);

	if (!io_dev_size)
		return false;

	bdev_size = i_size_read(axfs_bdev(sb)->bd_inode);
	if (io_dev_size <= bdev_size)
		return true;

	printk(KERN_ERR "axfs: image (%lluB) doesn't fit in blkdev(%lluB)\n",
	       io_dev_size, bdev_size);
	*err = -EIO;
	return true;
}

#else

int axfs_has_bdev(struct super_block *sb)
{
	return false;
}

struct dentry *axfs_mount_bdev(struct file_system_type *fs_type, int flags,
		     const char *dev_name, struct axfs_super *sbi)
{
	return NULL;
}

void axfs_kill_block_super(struct super_block *sb)
{
}

void axfs_copy_block(struct super_block *sb, void *dst_addr, u64 fsoffset,
		     u64 len)
{
	return;
}

int axfs_verify_bdev_sizes(struct super_block *sb, int *err)
{
	*err = 0;
	return true;
}

#endif /* CONFIG_BLOCK */
