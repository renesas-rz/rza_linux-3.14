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
 * axfs_mtd.c -
 *   Allows axfs to use mtd devices or has dummy functions if mtd
 *   device support is compiled out of the kernel.
 */
#include "axfs.h"

#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/ctype.h>
#include <linux/namei.h>

#ifdef CONFIG_MTD
#include <linux/mtd/super.h>

struct mtd_info *axfs_mtd(struct super_block *sb)
{
	return (void *)sb->s_mtd;
}

struct mtd_info *axfs_mtd0(struct super_block *sb)
{
	struct axfs_super *sbi = AXFS_SB(sb);

	if (sbi->mtd0 != NULL)
		return sbi->mtd0;
	else
		return axfs_mtd(sb);
}

struct mtd_info *axfs_mtd1(struct super_block *sb)
{
	struct axfs_super *sbi = AXFS_SB(sb);

	return sbi->mtd1;
}

int axfs_has_mtd(struct super_block *sb)
{
	if (sb->s_fs_info == NULL)
		return false;

	if (axfs_mtd0(sb))
		return true;

	if (axfs_mtd1(sb))
		return true;

	if (axfs_mtd(sb))
		return true;

	return false;
}

struct mtd_info *axfs_get_mtd_device(int mtdnr)
{
	struct mtd_info *device;

	device = get_mtd_device(NULL, mtdnr);

	if (!PTR_ERR(device))
		return NULL;

	return device;
}

static int axfs_is_dev_mtd(char *path, int *mtdnr)
{
	char *off = NULL;
	char *endptr = NULL;
	char dev[] = "/dev/\0";
	char mtd[] = "mtd\0";
	char mtdblk[] = "mtdblock\0";

	if (!path || !*path)
		return false;

	off = path;

	if (strncmp(dev, off, strlen(dev)) == 0)
		off += strlen(dev);

	if (!strncmp(mtd, off, strlen(mtd)) && isdigit(off[strlen(mtd)]))
		off += strlen(mtd);

	if (!strncmp(mtdblk, off, strlen(mtdblk))
	    && isdigit(off[strlen(mtdblk)]))
		off += strlen(mtdblk);

	*mtdnr = simple_strtoul(off, &endptr, 0);

	if (!*endptr)
		return true;

	return false;
}

static struct mtd_info *axfs_get_mtd_info(struct super_block *sb, u64 fsoffset)
{
	struct axfs_super *sbi = AXFS_SB(sb);

	if (fsoffset == 0)
		return (struct mtd_info *)axfs_mtd0(sb);

	if (fsoffset < sbi->mmap_size)
		return (struct mtd_info *)axfs_mtd0(sb);

	if (axfs_mtd1(sb) != NULL)
		return (struct mtd_info *)axfs_mtd1(sb);

	return (struct mtd_info *)axfs_mtd0(sb);
}

int axfs_copy_mtd(struct super_block *sb, void *dst, u64 fsoffset, u64 len)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	u64 offset = axfs_fsoffset_to_devoffset(sbi, fsoffset);
	struct mtd_info *mtd;
	u_char *mtdbuf = (u_char *) dst;
	size_t retlen;
	int err = 0;

	if (len == 0)
		return 0;

	mtd = axfs_get_mtd_info(sb, fsoffset);
	err = mtd_read(mtd, (loff_t) offset, (size_t) len, &retlen, mtdbuf);

	if (len != retlen)
		return -EIO;

	return err;
}

/******************************************************************************
 *
 * axfs_map_mtd
 *
 * Description: When provided, uses the mtd point() capability to map allow
 *	      axfs a direct memory access to the filesystem.
 *
 * Parameters:
 *    (IN) sb - pointer to the super_block structure
 *
 * Returns:
 *    0 or error number
 *
 *****************************************************************************/
int axfs_map_mtd(struct super_block *sb)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	struct mtd_info *mtd = (struct mtd_info *)axfs_mtd0(sb);
	size_t retlen;
	int err = 0;
	void *virt;
	resource_size_t phys;


	err = mtd_point(mtd, 0, sbi->mmap_size, &retlen, &virt, &phys);
	if (err)
		return err;

	if (retlen != sbi->mmap_size) {
		mtd_unpoint(mtd, 0, retlen);
		return -EINVAL;
	}

	sbi->virt_start_addr = (unsigned long)virt;
	sbi->phys_start_addr = (unsigned long)phys;
	sbi->mtd_pointed = true;

	return 0;
}

void axfs_unmap_mtd(struct super_block *sb)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	struct mtd_info *mtd = (struct mtd_info *)axfs_mtd0(sb);

	if (!sbi)
		return;

	if (axfs_mtd1(sb))
		put_mtd_device((struct mtd_info *)axfs_mtd1(sb));

	if (axfs_is_pointed(sbi)) {
		mtd_unpoint(mtd, 0, sbi->mmap_size);
	} else {
		if (axfs_mtd0(sb))
			put_mtd_device((struct mtd_info *)axfs_mtd0(sb));
	}
}

int axfs_verify_mtd_sizes(struct super_block *sb, int *err)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	struct mtd_info *mtd0 = (struct mtd_info *)axfs_mtd0(sb);
	struct mtd_info *mtd1 = (struct mtd_info *)axfs_mtd1(sb);
	u64 io_dev_size;

	*err = 0;
	io_dev_size = axfs_get_io_dev_size(sb);

	if (!mtd0 && !mtd1)
		return false;

	/* One mtd device entirely mmaped */
	if (sbi->mtd_pointed && !io_dev_size) {
		if (sbi->mmap_size != sbi->size) {
			*err = -EINVAL;
			return false;
		}

		return true;
	}

	if (!io_dev_size)
		return false;

	/* filesystem split across two mtd devs */
	if (mtd1) {
		if (io_dev_size > mtd1->size)
			goto too_small;
		else
			return true;
	}

	/* One mtd device partially mmaped, partially io */
	if (sbi->mtd_pointed) {
		if (sbi->size > mtd0->size)
			goto too_small;
		else
			return true;
	}

	/* One mtd device as a IO dev or split with physaddr */
	if (io_dev_size > mtd0->size)
		goto too_small;

	return true;

too_small:
	printk(KERN_ERR "axfs: filesystem extends beyond end of MTD, ");
	printk(KERN_ERR "expected 0x%llx ", io_dev_size);
	printk(KERN_ERR "got 0x%llx\n", (mtd1) ? mtd1->size : mtd0->size);
	*err = -EINVAL;
	return true;
}


struct dentry *axfs_mount_mtd(struct file_system_type *fs_type, int flags,
			       const char *dev_name, struct axfs_super *sbi)
{
	int nflags, mtdnr;

	if (axfs_is_dev_mtd(sbi->second_dev, &mtdnr)) {
		sbi->mtd1 = (void *)axfs_get_mtd_device(mtdnr);
		if (!sbi->mtd1) {
			return ERR_PTR(-EINVAL);
		}
	}
	nflags = flags & MS_SILENT;

	return mount_mtd(fs_type, nflags, dev_name, sbi, axfs_fill_super);
}

void axfs_kill_mtd_super(struct super_block *sb)
{
	kill_mtd_super(sb);
}
#else /* else !CONFIG_MTD */
struct mtd_info *axfs_mtd(struct super_block *sb)
{
	return NULL;
}

struct mtd_info *axfs_mtd0(struct super_block *sb)
{
	return NULL;
}

struct mtd_info *axfs_mtd1(struct super_block *sb)
{
	return NULL;
}

int axfs_has_mtd(struct super_block *sb)
{
	return false;
}

struct mtd_info *axfs_get_mtd_device(int mtdnr)
{
	return NULL;
}

int axfs_map_mtd(struct super_block *sb)
{
	return 0;
}

void axfs_unmap_mtd(struct super_block *sb)
{
}

int axfs_copy_mtd(struct super_block *sb, void *dst, u64 fsoffset, u64 len)
{
	return -EINVAL;
}

struct dentry *axfs_mount_mtd(struct file_system_type *fs_type, int flags,
			       const char *dev_name, struct axfs_super *sbi)
{
	return NULL;
}

void axfs_kill_mtd_super(struct super_block *sb)
{
}

int axfs_verify_mtd_sizes(struct super_block *sb, int *err)
{
	*err = 0;
	return true;
}

#endif /* CONFIG_MTD */
