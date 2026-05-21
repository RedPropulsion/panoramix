/*
 * Copyright (c) 2024 Panoramix
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(file_logger, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>

#include <ff.h>
#include <string.h>
#include <stdio.h>

#include "file_logger.h"

static FATFS fatfs_fs;
static struct fs_mount_t fat_fs_mnt = {
	.type = FS_FATFS,
	.fs_data = &fatfs_fs,
	.mnt_point = "/SD:"
};

static bool sd_mounted;
static bool is_initialized;

int file_logger_init(void)
{
	if (is_initialized) {
		return 0;
	}

	LOG_INF("Initializing file logger...");

	if (disk_access_init("SD") != 0) {
		LOG_ERR("Failed to initialize storage");
		return -EIO;
	}

	int ret = fs_mount(&fat_fs_mnt);
	if (ret == 0) {
		sd_mounted = true;
		LOG_INF("SD card mounted");
	} else if (ret == -EBUSY) {
		sd_mounted = true;
		LOG_INF("SD card already mounted");
	} else {
		LOG_ERR("Failed to mount SD card: %d", ret);
		return ret;
	}

	is_initialized = true;
	return 0;
}

bool file_logger_is_mounted(void)
{
	return sd_mounted;
}

int file_logger_open(const char *path, int mode, struct file_logger_file *file)
{
	if (!sd_mounted || !is_initialized) {
		LOG_ERR("File logger not initialized");
		return -EINVAL;
	}

	if (path == NULL || file == NULL) {
		return -EINVAL;
	}

	fs_file_t_init(&file->fobj);

	int ret = fs_open(&file->fobj, path, mode);
	if (ret < 0) {
		LOG_ERR("Failed to open file %s: %d", path, ret);
		return ret;
	}

	strncpy(file->path, path, sizeof(file->path) - 1);
	file->path[sizeof(file->path) - 1] = '\0';
	file->open = true;

	LOG_DBG("Opened file: %s", path);
	return 0;
}

int file_logger_close(struct file_logger_file *file)
{
	if (file == NULL || !file->open) {
		return -EINVAL;
	}

	int ret = fs_close(&file->fobj);
	file->open = false;

	if (ret < 0) {
		LOG_ERR("Failed to close file: %d", ret);
	}

	return ret;
}

int file_logger_write(struct file_logger_file *file, const uint8_t *data, size_t len)
{
	if (file == NULL || !file->open || data == NULL || len == 0) {
		return -EINVAL;
	}

	int written = fs_write(&file->fobj, data, len);
	if (written < 0) {
		LOG_ERR("Failed to write: %d", written);
		return written;
	}

	return written;
}

int file_logger_write_str(struct file_logger_file *file, const char *str)
{
	if (file == NULL || str == NULL) {
		return -EINVAL;
	}

	return file_logger_write(file, (const uint8_t *)str, strlen(str));
}

int file_logger_read(struct file_logger_file *file, uint8_t *buf, size_t len)
{
	if (file == NULL || !file->open || buf == NULL || len == 0) {
		return -EINVAL;
	}

	int bytes_read = fs_read(&file->fobj, buf, len);
	if (bytes_read < 0) {
		LOG_ERR("Failed to read: %d", bytes_read);
		return bytes_read;
	}

	return bytes_read;
}

int file_logger_read_str(struct file_logger_file *file, char *buf, size_t max_len)
{
	if (file == NULL || !file->open || buf == NULL || max_len == 0) {
		return -EINVAL;
	}

	/* Save current position */
	off_t orig_pos = fs_tell(&file->fobj);
	if (orig_pos < 0) {
		return orig_pos;
	}

	/* Read up to max_len - 1 bytes */
	int bytes_read = fs_read(&file->fobj, buf, max_len - 1);
	if (bytes_read < 0) {
		/* Restore position on error */
		fs_seek(&file->fobj, orig_pos, FS_SEEK_SET);
		return bytes_read;
	}

	/* Null-terminate */
	buf[bytes_read] = '\0';

	/* Restore position */
	fs_seek(&file->fobj, orig_pos, FS_SEEK_SET);

	return bytes_read;
}

int file_logger_flush(struct file_logger_file *file)
{
	if (file == NULL || !file->open) {
		return -EINVAL;
	}

	return fs_sync(&file->fobj);
}


/*
	Moves the file pointer to a new location based on offset and whence.
	Whence can be:
	- SEEK_SET: offset is set to the specified number of bytes from the beginning of the file.
	- SEEK_CUR: offset is set to its current location plus the specified number of bytes.
	- SEEK_END: offset is set to the size of the file plus the specified number of bytes.
*/
off_t file_logger_seek(struct file_logger_file *file, off_t offset, int whence)
{
	if (file == NULL || !file->open) {
		return -EINVAL;
	}

	return fs_seek(&file->fobj, offset, whence);
}

off_t file_logger_tell(struct file_logger_file *file)
{
	if (file == NULL || !file->open) {
		return -EINVAL;
	}

	return fs_tell(&file->fobj);
}

bool file_logger_exists(const char *path)
{
	struct fs_dirent dirent;
	int ret = fs_stat(path, &dirent);

	return (ret == 0);
}

int file_logger_remove(const char *path)
{
	if (path == NULL) {
		return -EINVAL;
	}

	return fs_unlink(path);
}

int file_logger_list_dir(const char *path, struct fs_dirent *entries, size_t max_entries)
{
	if (!sd_mounted || !is_initialized) {
		LOG_ERR("File logger not initialized");
		return -EINVAL;
	}

	if (path == NULL || entries == NULL || max_entries == 0) {
		return -EINVAL;
	}

	struct fs_dir_t dir;
	int ret = fs_opendir(&dir, path);
	if (ret < 0) {
		LOG_ERR("Failed to open dir %s: %d", path, ret);
		return ret;
	}

	size_t count = 0;
	struct fs_dirent entry;

	while (count < max_entries) {
		ret = fs_readdir(&dir, &entry);
		if (ret < 0) {
			LOG_ERR("Failed to read dir: %d", ret);
			fs_closedir(&dir);
			return ret;
		}

		if (ret == 0) {
			break;
		}

		entries[count++] = entry;
	}

	fs_closedir(&dir);
	return (int)count;
}