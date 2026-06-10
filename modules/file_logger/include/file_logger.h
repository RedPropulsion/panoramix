/*
 * Copyright (c) 2024 Panoramix
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FILE_LOGGER_H
#define FILE_LOGGER_H

#include <stddef.h>
#include <stdbool.h>
#include <zephyr/fs/fs.h>

struct file_logger_file {
	struct fs_file_t fobj;
	char path[128];
	bool open;
};

int file_logger_init(void);

bool file_logger_is_mounted(void);

int file_logger_open(const char *path, int mode, struct file_logger_file *file);

int file_logger_close(struct file_logger_file *file);

int file_logger_write(struct file_logger_file *file, const uint8_t *data, size_t len);

int file_logger_write_str(struct file_logger_file *file, const char *str);

int file_logger_read(struct file_logger_file *file, uint8_t *buf, size_t len);

/**
 * @brief Read string from file (auto null-terminates, keeps position)
 *
 * @param file File handle
 * @param buf Buffer for string
 * @param max_len Maximum bytes to read (buffer size)
 *
 * @return Number of bytes read (excluding null terminator)
 */
int file_logger_read_str(struct file_logger_file *file, char *buf, size_t max_len);

int file_logger_flush(struct file_logger_file *file);

off_t file_logger_seek(struct file_logger_file *file, off_t offset, int whence);

off_t file_logger_tell(struct file_logger_file *file);

bool file_logger_exists(const char *path);

int file_logger_remove(const char *path);

/**
 * @brief List files in a directory
 *
 * @param path Directory path (e.g., "/SD:/")
 * @param entries Array of dirent structures
 * @param max_entries Maximum entries to return
 *
 * @return Number of entries written, negative error code on failure
 */
int file_logger_list_dir(const char *path, struct fs_dirent *entries, size_t max_entries);

#endif /* FILE_LOGGER_H */