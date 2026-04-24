/**
 * @file sd_card.c
 * @brief SD card logging module implementation
 *
 * Non-blocking writes with background sync thread.
 * Uses dual buffer strategy: write buffer and sync buffer.
 */

#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sd_card);

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "sd_card.h"

/* Buffer configuration */
#define SD_WRITE_BUFFER_SIZE 4096
#define SD_SYNC_STACK_SIZE 512
#define SD_SYNC_PRIORITY 8

/* FatFs work area and mount point */
static FATFS fatfs_fs;
static struct fs_mount_t fat_fs_mnt = {
    .type = FS_FATFS,
    .fs_data = &fatfs_fs,
    .mnt_point = "/SD:",
};

/* Write buffer (filled by sd_card_write) */
static uint8_t write_buffer[SD_WRITE_BUFFER_SIZE];
static size_t write_buffer_pos;

/* File handle for session */
static FIL session_file;
static bool session_open;

/* Mount state */
static bool sd_mounted;

/* Sync thread control */
static struct k_sem sync_sem;
static bool sync_thread_running;
static struct k_thread sync_thread;
static K_THREAD_STACK_DEFINE(sync_thread_stack, SD_SYNC_STACK_SIZE);

/**
 * @brief Internal: List directory contents
 */
static int lsdir(const char *path)
{
    int res;
    struct fs_dir_t dirp;
    static struct fs_dirent entry;

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, path);
    if (res) {
        LOG_WRN("Error opening dir %s [%d]", path, res);
        return res;
    }

    LOG_INF("Listing dir %s ...", path);
    for (;;) {
        res = fs_readdir(&dirp, &entry);

        if (res || entry.name[0] == 0) {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR) {
            LOG_DBG("[DIR ] %s", entry.name);
        } else {
            LOG_INF("[FILE] %s (size = %zu)", entry.name, entry.size);
        }
    }

    fs_closedir(&dirp);
    return res;
}

/**
 * @brief Internal: Flush write buffer to disk
 */
static int flush_buffer(void)
{
    UINT bw;
    int res;

    if (write_buffer_pos == 0) {
        return 0;
    }

    if (!session_open) {
        LOG_WRN("No session file open, discarding %zu bytes", write_buffer_pos);
        write_buffer_pos = 0;
        return 0;
    }

    res = f_write(&session_file, write_buffer, write_buffer_pos, &bw);
    if (res != FR_OK) {
        LOG_ERR("f_write failed: %d", res);
        return -EIO;
    }

    if (bw != write_buffer_pos) {
        LOG_WRN("Partial write: %u/%zu", bw, write_buffer_pos);
    }

    write_buffer_pos = 0;
    return 0;
}

/**
 * @brief Sync thread: waits for sync requests and writes to SD
 */
static void sync_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    LOG_INF("Sync thread started");

    while (sync_thread_running) {
        k_sem_take(&sync_sem, K_FOREVER);

        if (!sync_thread_running) {
            break;
        }

        flush_buffer();

        if (session_open) {
            f_sync(&session_file);
        }
    }

    LOG_INF("Sync thread stopped");
}

int sd_card_init(void)
{
    int res;
    uint32_t block_count;
    uint32_t block_size;
    uint64_t memory_size_mb;

    LOG_INF("Initializing SD card");

    /* Initialize disk */
    static const char *disk_pdrv = "SD";

    if (disk_access_init(disk_pdrv) != 0) {
        LOG_ERR("Storage init failed");
        return -EIO;
    }

    if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
        LOG_ERR("Unable to get sector count");
        return -EIO;
    }

    if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
        LOG_ERR("Unable to get sector size");
        return -EIO;
    }

    memory_size_mb = (uint64_t)block_count * block_size;
    LOG_INF("SD card: %u sectors, %u bytes/sector, %llu MB",
           block_count, block_size, memory_size_mb >> 20);

    /* Mount filesystem */
    res = fs_mount(&fat_fs_mnt);
    if (res != 0) {
        LOG_ERR("Mount failed: %d", res);
        return res;
    }

    sd_mounted = true;
    LOG_INF("SD Card mounted at %s", fat_fs_mnt.mnt_point);

    /* Initialize write buffer */
    write_buffer_pos = 0;

    /* Initialize sync semaphore */
    k_sem_init(&sync_sem, 0, 1);

    /* List files */
    lsdir(fat_fs_mnt.mnt_point);

    return 0;
}

int sd_card_create_session(char *filename, size_t max_len)
{
    time_t now;
    struct tm timeinfo;
    char filepath[64];
    FRESULT fres;
    UINT bw;
    struct session_header header;

    if (!sd_card_is_mounted()) {
        LOG_ERR("SD card not mounted");
        return -ENODEV;
    }

    /* Generate filename */
    (void)time(&now);
    (void)gmtime_r(&now, &timeinfo);

    snprintf(filepath, sizeof(filepath), "%s/FLIGHT_%04d%02d%02d_%02d%02d%02d.bin",
            fat_fs_mnt.mnt_point,
            timeinfo.tm_year + 1900,
            timeinfo.tm_mon + 1,
            timeinfo.tm_mday,
            timeinfo.tm_hour,
            timeinfo.tm_min,
            timeinfo.tm_sec);

    /* Create file */
    fres = f_open(&session_file, filepath, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) {
        LOG_ERR("f_open failed: %d", fres);
        return -EIO;
    }

    session_open = true;
    write_buffer_pos = 0;

    /* Write header */
    memset(&header, 0, sizeof(header));
    header.magic = SESSION_MAGIC;
    header.start_time = k_uptime_get_32();

    fres = f_write(&session_file, &header, sizeof(header), &bw);
    if (fres != FR_OK || bw != sizeof(header)) {
        LOG_ERR("Header write failed");
        f_close(&session_file);
        session_open = false;
        return -EIO;
    }

    /* Sync header immediately */
    f_sync(&session_file);

    /* Return filename */
    snprintf(filename, max_len, "%s", filepath);

    LOG_INF("Created session file: %s", filename);

    /* Start sync thread */
    sync_thread_running = true;
    k_thread_create(&sync_thread,
                    sync_thread_stack,
                    SD_SYNC_STACK_SIZE,
                    sync_thread_fn,
                    NULL, NULL, NULL,
                    SD_SYNC_PRIORITY,
                    0,
                    K_NO_WAIT);

    return 0;
}

int sd_card_write(const uint8_t *data, size_t len)
{
    size_t remaining = len;

    if (!sd_card_is_mounted()) {
        return -ENODEV;
    }

    if (!session_open) {
        return -ENODEV;
    }

    /* Handle data larger than buffer */
    while (remaining > 0) {
        size_t space = SD_WRITE_BUFFER_SIZE - write_buffer_pos;

        if (len > space) {
            memcpy(&write_buffer[write_buffer_pos], data, space);
            write_buffer_pos = SD_WRITE_BUFFER_SIZE;
            data += space;
            remaining -= space;

            k_sem_give(&sync_sem);
        } else {
            memcpy(&write_buffer[write_buffer_pos], data, remaining);
            write_buffer_pos += remaining;
            remaining = 0;
        }
    }

    return 0;
}

int sd_card_sync(void)
{
    if (!session_open) {
        return 0;
    }

    k_sem_give(&sync_sem);
    return 0;
}

int sd_card_close_session(void)
{
    FRESULT fres;

    if (!session_open) {
        return 0;
    }

    LOG_INF("Closing session");

    /* Signal sync thread to stop */
    sync_thread_running = false;
    k_sem_give(&sync_sem);

    /* Final flush */
    flush_buffer();

    fres = f_sync(&session_file);
    if (fres != FR_OK) {
        LOG_ERR("f_sync failed: %d", fres);
        return -EIO;
    }

    fres = f_close(&session_file);
    session_open = false;

    if (fres != FR_OK) {
        LOG_ERR("f_close failed: %d", fres);
        return -EIO;
    }

    write_buffer_pos = 0;

    LOG_INF("Session closed");
    return 0;
}

int sd_card_list_files(void)
{
    if (!sd_card_is_mounted()) {
        return -ENODEV;
    }

    return lsdir(fat_fs_mnt.mnt_point);
}

int sd_card_read_header(const char *filename, struct session_header *header)
{
    FIL file;
    FRESULT fres;
    UINT br;

    fres = f_open(&file, filename, FA_READ);
    if (fres != FR_OK) {
        LOG_ERR("f_open failed: %d", fres);
        return -EIO;
    }

    fres = f_read(&file, header, sizeof(*header), &br);
    f_close(&file);

    if (fres != FR_OK) {
        LOG_ERR("f_read failed: %d", fres);
        return -EIO;
    }

    if (br != sizeof(*header)) {
        LOG_ERR("Partial read: %u/%zu", br, sizeof(*header));
        return -EIO;
    }

    return 0;
}

bool sd_card_is_mounted(void)
{
    return sd_mounted;
}