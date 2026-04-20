# FatFs - File System for STM32

## What is FatFs?

FatFs is a generic FAT (File Allocation Table) file system module for embedded systems. It's an open-source librarywritten by ChaN (ChaNung Park) that provides a standard API for reading and writing files on SD cards, USB drives, and other storage media formatted with FAT12, FAT16, or FAT32 filesystems.

### Key Features

- **FAT12/FAT16/FAT32** support
- **Long File Name (LFN)** support (optional)
- **Multiple volumes** support
- **Read-only and read-write** modes
- **Small footprint** - suitable for microcontrollers
- **Platform independent** - works with any storage device

## How It Works in Zephyr

### Architecture Overview

```
+------------------+
|   Application    |
+------------------+
        |
        v
+------------------+     +------------------+
|   FS Interface   |<--->|  FatFs Module    |
|   (VFS API)      |     |   (ff.c)         |
+------------------+     +------------------+
        |                        |
        v                        v
+------------------+      +------------------+
|  Disk Access     |<---->|   Disk I/O       |
|   Driver         |      |  (diskio.c)      |
+------------------+      +------------------+
        |
        v
+------------------+
|   SDMMC/SDIO     |
|   Driver         |
+------------------+
```

### Layers

1. **Application** - Your code using standard file operations
2. **FS Interface** - Zephyr's VFS (Virtual File System) API
3. **FatFs Module** - The actual FAT implementation
4. **Disk I/O** - Glue between FatFs and storage driver
5. **Storage Driver** - SDMMC, SPI flash, etc.

## Required Configuration

### Kconfig Options

Add to `prj.conf`:

```kconfig
# Enable filesystem subsystem
CONFIG_FILE_SYSTEM=y

# Enable FAT file system
CONFIG_FAT_FILESYSTEM_ELM=y

# Optional settings
CONFIG_FS_FATFS_READ_ONLY=n       # Enable write support (default)
CONFIG_FS_FATFS_MKFS=y            # Enable formatting
CONFIG_FS_FATFS_LFN=y             # Enable long filenames
CONFIG_FS_FATFS_MAX_LFN=255       # Max filename length
```

### Dependencies

FatFs requires these configurations:

```kconfig
CONFIG_FILE_SYSTEM=y
CONFIG_FILE_SYSTEM_LIB_LINK=y
CONFIG_DISK_ACCESS=y           # Must match your storage (SDMMC, SPI, etc.)
CONFIG_DISK_DRIVER_SDMMC=y     # For SD card support
```

## Using FatFs in Your Application

### Mount a Volume

```c
#include <zephyr/fs/fs.h>

static struct fs_mount_t fat_fs_mnt = {
    .type = FS_FATFS,
    .fs_data = &fatfs_fs,       /* FatFs work area */
    .mnt_point = "/SD:",       /* Mount point */
};

/* Mount the SD card */
int ret = fs_mount(&fat_fs_mnt);
if (ret < 0) {
    LOG_ERR("Mount failed: %d", ret);
}
```

### Open a File

```c
struct fs_file_t filep;
fs_file_t_init(&filep);

ret = fs_open(&filep, "/SD:/test.txt", FS_O_CREATE | FS_O_WRITE);
if (ret < 0) {
    LOG_ERR("Open failed: %d", ret);
    return;
}

/* Write data */
const char *data = "Hello FatFs!";
ret = fs_write(&filep, data, strlen(data));
if (ret < 0) {
    LOG_ERR("Write failed: %d", ret);
}

fs_close(&filep);
```

### Read a File

```c
struct fs_file_t filep;
uint8_t buf[128];

fs_file_t_init(&filep);

ret = fs_open(&filep, "/SD:/test.txt", FS_O_READ);
if (ret < 0) {
    LOG_ERR("Open failed: %d", ret);
    return;
}

ret = fs_read(&filep, buf, sizeof(buf) - 1);
if (ret < 0) {
    LOG_ERR("Read failed: %d", ret);
} else {
    buf[ret] = '\0';
    LOG_INF("Read: %s", buf);
}

fs_close(&filep);
```

### List Directory

```c
struct fs_dir_t dirp;
struct fs_dirent entry;

fs_dir_t_init(&dirp);

ret = fs_opendir(&dirp, "/SD:/");
if (ret < 0) {
    LOG_ERR("Open dir failed: %d", ret);
    return;
}

while (1) {
    ret = fs_readdir(&dirp, &entry);
    if (ret < 0) {
        LOG_ERR("Read dir failed: %d", ret);
        break;
    }
    if (entry.name[0] == '\0') {
        break;  /* End of directory */
    }
    LOG_INF(" %c %d %s",
           (entry.type == FS_DIR_ENTRY_FILE) ? 'f' : 'd',
           entry.size,
           entry.name);
}

fs_closedir(&dirp);
```

### Format a Volume

```c
#include <zephyr/storage/flash_map.h>

/* Get the flash device */
const struct device *flash_dev = DEVICE_DT_GET(EXECUTABLEABLE);

ret = fs_format(flash_dev, FS_FATFS);
if (ret < 0) {
    LOG_ERR("Format failed: %d", ret);
}
```

### Unmount

```c
ret = fs_unmount(&fat_fs_mnt);
if (ret < 0) {
    LOG_ERR("Unmount failed: %d", ret);
}
```

## FatFs Work Area

The FatFs library requires a work area (FATFS structure) to store filesystem information:

```c
/* In your source file, outside of any function */
FATFS fatfs_fs;

/* Or as static allocation */
static FATFS fatfs_fs;
```

Note: The size depends on the number of open files and directories configured in Kconfig:
- `CONFIG_FS_FATFS_NUM_FILES` - Maximum open files
- `CONFIG_FS_FATFS_NUM_DIRS` - Maximum open directories

## Configuration Options Explained

| Option | Description | Default |
|--------|-------------|---------|
| `CONFIG_FAT_FILESYSTEM_ELM` | Enable ELM FAT implementation | n |
| `CONFIG_FS_FATFS_READ_ONLY` | Disable write support | n |
| `CONFIG_FS_FATFS_MKFS` | Enable mkfs formatting | y |
| `CONFIG_FS_FATFS_LFN` | Long filename support | n |
| `CONFIG_FS_FATFS_EXFAT` | exFAT support | n |
| `CONFIG_FS_FATFS_NUM_FILES` | Max open files | 4 |
| `CONFIG_FS_FATFS_NUM_DIRS` | Max open dirs | 4 |

## Common Issues

### 1. Mount Fails

**Error:** `fs_mount error (-5)` (-EIO)

**Causes:**
- Storage not initialized
- No card inserted
- Storage driver error

**Solutions:**
- Check if SDMMC is initialized first
- Verify card is inserted
- Check storage driver logs

### 2. File Not Found

**Error:** `No such file or directory`

**Causes:**
- Wrong path
- File doesn't exist

**Solutions:**
- Use correct mount point (`/SD:/` not `/SD`)
- Check file exists with `fs_readdir()`

### 3. Write Fails

**Error:** `Read-only file system`

**Solutions:**
- Check `CONFIG_FS_FATFS_READ_ONLY=n`
- Card may be write-protected physically

### 4. Formatting Fails

**Error:** `fs_format error`

**Solutions:**
- Verify `CONFIG_FS_FATFS_MKFS=y`
- Card may be write-protected

## File Path Format

Paths in Zephyr FatFs follow this pattern:

```
<mount_point>/<path>/

Examples:
/SD:/             - Root of SD card
/SD:/data.txt     - File in root
/SD:/dir/file.txt - File in subdirectory
```

Note: The mount point must end with `:` for FAT volumes.

## References

- FatFs Official Website: http://elm-chan.org/fsw/ff/00index_e.html
- Zephyr FS API: `zephyr/include/zephyr/fs/fs.h`
- Zephyr Storage API: `zephyr/include/zephyr/storage/disk_access.h`
- Original FatFs source: `modules/fs/fatfs/`