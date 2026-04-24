/**
 * @file sd_card.h
 * @brief SD card logging module for Obelics
 *
 * Provides non-blocking SD card writes with background sync thread.
 * Designed for logging MavLink packets during rocket flight sessions.
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>

/* Session header structure */
struct session_header {
    uint32_t magic;
    uint32_t start_time;
    int32_t  rtt_offset;
    uint8_t  reserved[8];
};

#define SESSION_MAGIC 0x524C4731  /* "RLG1" - Rocket Log */

/**
 * @brief Initialize SD card and mount filesystem
 *
 * @return 0 on success, negative errno on error
 */
int sd_card_init(void);

/**
 * @brief Create a new session file
 *
 * @param filename Buffer to store filename (output)
 * @param max_len Maximum filename buffer size
 * @return 0 on success, negative errno on error
 */
int sd_card_create_session(char *filename, size_t max_len);

/**
 * @brief Write data to session file (non-blocking)
 *
 * Data is buffered in RAM. Actual write to SD happens:
 * - When buffer fills
 * - When sd_card_sync() is called
 * - When session ends
 *
 * @param data Data to write
 * @param len Number of bytes to write
 * @return 0 on success, negative errno on error
 */
int sd_card_write(const uint8_t *data, size_t len);

/**
 * @brief Sync buffered data to SD card
 *
 * Call this periodically to ensure data is written.
 * Also called automatically at session end.
 *
 * @return 0 on success, negative errno on error
 */
int sd_card_sync(void);

/**
 * @brief Flush all pending writes and close session
 *
 * @return 0 on success, negative errno on error
 */
int sd_card_close_session(void);

/**
 * @brief List files on SD card
 *
 * @return 0 on success, negative errno on error
 */
int sd_card_list_files(void);

/**
 * @brief Read session header from file
 *
 * @param filename Filename to read
 * @param header Pointer to header structure (output)
 * @return 0 on success, negative errno on error
 */
int sd_card_read_header(const char *filename, struct session_header *header);

/**
 * @brief Check if SD card is mounted
 *
 * @return true if mounted, false otherwise
 */
bool sd_card_is_mounted(void);

#endif /* SD_CARD_H */