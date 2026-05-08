#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize UDP client
 *
 * Starts the network thread and prepares for communication.
 * Network configuration is read from Kconfig options.
 *
 * @return 0 on success, negative error code on failure
 */
int udp_client_init(void);

/**
 * @brief Send data via UDP
 *
 * @param data Pointer to data to send
 * @param len Length of data in bytes
 * @return Number of bytes sent on success, negative error code on failure
 */
int udp_client_send(const void *data, size_t len);

/**
 * @brief Receive data via UDP (blocking with timeout)
 *
 * @param buf Buffer to store received data
 * @param max_len Maximum bytes to receive
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes received on success, -EAGAIN on timeout, negative on error
 */
int udp_client_recv(void *buf, size_t max_len, int timeout_ms);

/**
 * @brief Deinitialize UDP client
 *
 * Closes sockets and cleans up resources.
 */
void udp_client_deinit(void);

#endif /* UDP_CLIENT_H */