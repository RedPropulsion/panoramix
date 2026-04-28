#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

int udp_client_init(void);
int udp_client_send(const void *data, size_t len);
int udp_client_recv(void *buf, size_t max_len);

#endif /* UDP_CLIENT_H */