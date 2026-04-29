#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udp_client, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/posix/poll.h>

#include <string.h>

#ifdef CONFIG_UDP_CLIENT_SERVER_IP
#define UDP_SERVER_IP CONFIG_UDP_CLIENT_SERVER_IP
#else
#define UDP_SERVER_IP "192.168.1.1"
#endif

#ifdef CONFIG_UDP_CLIENT_SERVER_PORT
#define UDP_SERVER_PORT CONFIG_UDP_CLIENT_SERVER_PORT
#else
#define UDP_SERVER_PORT 14550
#endif

#ifdef CONFIG_UDP_CLIENT_LOCAL_PORT
#define UDP_LOCAL_PORT CONFIG_UDP_CLIENT_LOCAL_PORT
#else
#define UDP_LOCAL_PORT 14551
#endif

#ifdef CONFIG_UDP_CLIENT_STACK_SIZE
#define ETH_STACK_SIZE CONFIG_UDP_CLIENT_STACK_SIZE
#else
#define ETH_STACK_SIZE 2048
#endif

#ifdef CONFIG_UDP_CLIENT_PRIORITY
#define ETH_PRIORITY CONFIG_UDP_CLIENT_PRIORITY
#else
#define ETH_PRIORITY 5
#endif

static K_SEM_DEFINE(net_ready, 0, 1);
static struct k_thread eth_thread_data;
static K_KERNEL_STACK_DEFINE(eth_stack, ETH_STACK_SIZE);

static int sock = -1;

static void net_mgmt_cb(struct net_mgmt_event_callback *cb,
			 uint64_t mgmt_event, struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);
	if (mgmt_event == NET_EVENT_IF_UP) {
		k_sem_give(&net_ready);
	}
}

static int wait_for_net(void)
{
	struct net_mgmt_event_callback mgmt_cb;

	net_mgmt_init_event_callback(&mgmt_cb, net_mgmt_cb, NET_EVENT_IF_UP);
	net_mgmt_add_event_callback(&mgmt_cb);

	if (net_if_is_up(net_if_get_default())) {
		k_sem_give(&net_ready);
	}

	k_sem_take(&net_ready, K_FOREVER);
	net_mgmt_del_event_callback(&mgmt_cb);

	return 0;
}

int udp_client_send(const void *data, size_t len)
{
	struct net_sockaddr_in server_addr;
	int ret;

	if (sock < 0) {
		return -ENOTCONN;
	}

	server_addr.sin_family = NET_AF_INET;
	server_addr.sin_port = net_htons(UDP_SERVER_PORT);
	ret = net_addr_pton(NET_AF_INET, UDP_SERVER_IP, &server_addr.sin_addr);
	if (ret < 0) {
		return ret;
	}

	return zsock_sendto(sock, data, len, 0,
			  (struct net_sockaddr *)&server_addr, sizeof(server_addr));
}

int udp_client_recv(void *buf, size_t max_len, int timeout_ms)
{
	struct net_sockaddr_in from;
	net_socklen_t from_len = sizeof(from);
	struct pollfd pfd = {
		.fd = sock,
		.events = POLLIN,
	};

	if (sock < 0) {
		return -ENOTCONN;
	}

	if (poll(&pfd, 1, timeout_ms) <= 0) {
		return -EAGAIN;
	}

	return zsock_recvfrom(sock, buf, max_len, 0,
			   (struct net_sockaddr *)&from, &from_len);
}

static void eth_thread(void *p1, void *p2, void *p3)
{
	int ret;
	struct net_sockaddr_in local_addr;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("UDP client thread started");

	wait_for_net();

	sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create socket: %d", -errno);
		return;
	}

	memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sin_family = NET_AF_INET;
	local_addr.sin_port = net_htons(UDP_LOCAL_PORT);
	local_addr.sin_addr.s_addr = net_htonl(NET_ADDR_ANY);

	ret = zsock_bind(sock, (struct net_sockaddr *)&local_addr, sizeof(local_addr));
	if (ret < 0) {
		LOG_ERR("Failed to bind socket: %d", -errno);
		zsock_close(sock);
		sock = -1;
		return;
	}

	LOG_INF("UDP socket bound to port %d", UDP_LOCAL_PORT);
	LOG_INF("Target: %s:%d", UDP_SERVER_IP, UDP_SERVER_PORT);

	char test_msg[] = "Status: OK";
	ret = udp_client_send(test_msg, sizeof(test_msg) - 1);

	if (ret > 0) {
		LOG_DBG("UDP send success: %d bytes", ret);
	} else {
		LOG_ERR("UDP send failed: %d", ret);
	}

	char rx_buf[512];
	while (1) {
		ret = udp_client_recv(rx_buf, sizeof(rx_buf) - 1, 100);
		if (ret > 0) {
			rx_buf[ret] = '\0';
			LOG_DBG("UDP recv: %s", rx_buf);
		}
		k_sleep(K_MSEC(100));
	}
}

int udp_client_init(void)
{
	LOG_INF("Initializing UDP client --- Make sure Ethernet cable is connected!");
	k_thread_create(&eth_thread_data, eth_stack,
			K_KERNEL_STACK_SIZEOF(eth_stack),
			eth_thread, NULL, NULL, NULL,
			ETH_PRIORITY, 0, K_NO_WAIT);
	return 0;
}

void udp_client_deinit(void)
{
	if (sock >= 0) {
		zsock_close(sock);
		sock = -1;
	}
}