/*
 * lwIP options for bare-metal, polling-only use on this project.
 */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                      1
#define LWIP_TIMERS                 1
#define LWIP_DHCP                   1
#define LWIP_ICMP                   1
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0

#define LWIP_RAW                    1
#define LWIP_TCP                    0
#define LWIP_UDP                    1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

#define SYS_LIGHTWEIGHT_PROT        0
#define MEM_ALIGNMENT               32
#define MEM_SIZE                    (16 * 1024)
#define PBUF_POOL_SIZE              8
#define MEMP_NUM_UDP_PCB            4
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_LINK_CALLBACK    0
#define LWIP_NETIF_STATUS_CALLBACK  0

#endif /* LWIPOPTS_H */
