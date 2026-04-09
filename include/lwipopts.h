/*
 * lwIP options for FreeRTOS + tcpip_thread operation on this project.
 */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                      0
#define LWIP_TIMERS                 1
#define LWIP_DHCP                   1
#define LWIP_ICMP                   1
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_NETIF_API              1

#define LWIP_RAW                    1
#define LWIP_TCP                    0
#define LWIP_UDP                    1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

#define SYS_LIGHTWEIGHT_PROT        1
#define MEM_ALIGNMENT               32
#define MEM_SIZE                    (32 * 1024)
#define PBUF_POOL_SIZE              12
#define MEMP_NUM_UDP_PCB            4
#define MEMP_NUM_RAW_PCB            8
#define MEMP_NUM_TCPIP_MSG_API      16
#define MEMP_NUM_NETIFAPI_MSG       16
#define MEMP_NUM_TCPIP_MSG_INPKT    16
#define MEMP_NUM_SYS_TIMEOUT        16
#define TCPIP_THREAD_STACKSIZE      2048
#define TCPIP_THREAD_PRIO           osPriorityAboveNormal
#define TCPIP_MBOX_SIZE             16
#define DEFAULT_THREAD_STACKSIZE    1024
#define DEFAULT_RAW_RECVMBOX_SIZE   8
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_LINK_CALLBACK    0
#define LWIP_NETIF_STATUS_CALLBACK  0

#endif /* LWIPOPTS_H */
