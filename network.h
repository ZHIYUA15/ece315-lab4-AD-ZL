#ifndef NETWORK_H
#define NETWORK_H

#include "xparameters.h"
#include "netif/xadapter.h"
#include "lwip/init.h"
#include "platform_config.h"
#include "xil_printf.h"

// IP address settings---------------------------------------------------------
#define ADDR1
#define ADDR2
#define ADDR3
#define ADDR4

// Gateway address settings----------------------------------------------------
#define GW1
#define GW2
#define GW3
#define GW4

// Netmask settings------------------------------------------------------------
#define NETMASK1
#define NETMASK2
#define NETMASK3
#define NETMASK4

#define THREAD_STACKSIZE 	1024
#define START_DELAY 		1000

// useful for printing the defined values--------------------------------------
#define PRINT_IP(msg, ip) \
	xil_printf(msg); \
	xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),ip4_addr3(ip), ip4_addr4(ip));

// Function prototypes
int main_thread();
void network_thread(void *p);
void print_ip_setup(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw);

#endif
