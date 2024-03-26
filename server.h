#ifndef SERVER_H
#define SERVER_H

#include <stdio.h>
#include <string.h>

#include "lwip/sockets.h"
#include "netif/xadapter.h"
#include "lwipopts.h"
#include "xil_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stepper.h"

#define THREAD_STACKSIZE 	1024
#define RECV_BUF_SIZE 		2048
#define RESPONSE_SIZE 		600
#define SERVER_PORT 		80


// Variables
long motor_position;
extern QueueHandle_t buttonStateQueue;
extern QueueHandle_t xQueue_FIFO1;

// Function prototypes
void server_application_thread(void);
int write_to_socket(int sd, const char* send_buf);
int parse_query_parameter(const char* name, const char* value, MotorParameters* motor_parameters);
void process_query_string(const char* query, MotorParameters* params);
void print_server_app_header(void);

#endif
