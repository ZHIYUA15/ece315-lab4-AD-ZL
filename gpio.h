#ifndef GPIO_H
#define GPIO_H

#include "xgpio.h"
#include "stepper.h"
#include "shared_resources.h"

#define BUTTONS_DEVICE_ID 	XPAR_AXI_GPIO_INPUTS_DEVICE_ID
#define MOTOR_DEVICE_ID   	XPAR_GPIO_2_DEVICE_ID
#define BUTTONS_CHANNEL 1
#define DELAY_50_MS pdMS_TO_TICKS(50)

XGpio BtnsInst;

extern QueueHandle_t buttonStateQueue;
extern QueueHandle_t xQueue_FIFO1;

void _Task_Motor( void *pvParameters );
void pushbutton_task(void *pvParameters);

#endif
