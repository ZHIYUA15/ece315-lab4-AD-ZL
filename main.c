#include "xparameters.h"
#include "xil_printf.h"
#include "network.h"
#include "shared_resources.h"
#include "gpio.h"

// Queue handles
QueueHandle_t buttonStateQueue = NULL;
QueueHandle_t xQueue_FIFO1 = NULL;

// Function prototypes
int Initialize_UART();

// Task handles
TaskHandle_t motorTaskHandle = NULL;


int main()
{
    int status;

    buttonStateQueue = xQueueCreate(1, sizeof(u32));
    xQueue_FIFO1 = xQueueCreate( 25, sizeof(MotorParameters) );

    configASSERT(buttonStateQueue);
	configASSERT(xQueue_FIFO1);

	// Initialize the PMOD for motor signals (JC PMOD is being used)
	status = XGpio_Initialize(&PModMotorInst, MOTOR_DEVICE_ID);

	if(status != XST_SUCCESS){
		xil_printf("GPIO Initialization for PMOD unsuccessful.\r\n");
		return XST_FAILURE;
	}

	//Initialize the UART
	status = Initialize_UART();

	if (status != XST_SUCCESS){
		xil_printf("UART Initialization failed\n");
	}

	// Initialize GPIO buttons
    status = XGpio_Initialize(&BtnsInst, BUTTONS_DEVICE_ID);

    if (status != XST_SUCCESS) {
        xil_printf("GPIO Initialization Failed\r\n");
        return XST_FAILURE;
    }

    XGpio_SetDataDirection(&BtnsInst, BUTTONS_CHANNEL, 0xFF);

	xTaskCreate( _Task_Motor
			   , "Motor Task"
			   , configMINIMAL_STACK_SIZE
			   , NULL
			   , DEFAULT_THREAD_PRIO + 1
			   , &motorTaskHandle
			   );

    xTaskCreate( pushbutton_task
    		   , "PushButtonTask"
			   , THREAD_STACKSIZE
			   , NULL
			   , DEFAULT_THREAD_PRIO
			   , NULL
			   );

    sys_thread_new( "main_thrd"
    			  , (void(*)(void*))main_thread
				  , 0
				  , THREAD_STACKSIZE
				  , DEFAULT_THREAD_PRIO + 1
				  );

    vTaskStartScheduler();
    while(1);
    return 0;
}
