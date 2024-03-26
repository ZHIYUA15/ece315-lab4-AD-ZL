#include "gpio.h"


void pushbutton_task(void *pvParameters)
{
	u8 button_val, last_button_val= 0;

    while(1){
        u8 pushButtonState = XGpio_DiscreteRead(&BtnsInst, BUTTONS_CHANNEL);
        button_val = XGpio_DiscreteRead(&BtnsInst, BUTTONS_CHANNEL);
        if (button_val != last_button_val){
        	while (xQueueSend(buttonStateQueue, &pushButtonState, 0) != pdPASS){
        		vTaskDelay(DELAY_50_MS);
			}
        	xil_printf("Pushbutton State sent to queue: %d\n\r", pushButtonState);
        	last_button_val = button_val;
        }
        vTaskDelay(DELAY_50_MS);
    }
}


void _Task_Motor( void *pvParameters )
{
	const TickType_t poll_period = 100;
	const TickType_t dwell_time = 3000;
	u32 loop=0;
	long motor_position = 0;
	// Initialization of motor parameter values
	MotorParameters motor_parameters = {0};

	Stepper_PMOD_pins_to_output();
	Stepper_Initialize();

	while(1){
		if(xQueueReceive(xQueue_FIFO1, &motor_parameters, 0) == pdTRUE){
			Stepper_setSpeedInStepsPerSecond(motor_parameters.rotational_speed);
			Stepper_setAccelerationInStepsPerSecondPerSecond(motor_parameters.rotational_acceleration);
			Stepper_setDecelerationInStepsPerSecondPerSecond(motor_parameters.rotational_deceleration);
			Stepper_setCurrentPositionInSteps(motor_parameters.currentposition_in_steps);
			xil_printf("finished on position: %d\n\n", motor_parameters.final_position);
			Stepper_moveToPositionInSteps(motor_parameters.final_position);
			motor_position = Stepper_getCurrentPositionInSteps();
			xil_printf("finished on position: %d\n\n", motor_position);
			vTaskDelay(poll_period); // dwell time
			xil_printf("after dwell\n\n", motor_position);
			loop++;
			xil_printf("\n\nloop: %d\n", loop);
		}
		vTaskDelay(poll_period); // polling period
	}
}
