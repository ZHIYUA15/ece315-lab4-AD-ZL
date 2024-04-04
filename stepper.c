/*
 * stepper_driver.c
 *
 *  Created on: Mar 24, 2021
 *  Modified by : Shyama Gandhi
 *  Modified by : Antonio Andara on March 19, 2024
 *  The driver has been modified according to the lab needs and the code is adapted from TinyStepper_28BYJ_48 arduino library
 */

#include "stepper.h"

// Define the step mode enum
int idx;

void Stepper_PMOD_pins_to_output()
{
	// set the GPIO pins to output direction
	// write the initial value of "0b0000" to the four bit motor signal
	XGpio_SetDataDirection(&PModMotorInst, 1, 0x00);
	XGpio_DiscreteWrite(&PModMotorInst, 1, WINDINGS_OFF);
}

void Stepper_Initialize()
{
	// pmod motor signals
	_signals_to_motor[0] = 0;
	_signals_to_motor[1] = 0;
	_signals_to_motor[2] = 0;
	_signals_to_motor[3] = 0;

	currentPosition_InSteps = 0;
	desiredSpeed_InStepsPerSecond = 2048.0 / 4.0;		  // initial speed
	acceleration_InStepsPerSecondPerSecond = 2048.0 / 10; // initial acceleration
	currentStepPeriod = 0.0;
	stepPhase = 0;
}

/*
 * enter the current position of motor in steps and this function will not cause any motor rotation
 * Call this function only when the motor is at rest
 */
void Stepper_setCurrentPositionInSteps(long currentPositionInSteps)
{
	currentPosition_InSteps = currentPositionInSteps;
}

/*
 * This function gets the current position of the motor in steps when the motor is in motion
 * RETURNS: signed steps returned
 */
long Stepper_getCurrentPositionInSteps()
{
	return (currentPosition_InSteps);
}

void Stepper_SetupStop()
{
	if (direction_Scaler > 0)
		targetPosition_InSteps = currentPosition_InSteps + decelerationDistance_InSteps;
	else
		targetPosition_InSteps = currentPosition_InSteps - decelerationDistance_InSteps;
}

/*
 * This function will set the maximum speed in steps/second that will be reached when in accelerating mode
 * ENTER: speedInStepsPerSecond = accelerate the motor to this speed speed and then the cruising/slewing mode is at this speed
 */
void Stepper_setSpeedInStepsPerSecond(float speedInStepsPerSecond)
{
	desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
}

/*
 * set the acceleration in steps/second/second
 * ENTER: accelerationInStepsPerSecondPerSecond = ACCELERATION RATE. (units: steps/second/second)
 */
void Stepper_setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond)
{
	acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
}

/*
 * set the deceleration in steps/second/second
 * ENTER: decelerationInStepsPerSecondPerSecond = DECELERATION RATE. (units: steps/second/second)
 */
void Stepper_setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond)
{
	deceleration_InStepsPerSecondPerSecond = decelerationInStepsPerSecondPerSecond;
}

/*
 * move relative to the current position in steps, this function does not return until the motor motion will be complete
 */
void Stepper_moveRelativeInSteps(long distanceToMoveInSteps)
{
	Stepper_SetupRelativeMoveInSteps(distanceToMoveInSteps);
	while (!Stepper_processMovement())
	{
	};
}

void Stepper_SetupRelativeMoveInSteps(long distanceToMoveInSteps)
{
	Stepper_SetupMoveInSteps(currentPosition_InSteps + distanceToMoveInSteps);
}

void Stepper_moveToPositionInSteps(long absolutePositionToMoveToInSteps)
{
	Stepper_SetupMoveInSteps(absolutePositionToMoveToInSteps);
	while (!Stepper_processMovement())
	{
		// Check for emergency stop button press
		if (emergency_stop_triggered)
		{
			Stepper_disableMotor();
			break;
		}
		vTaskDelay(1); // Delay for 1 tick to prevent blocking behavior
	}
	if (!emergency_stop_triggered)
	{
		Stepper_disableMotor();
	}
}

Based on the requirements and the provided code files, here are the modified code files for stepper.h, stepper.c, and main.c:

<document index="3"> <source>stepper.h</source> <document_content> // Add the following declarations in stepper.h
// Function prototype for updating the LED animation
void Stepper_updateLEDAnimation();

// External variable to track the emergency stop state
extern _Bool emergency_stop_triggered;
</ document_content>
	</ document>

	<document index = "4"><source> stepper.c</ source><document_content> // Add the following function in stepper.c
	void Stepper_updateLEDAnimation()
{
	if (Stepper_motionComplete())
	{
		// Turn off all LEDs when motor is not in motion
		XGpio_DiscreteWrite(&LEDInst, LED_CHANNEL, 0x00);
	}
	else
	{
		// Animate LEDs based on the current step mode
		switch (currentStepMode)
		{
		case WAVE_DRIVE:
			// Implement LED animation for wave drive mode
			break;
		case FULL_STEP:
			// Implement LED animation for full step mode
			break;
		case HALF_STEP:
			// Implement LED animation for half step mode
			break;
		}
	}
}

//----------------------------------------------Zhiyuan Li------------------------------------------------

// Update the Stepper_moveToPositionInSteps function to check for emergency stop

void Stepper_moveToPositionInSteps(long absolutePositionToMoveToInSteps)
{
	Stepper_SetupMoveInSteps(absolutePositionToMoveToInSteps);
	while (!Stepper_processMovement())
	{
		// Check for emergency stop button press
		if (emergency_stop_triggered)
		{
			Stepper_disableMotor();
			break;
		}
		vTaskDelay(1); // Delay for 1 tick to prevent blocking behavior
	}
	if (!emergency_stop_triggered)
	{
		Stepper_disableMotor();
	}
}
</ document_content>
	</ document>

	<document index = "5"><source> main.c</ source><document_content> // Add the following tasks in main.c
	void emergency_stop_task(void *pvParameters)
{
	while (1)
	{
		u32 buttonState;
		XGpio_DiscreteRead(&BtnsInst, BUTTONS_CHANNEL, &buttonState);
		if (buttonState & EMERGENCY_STOP_BUTTON_MASK)
		{
			emergency_stop_triggered = TRUE;
		}

		vTaskDelay(10); // Delay for 10 ticks (100 ms)
	}
}

void led_animation_task(void *pvParameters)
{
	while (1)
	{
		Stepper_updateLEDAnimation();
		vTaskDelay(50); // Delay for 50 ticks (500 ms) to slow down the animation
	}
}

//----------------------------------------------Zhiyuan Li------------------------------------------------

// External variable to track the emergency stop state
extern _Bool emergency_stop_triggered;
</ document_content>
	</ document>

	<document index = "4"><source> stepper.c</ source><document_content> // Add the following function in stepper.c

	// Add a new function to update the LED animation based on the current step mode
	void Stepper_updateLEDAnimation()
{
	if (Stepper_motionComplete())
	{
		// Turn off all LEDs when motor is not in motion
		XGpio_DiscreteWrite(&LEDInst, LED_CHANNEL, 0x00);
	}
	else
	{
		// Animate LEDs based on the current step mode
		switch (currentStepMode)
		{
		case WAVE_DRIVE:
			// Implement LED animation for wave drive mode
			break;
		case FULL_STEP:
			// Implement LED animation for full step mode
			break;
		case HALF_STEP:
			// Implement LED animation for full step mode
			break;
		}
	}
}

void Stepper_SetupMoveInSteps(long absolutePositionToMoveToInSteps)
{
	long distanceToTravel_InSteps;

	// save the target location
	targetPosition_InSteps = absolutePositionToMoveToInSteps;

	// determine the period of the first step
	ramp_InitialStepPeriod = 1000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);

	// determine the period between steps when going at the desired velocity
	desiredStepPeriod = 1000.0 / desiredSpeed_InStepsPerSecond;

	// determine the number of steps needed to go from the desired velocity down to a
	// velocity of 0,  Steps = Velocity^2 / (2 * Acceleration)
	decelerationDistance_InSteps = (long)round((desiredSpeed_InStepsPerSecond *
												desiredSpeed_InStepsPerSecond) /
											   (2.0 * deceleration_InStepsPerSecondPerSecond));

	// determine the distance and direction to travel
	distanceToTravel_InSteps = targetPosition_InSteps - currentPosition_InSteps;
	if (distanceToTravel_InSteps < 0)
	{
		distanceToTravel_InSteps = -distanceToTravel_InSteps;
		direction_Scaler = -1;
	}
	else
	{
		direction_Scaler = 1;
	}

	// check if travel distance is too short to accelerate up to the desired velocity
	if (distanceToTravel_InSteps <= (decelerationDistance_InSteps * 2L))
	{
		decelerationDistance_InSteps = (distanceToTravel_InSteps / 2L);
	}
	// start the acceleration ramp at the beginning
	ramp_NextStepPeriod = ramp_InitialStepPeriod;
	acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E6;
	deceleration_InStepsPerUSPerUS = deceleration_InStepsPerSecondPerSecond / 1E6;
	startNewMove = TRUE;
}

_Bool Stepper_processMovement(void)
{
	unsigned long currentTime_InUS;
	unsigned long periodSinceLastStep_InUS;
	long distanceToTarget_InSteps;

	// check if already at the target position
	if (currentPosition_InSteps == targetPosition_InSteps)
	{
		return (TRUE);
	}

	// check if this is the first call to start this new move
	if (startNewMove)
	{
		ramp_LastStepTime = xTaskGetTickCount();
		startNewMove = FALSE;
	}

	// determine how much time has elapsed since the last step (Note 1: this method
	// works even if the time has wrapped. Note 2: all variables must be unsigned)
	currentTime_InUS = xTaskGetTickCount();

	periodSinceLastStep_InUS = currentTime_InUS - ramp_LastStepTime;

	// if it is not time for the next step, return
	if (periodSinceLastStep_InUS < (unsigned long)ramp_NextStepPeriod)
	{
		return (FALSE);
	}

	// determine the distance from the current position to the target
	distanceToTarget_InSteps = targetPosition_InSteps - currentPosition_InSteps;

	if (distanceToTarget_InSteps < 0)
	{
		distanceToTarget_InSteps = -distanceToTarget_InSteps;
	}

	// test if it is time to start decelerating, if so change from accelerating to
	// decelerating
	if (distanceToTarget_InSteps == decelerationDistance_InSteps)
	{
		acceleration_InStepsPerUSPerUS = -deceleration_InStepsPerUSPerUS;
	}

	// execute the step on the rising edge
	Stepper_setNextStep(direction_Scaler, WAVE_DRIVE);

	// update the current position and speed
	currentPosition_InSteps += direction_Scaler;

	currentStepPeriod = ramp_NextStepPeriod;

	// compute the period for the next step
	//   StepPeriod =
	//   LastStepPeriod * (1 - AccelerationInStepsPerUSPerUS * LastStepPeriod^2)
	ramp_NextStepPeriod = ramp_NextStepPeriod *
						  (1.0 - acceleration_InStepsPerUSPerUS * ramp_NextStepPeriod * ramp_NextStepPeriod);

	// clip the speed so that it does not accelerate beyond the desired velocity

	if (ramp_NextStepPeriod < desiredStepPeriod)
	{
		ramp_NextStepPeriod = desiredStepPeriod;
	}

	if (round(ramp_NextStepPeriod > 2))
	{
		vTaskDelay((TickType_t)ramp_NextStepPeriod - 1);
	}

	// update the acceleration ramp
	ramp_LastStepTime = currentTime_InUS;

	// check if the move has reached its final target position, return true if all done
	if (currentPosition_InSteps == targetPosition_InSteps)
	{
		currentStepPeriod = 0.0;
		return (TRUE);
	}
	return (FALSE);
}

///////////////////////////////////////////////////////////////////////////////
/*
 * Currently, Stepper_setNextWaveDriveStep advances the stepper motor to the next position in the
 * wave drive stepping sequence based on the 'direction' parameter.
 *
 * TODO 1: Refactor Function Signature
 *	 	- Refactor this function to:
 *  	  void Stepper_setNextStep(int direction, StepMode mode)
 *		- Add a step 'mode' parameter to allow for different stepping modes
 *		  (wave drive, full step, half step).
 *
 * TODO 2: Enum Definition
 *		- Define an enum 'StepMode' with values for each stepping mode:
 *  	  typedef enum { WAVE_DRIVE, FULL_STEP, HALF_STEP } StepMode;
 *		- This enum will facilitate switching between different motor control strategies.
 *
 * TODO 3: Implement Stepping Mode Logic
 *	 	- Inside Stepper_setNextStep, implement logic to handle each stepping mode.
 * 		- Use conditional branching (if-else) or switch-case to activate the correct coil
 *   	  pattern based on 'step_mode'.
 *
 * TODO 4: Coil Activation Patterns
 * 		- Ensure each stepping mode correctly sequences through its respective coil activation patterns.
 * 		- Reference the defined bit patterns for coil activation in different modes.
 *
 * The goal is to dynamically adapt the motor's stepping behavior based on the
 * selected mode, enhancing the functionality of your motor control application.
 *
 * NOTE: The current implementation is limited to the wave drive mode. Your task
 *       is to extend it for full step and half step modes, considering the step 'mode'
 *       to determine the sequence of coil activations.
 */

// Updated function with step mode
void Stepper_setNextStep(int direction, StepMode mode)
{
	// Compute the next phase number based on the direction
	stepPhase += direction;
	if (stepPhase < 0)
	{
		stepPhase += (mode == HALF_STEP) ? 8 : 4;
	}
	stepPhase %= (mode == HALF_STEP) ? 8 : 4;

	// Act based on the mode
	switch (mode)
	{
	case WAVE_DRIVE:
		switch (stepPhase)
		{
		case 0:
			XGpio_DiscreteWrite(&PModMotorInst, 1, WAVE_DRIVE_1);
			break;
		case 1:
			XGpio_DiscreteWrite(&PModMotorInst, 1, WAVE_DRIVE_2);
			break;
		case 2:
			XGpio_DiscreteWrite(&PModMotorInst, 1, WAVE_DRIVE_3);
			break;
		case 3:
			XGpio_DiscreteWrite(&PModMotorInst, 1, WAVE_DRIVE_4);
			break;
		}
		break;
	case FULL_STEP:
		switch (stepPhase)
		{
		case 0:
			XGpio_DiscreteWrite(&PModMotorInst, 1, FULL_STEP_1);
			break;
		case 1:
			XGpio_DiscreteWrite(&PModMotorInst, 1, FULL_STEP_2);
			break;
		case 2:
			XGpio_DiscreteWrite(&PModMotorInst, 1, FULL_STEP_3);
			break;
		case 3:
			XGpio_DiscreteWrite(&PModMotorInst, 1, FULL_STEP_4);
			break;
		}
		break;
	case HALF_STEP:
		switch (stepPhase)
		{
		case 0:
			XGpio_DiscreteWrite(&PModMotorInst, 1, HALF_STEP_1);
			break;
		case 1:
			XGpio_DiscreteWrite(&PModMotorInst, 1, HALF_STEP_2);
			break;
		case 2:
			XGpio_DiscreteWrite(&PModMotorInst, 1, HALF_STEP_3);
			break;
		case 3:
			XGpio_DiscreteWrite(&PModMotorInst, 1, HALF_STEP_4);
			break;
		case 4:
			XGpio_DiscreteWrite(&PModMotorInst, 1, HALF_STEP_5);
			break;
		case 5:
			XGpio_DiscreteWrite(&PModMotorInst, 1, HALF_STEP_6);
			break;
		case 6:
			XGpio_DiscreteWrite(&PModMotorInst, 1, HALF_STEP_7);
			break;
		case 7:
			XGpio_DiscreteWrite(&PModMotorInst, 1, HALF_STEP_8);
			break;
		}
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////

/*
 * disable the motor, all the drive coils are turned off to save power
 * and reduce heat when motor is not in motion, any movement command will
 * automatically re-enable the stepper motor
 */

void Stepper_disableMotor()
{
	XGpio_DiscreteWrite(&PModMotorInst, 1, WINDINGS_OFF);
}

float Stepper_getCurrentVelocityInStepsPerSecond()
{
	if (currentStepPeriod == 0.0)
	{
		return (0);
	}
	else
	{
		if (direction_Scaler > 0)
		{
			return (1000.0 / currentStepPeriod);
		}
		else
		{
			return (-1000.0 / currentStepPeriod);
		}
	}
}

_Bool Stepper_motionComplete()
{
	if (currentPosition_InSteps == targetPosition_InSteps)
	{
		return (TRUE);
	}
	else
	{
		return (FALSE);
	}
}
