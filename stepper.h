/*
 *  Created on: Mar 24, 2021
 *  Modified by : Shyama Gandhi
 *  Modified by : Antonio Andara on March 19, 2024
 *  The driver has been modified according to the lab needs and the code is adapted from TinyStepper_28BYJ_48 arduino library
 *
 */

#ifndef SRC_STEPPER_H_
#define SRC_STEPPER_H_

#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xparameters.h"
#include "xgpio.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_printf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "math.h"

#define NO_OF_STEPS_PER_REVOLUTION_HALF_DRIVE 4096
#define NO_OF_STEPS_PER_REVOLUTION_FULL_DRIVE 2048

XGpio PModMotorInst;
#define PMOD_MOTOR_DEVICE_ID XPAR_STEPPER_MOTOR_DEVICE_ID

#define WINDINGS_OFF 0b0000

/***********************Stepper Motor Control Patterns***********************/

// Wave Drive Mode: Activates a single coil at a time.
#define WAVE_DRIVE_1 0b0001 // Coil 1 activated
#define WAVE_DRIVE_2 0b0010 // Coil 2 activated
#define WAVE_DRIVE_3 0b0100 // Coil 3 activated
#define WAVE_DRIVE_4 0b1000 // Coil 4 activated

/*
 * TODO 1: Define Coil Activation Patterns for Full Step Mode
 * Full Step Mode activates two adjacent coils at a time to produce higher torque.
 * Define the bit patterns for each step in the sequence.
 * NOTE: Ensure each pattern correctly represents the simultaneous activation of two adjacent coils.
 */

/*
 * TODO 2: Define Coil Activation Patterns for Half Step Mode
 * Half Step Mode interleaves single and dual coil activations for higher resolution.
 * Define the bit patterns for each step in the sequence.
 * NOTE: Half Step Mode typically doubles the number of steps per revolution, offering finer control.
 */

/**************************************************************************/

int _signals_to_motor[4];
int stepPhase;
int direction_Scaler;

float desiredSpeed_InStepsPerSecond;
float acceleration_InStepsPerSecondPerSecond;
float deceleration_InStepsPerSecondPerSecond;
float currentStepPeriod;

long currentPosition_InSteps;
long targetPosition_InSteps;
long decelerationDistance_InSteps;

float ramp_InitialStepPeriod;
float desiredStepPeriod;
float ramp_NextStepPeriod;
float acceleration_InStepsPerUSPerUS;
float deceleration_InStepsPerUSPerUS;

_Bool startNewMove;

unsigned long ramp_LastStepTime;

/*
 * TODO 3: Define an Enum for Stepper Motor Control Modes
 * The task involves creating an enumeration ('enum') to represent the different control modes
 * available for operating the stepper motor. This enum will be used throughout your program
 * to switch between different stepping strategies, enhancing the motor control's flexibility and precision.
 *
 * Define the 'StepMode' enum with three values: WAVE_DRIVE, FULL_STEP, and HALF_STEP.
 * These modes correspond to different coil activation patterns that the stepper motor can operate in.
 * Additionally, declare a variable 'step_mode' of type 'StepMode'. This variable will hold
 * the current stepping mode that your program should use for controlling the stepper motor.
 */

// struct for motor parameters
typedef struct
{
	long final_position;
	long currentposition_in_steps;
	float rotational_speed;
	float rotational_acceleration;
	float rotational_deceleration;
} MotorParameters;

typedef enum
{
	WAVE_DRIVE,
	FULL_STEP,
	HALF_STEP
} StepMode;

/////////////////////////////////////////////////////
_Bool Stepper_processMovement(void);
_Bool Stepper_motionComplete();
float Stepper_getCurrentVelocityInStepsPerSecond();
long Stepper_getCurrentPositionInSteps();

void Stepper_PMOD_pins_to_output();
void Stepper_Initialize();
void Stepper_setCurrentPositionInSteps(long currentPositionInSteps);
void Stepper_SetupStop();
void Stepper_setSpeedInStepsPerSecond(float speedInStepsPerSecond);
void Stepper_setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
void Stepper_setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond);

// Update the function prototype to include the StepMode
void Stepper_setNextStep(int direction, StepMode mode);

void Stepper_moveRelativeInSteps(long distanceToMoveInSteps);
void Stepper_SetupRelativeMoveInSteps(long distanceToMoveInSteps);
void Stepper_SetupMoveInSteps(long absolutePositionToMoveToInSteps);
void Stepper_moveToPositionInSteps(long absolutePositionToMoveToInSteps);
void Stepper_setNextWaveDriveStep(int direction);
void Stepper_disableMotor();

#endif /* SRC_STEPPER_H_ */
