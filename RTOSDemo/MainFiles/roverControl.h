#ifndef ROVERCONTROL_H
#define ROVERCONTROL_H

#include "uartDriver.h"

#define DISTANCE_BETWEEN_IR 7

typedef struct __RoverControlStruct {
	UARTstruct *uartDevice;
	xQueueHandle inQ;
	float leftShortSensor;
	float rightShortSensor;
	float leftMediumSensor;
	float rightMediumSensor;
	float leftLongSensor;
	float rightLongSensor;
	float shortSensorAngle;
} RoverControlStruct;

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart);

#endif