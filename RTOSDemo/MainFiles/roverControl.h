#ifndef ROVERCONTROL_H
#define ROVERCONTROL_H

#include "uartDriver.h"

#define DISTANCE_BETWEEN_IR 7

typedef struct __RoverControlStruct {
	UARTstruct *uartDevice;
	xQueueHandle inQ;
	float leftShortSensor = 0, rightShortSensor = 0,
		  leftMediumSensor = 0, rightMediumSensor = 0,
		  leftLongSensor = 0, rightLongSensor = 0,
		  shortSensorAngle = 0;
} RoverControlStruct;

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart);
// defined functions for sensors calculations
void readNewMsg(RoverControlStruct *roverControlData, public_message_t receivedMsg);
void convertToDistance(RoverControlStruct *roverControlData);
void checkSensorsRange(RoverControlStruct *roverControlData);
void findAngles(RoverControlStruct *roverControlData);

#endif