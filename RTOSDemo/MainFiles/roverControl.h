#ifndef ROVERCONTROL_H
#define ROVERCONTROL_H

#include "uartDriver.h"

#define DISTANCE_BETWEEN_IR 7
#define NUMBER_OF_SAMPLES 5
#define PARALLEL_THRESHOLD 1

typedef struct __RoverControlStruct {
	UARTstruct *uartDevice;
	xQueueHandle inQ;
	//define average value of sensors distance
	float leftShortSensor;
	float rightShortSensor;
	float leftMediumSensor;
	float rightMediumSensor;
	float leftLongSensor;
	float rightLongSensor;
	//define samples values for sensors
	float leftShortSensorSamples[NUMBER_OF_SAMPLES];
	float rightShortSensor[NUMBER_OF_SAMPLES];
	float leftMediumSensor[NUMBER_OF_SAMPLES];
	float rightMediumSensor[NUMBER_OF_SAMPLES];
	float leftLongSensor[NUMBER_OF_SAMPLES];
	float rightLongSensor[NUMBER_OF_SAMPLES];
	//counter for sampling
	int samplingCounter;
	//define angles
	float shortSensorAngle;
} RoverControlStruct;

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart);

#endif