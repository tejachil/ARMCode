#ifndef ROVERCONTROL_H
#define ROVERCONTROL_H

#include "uartDriver.h"

#define DISTANCE_BETWEEN_IR 7
#define PARALLEL_THRESHOLD 3

//number of things
#define NUMBER_OF_SAMPLES 5
#define NUMBER_OF_SENSORS 6

//Sensors
#define LEFT_SHORT_SENSOR 0
#define RIGHT_SHORT_SENSOR 1
#define LEFT_MEDIUM_SENSOR 2
#define RIGHT_MEDIUM_SENSOR 3
#define LEFT_LONG_SENSOR 4
#define RIGHT_LONG_SENSOR 5

typedef struct __RoverControlStruct {
	UARTstruct *uartDevice;
	xQueueHandle inQ;
	//define average value of sensors distance
	float sensorDistance[NUMBER_OF_SENSORS];
	float sensorDistanceSamples[NUMBER_OF_SENSORS][NUMBER_OF_SAMPLES];
	//define samples values for sensors
	/*float leftShortSensorSamples[NUMBER_OF_SAMPLES];
	float rightShortSensor[NUMBER_OF_SAMPLES];
	float leftMediumSensor[NUMBER_OF_SAMPLES];
	float rightMediumSensor[NUMBER_OF_SAMPLES];
	float leftLongSensor[NUMBER_OF_SAMPLES];
	float rightLongSensor[NUMBER_OF_SAMPLES];*/
	//counter for sampling
	int samplingCounter;
	//define angles
	float shortSensorAngle;
} RoverControlStruct;

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart);

#endif