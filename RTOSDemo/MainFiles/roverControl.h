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

//State machine
#define INIT 0
#define TRAVERSAL 1
#define STOP	  2

typedef struct __RoverControlStruct {
	UARTstruct *uartDevice;
	xQueueHandle inQ;
	//define average value of sensors distance
	float sensorDistance[NUMBER_OF_SENSORS];
	float sensorDistanceSamples[NUMBER_OF_SENSORS][NUMBER_OF_SAMPLES];
	int state;
	//counter for sampling
	int samplingCounter;
	//define angles
	float shortSensorAngle;
} RoverControlStruct;

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart);

#endif