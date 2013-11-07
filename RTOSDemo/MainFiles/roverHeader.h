#ifndef ROVERHEADER_H
#define ROVERHEADER_H

#include "uartDriver.h"
#include "public_messages.h"

//constant values
#define DISTANCE_BETWEEN_IR 		6.5
#define PARALLEL_THRESHOLD 			0.6
#define SIDE_SENSOR_RANGE 			15
#define FRONT_STOP_DISTANCE			6

//number of things
#define NUMBER_OF_SAMPLES 			5
#define NUMBER_OF_SENSORS 			6

//Sensors
#define SIDE_REAR_SHORT_SENSOR 		0
#define SIDE_FRONT_SHORT_SENSOR 	1
#define FRONT_LEFT_MEDIUM_SENSOR 	2
#define FRONT_RIGHT_MEDIUM_SENSOR 	3
#define FRONT_LEFT_LONG_SENSOR 		4
#define FRONT_RIGHT_LONG_SENSOR 	5

//State machine
#define INIT 						0
#define TRAVERSAL 					1
#define FIX 						2
#define TURN						3
#define STOP	  					4

//parallel states
#define PARALLEL 					0
#define FIX_FRONT_LEFT  			1
#define FIX_FRONT_RIGHT 			2

//encoder defines
#define TICKS_PER_REVOLUTION		6000 //tyler was measuring 5250 and actual is 6000
#define WHEEL_CIRCUMFERENCE			14.1 //inches
#define ROVER_LENGTH				12

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

#endif