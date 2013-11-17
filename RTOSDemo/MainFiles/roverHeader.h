#ifndef ROVERHEADER_H
#define ROVERHEADER_H

#include "uartDriver.h"
#include "public_messages.h"

//constant values
#define DISTANCE_BETWEEN_IR 		6.5
#define PARALLEL_THRESHOLD 			1.2
#define SIDE_SENSOR_RANGE 			15
#define FRONT_STOP_DISTANCE			10
#define TOO_CLOSE_THRESHOLD			6
#define SPEED_RANGE					3

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
#define TOO_CLOSE					3
#define TOO_CLOSE_FRONT				4
#define TURN						5
#define STOP	  					6

//parallel states
#define PARALLEL 					0
#define FIX_FRONT_LEFT  			1
#define FIX_FRONT_RIGHT 			2
#define TOO_CLOSE_SIDE				3
#define TOO_FAR_SIDE				4

//encoder defines
#define TICKS_PER_REVOLUTION		5250.0 //tyler was measuring 5250 and actual is 6000
#define WHEEL_CIRCUMFERENCE			14.1 //inches
#define ROVER_LENGTH				12.0

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