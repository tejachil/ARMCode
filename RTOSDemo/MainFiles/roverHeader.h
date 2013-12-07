#ifndef ROVERHEADER_H
#define ROVERHEADER_H

#include "uartDriver.h"
#include "public_messages.h"

//constant values
#define DISTANCE_BETWEEN_SIDE_SHORT		7.625
#define DISTANCE_BETWEEN_FRONT_MEDIUM	4.00
#define PARALLEL_THRESHOLD 				0.6
#define SIDE_SENSOR_RANGE 				15
#define FRONT_STOP_DISTANCE				9
#define FRONT_AQUIRE_ANGLE_DISTANCE 	15
#define TOO_CLOSE_THRESHOLD				3
#define SPEED_RANGE						3
#define FRONT_THRESHOLD_ADJUSTMENT 		0.02

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
#define FIX 						7
#define TOO_CLOSE					3
#define TOO_CLOSE_FORWARD			4
#define TURN						5
#define STOP	  					6
#define STRAIGHT					8
#define GOTO						9

//front wall states
#define FAR_FRONT_WALL				0
#define ACQUIRE_FRONT_ANGLE			1				
#define CLOSE_FRONT_WALL			2

//parallel states
#define PARALLEL 					0
#define FIX_FRONT_LEFT  			1
#define FIX_FRONT_RIGHT 			2
#define TOO_CLOSE_SIDE				3
#define TOO_FAR_SIDE				4

//encoder defines
#define TICKS_PER_REVOLUTION		6207.34375 // spec says 6000
#define WHEEL_CIRCUMFERENCE			14.9 //inches
#define ROVER_LENGTH				11.0
#define ROVER_LENGTH_WIDTH_OFFSET	4.6

#define ANGLE_SAMPLE_COUNT			200

#define REVOLVE_ANGLE_STEP			30.0
#define REVOLVE_ANGLE_FUDGE_DEV		3.0

//task ID values
#define REGULAR						1
#define REVOLVE						2

//map parameters
#define MAP_SCALE_FACTOR			4
#define MAP_X_OFFSET				50
#define MAP_Y_OFFSET				350

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
	float frontSensorAngle;
} RoverControlStruct;

#endif