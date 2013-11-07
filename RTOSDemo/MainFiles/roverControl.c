#include "roverControl.h"
#include "sensorFunctions.h"
#include "roverFunctions.h"
#include <math.h>

#define roverControlQLen (10)

static RoverMapStruct *roverMap;

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart, RoverMapStruct *roverMapStruct) {
	roverControlData->uartDevice = uart;

	roverMap = roverMapStruct;

	// Create the queue that will be used to talk to this task
	if ((roverControlData->inQ = xQueueCreate(roverControlQLen,sizeof(public_message_t))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	/*if (xTaskCreate( roverControlTask, ( signed char * ) "Rover Control", roverSTACK_SIZE, (void *) roverControlData, uxPriority, ( xTaskHandle * ) NULL ) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}*/ // Teja moved this to the webserver
}

void roverControlTask( void *param ){
	// Get the parameters
	RoverControlStruct *roverControlData = (RoverControlStruct *) param;
	// Variable to hold received messages
	public_message_t receivedMsg;

	//initializing varablies
	roverControlData->state = INIT;
	roverControlData->samplingCounter = 0;

	// Sensor data request message
#warning "Still using old UARTmsg struct"
	UARTmsg sensorRequestMsg;
	sensorRequestMsg.msgType = PUB_MSG_T_SENS_DIST;
	sensorRequestMsg.msgID = 0; // The count will be updated before sending each request
	sensorRequestMsg.txLen = 0; // No data is included in the request

	// Request sensor data
	// Update the count and send the request
	sensorRequestMsg.msgID = public_message_get_count(PUB_MSG_T_SENS_DIST);
	uartEnQ(roverControlData->uartDevice, sensorRequestMsg.msgType, sensorRequestMsg.msgID, sensorRequestMsg.txLen,
		sensorRequestMsg.data);

	for(;;)
	{
		// 1. Read sensor data response from the conductor (blocks until
		// data is available).
		if (xQueueReceive(roverControlData->inQ, (void *) &receivedMsg, portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		vtLEDToggle(0x01);
		
		//printf(" :L: ");
		// 2. Request new sensor data.
		// Update the count and send the request
		sensorRequestMsg.msgID = public_message_get_count(PUB_MSG_T_SENS_DIST);
		uartEnQ(roverControlData->uartDevice, sensorRequestMsg.msgType, sensorRequestMsg.msgID, sensorRequestMsg.txLen,
			sensorRequestMsg.data);

		// 3. Process message received in step 1, contained in receivedMsg.
		// See public_messages.h for the structure of receivedMsg (it is type "public_message_t").


		if (receivedMsg.message_type == PUB_MSG_T_SENS_DIST){
			//read a new message
			readNewMsg(roverControlData, &receivedMsg);
			averageValues(roverControlData);
			convertToDistance(roverControlData);
			findAngles(roverControlData);
			printFloat("SiRear:", roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR], 0);
			printFloat("SiFront:", roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR], 0);
			printFloat("FroLeft:", roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR], 0);
			printFloat("FroRight:", roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR], 0);
			printFloat("Angle:", roverControlData->shortSensorAngle, 1);

			switch(roverControlData->state){
				case INIT:
					//send command to move rover
					moveRover(roverControlData);
					break;
				case TRAVERSAL:
					if(isFrontCloseToWall(roverControlData) == 1){
						turnRover(roverControlData);
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_LEFT){
						//fix to Left
						fixRover(roverControlData, FIX_FRONT_LEFT);
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_RIGHT){
						//fix to Right
						fixRover(roverControlData, FIX_FRONT_RIGHT);
					}
					break;
				case FIX:
					if(isFrontCloseToWall(roverControlData) == 1){
						turnRover(roverControlData);
					}
					else if(isRoverParallelToWall(roverControlData) == PARALLEL){
						moveRover(roverControlData);
					}
					break;
				case TURN:
					if(isFrontCloseToWall(roverControlData) == 0 && isRoverParallelToWall(roverControlData) == PARALLEL){
						moveRover(roverControlData);
					}
					break;
				/*case STOP:
					//if(isRoverParallelToWall(roverControlData) == PARALLEL && isSensorInRange(roverControlData) == 1){
					if(isFrontCloseToWall(roverControlData) == 0){
						//send command to move
						moveRover(roverControlData);
					}
					break;*/
			}
		}
		else if (receivedMsg.message_type == PUB_MSG_T_ENCODER_DATA){
			roverControlData->encoderDistance = getEncoderDistance(receivedMsg.data[2], receivedMsg.data[0] || (receivedMsg.data[1] << 8));
		}
		else{
			;
		}
	}
}
