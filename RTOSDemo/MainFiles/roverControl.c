#include "roverControl.h"
#include "sensorFunctions.h"
#include "roverFunctions.h"
#include <math.h>

#define roverControlQLen (10)
#define REQUEST_TYPE_DISTANCE	0
#define REQUEST_TYPE_ENCODER	1

static RoverMapStruct *roverMap;

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart, RoverMapStruct *roverMapStruct, xTaskHandle taskHandle) {
	roverControlData->uartDevice = uart;

	roverMap = roverMapStruct;

	// Create the queue that will be used to talk to this task
	if ((roverControlData->inQ = xQueueCreate(roverControlQLen,sizeof(public_message_t))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	if (xTaskCreate( roverControlTask, ( signed char * ) "Rover Control", roverSTACK_SIZE, (void *) roverControlData, uxPriority, ( xTaskHandle * ) taskHandle ) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
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

	UARTmsg encoderRequestMsg;
	encoderRequestMsg.msgType = PUB_MSG_T_ENCODER_DATA;
	encoderRequestMsg.msgID = 0; // The count will be updated before sending each request
	encoderRequestMsg.txLen = 0; // No data is included in the request
	
	uint8_t requestType = REQUEST_TYPE_DISTANCE;
	uint8_t encoderReceived = 0;
	uint8_t isFirstCorner = 0;
	MapCorner newCorner;
	newCorner.distSide = 0;
	//double totalExternalAngle = 0;

	// Update the count and send the request
	sensorRequestMsg.msgID = public_message_get_count(PUB_MSG_T_SENS_DIST);
	uartEnQ(roverControlData->uartDevice, sensorRequestMsg.msgType, sensorRequestMsg.msgID, sensorRequestMsg.txLen,
		sensorRequestMsg.data);

	for(;;){
		// 1. Read sensor data response from the conductor (blocks until
		// data is available).
		if (xQueueReceive(roverControlData->inQ, (void *) &receivedMsg, portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		vtLEDToggle(0x01);
		
		//printf(" :L: ");
		// 2. Request new sensor data.

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
						stopRover(roverControlData);
						requestType == REQUEST_TYPE_ENCODER;
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
						stopRover(roverControlData);
						requestType == REQUEST_TYPE_ENCODER;
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
				case STOP:
					vtLEDOn(0x40);
					//if(isRoverParallelToWall(roverControlData) == PARALLEL && isSensorInRange(roverControlData) == 1){
					/*if (totalExternalAngle >= 370.0){
						vtLEDOn(0x80);
					}
					else */if(encoderReceived != 0){
						vtLEDOn(0x20);
						//vtLEDOn(0x20);
						newCorner.distFromSide = (roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] + roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR])/2;
						newCorner.angleCorner = 90;
						//totalExternalAngle += newCorner.angleCorner;
						//newCorner.distSide += ROVER_LENGTH;// + (roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR] + roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR])/2;

						if(xQueueSend(roverMap->inQ, &newCorner,portMAX_DELAY) != pdTRUE)	VT_HANDLE_FATAL_ERROR(0);
						//vtLEDOn(0x40);
						turnRover(roverControlData);
						encoderReceived = 0;
					}
					break;
			}
		}
		else if (receivedMsg.message_type == PUB_MSG_T_ENCODER_DATA){
			vtLEDOn(0x80);
			encoderReceived = 1;
			newCorner.distSide = (receivedMsg.data[0] + (receivedMsg.data[1] << 8))/TICKS_PER_REVOLUTION;
			newCorner.distSide += receivedMsg.data[2];
			newCorner.distSide = newCorner.distSide * WHEEL_CIRCUMFERENCE;
			newCorner.distSide = newCorner.distSide + ROVER_LENGTH + FRONT_STOP_DISTANCE*1.0;
			requestType == REQUEST_TYPE_DISTANCE;
			//getEncoderDistance(receivedMsg.data[2], );
		}
		else{ // unhandled message type
			VT_HANDLE_FATAL_ERROR(0);
		}

		if(requestType == REQUEST_TYPE_DISTANCE){
			// Request sensor distance data
			// Update the count and send the request
			sensorRequestMsg.msgID = public_message_get_count(PUB_MSG_T_SENS_DIST);
			uartEnQ(roverControlData->uartDevice, sensorRequestMsg.msgType, sensorRequestMsg.msgID, sensorRequestMsg.txLen,
				sensorRequestMsg.data);
			}
		else{
			encoderRequestMsg.msgID = public_message_get_count(PUB_MSG_T_ENCODER_DATA);
			uartEnQ(roverControlData->uartDevice, encoderRequestMsg.msgType, encoderRequestMsg.msgID, encoderRequestMsg.txLen,
				encoderRequestMsg.data);
			//requestType == REQUEST_TYPE_DISTANCE;
		}
	}
}
