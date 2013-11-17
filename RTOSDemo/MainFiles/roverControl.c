#include "roverControl.h"
#include "sensorFunctions.h"
#include "roverFunctions.h"
#include <math.h>

#define roverControlQLen (10)
#define REQUEST_TYPE_DISTANCE		0
#define REQUEST_TYPE_ENCODER		1
#define REQUEST_TYPE_TURN_STATUS	2

float difference;
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

	UARTmsg turnRequestMsg;
	turnRequestMsg.msgType = PUB_MSG_T_TURN_STATUS;
	turnRequestMsg.msgID = 0; // The count will be updated before sending each request
	turnRequestMsg.txLen = 0; // No data is included in the request
	
	uint8_t requestType = REQUEST_TYPE_DISTANCE;
	uint8_t encoderReceived = 0;
	uint8_t turnStatusReceived = 0;
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
		vtLEDToggle(0x80);
		
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
			/*printFloat("SiRear:", roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR], 0);
			printFloat("SiFront:", roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR], 0);
			printFloat("FroLeft:", roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR], 0);
			printFloat("FroRight:", roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR], 0);
			printFloat("Angle:", roverControlData->shortSensorAngle, 1);*/

			switch(roverControlData->state){
				case INIT:
					//send command to move rover
					moveRover(roverControlData);
					break;
				case TRAVERSAL:
					vtLEDOn(0x01);	
					vtLEDOff(0x02);
					vtLEDOff(0x04);
					vtLEDOff(0x08);

					//if close to front wall move to stop state
					if(isFrontCloseToWall(roverControlData) == 1){
						stopRover(roverControlData);
						requestType = REQUEST_TYPE_ENCODER;
					}
					else if(isRoverParallelToWall(roverControlData) == TOO_CLOSE_SIDE){
						roverControlData->state = TOO_CLOSE;	
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_LEFT){
						difference = roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR];
						if(difference > (PARALLEL_THRESHOLD + 2*SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);
						}else if(difference > (PARALLEL_THRESHOLD + SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);
						}else if(difference > PARALLEL_THRESHOLD){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);	
						}
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_RIGHT){
						difference = roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR];
						if(difference > (PARALLEL_THRESHOLD + 2*SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}else if(difference > (PARALLEL_THRESHOLD + SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}else if(difference > PARALLEL_THRESHOLD){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}	
					}
					break;
				case FIX:
					vtLEDOff(0x01);	
					vtLEDOn(0x02);
					vtLEDOff(0x04);
					vtLEDOff(0x08);
					
					if(isFrontCloseToWall(roverControlData) == 1){
						stopRover(roverControlData);
						requestType = REQUEST_TYPE_ENCODER;
					}
					else if(isRoverParallelToWall(roverControlData) == PARALLEL){
						vtLEDOn(0x08);
						moveRover(roverControlData);
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_LEFT){
						difference = roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR];
						if(difference > (PARALLEL_THRESHOLD + 2*SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);
						}else if(difference > (PARALLEL_THRESHOLD + SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);
						}else if(difference > PARALLEL_THRESHOLD){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);	
						}
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_RIGHT){
						difference = roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR];
						if(difference > (PARALLEL_THRESHOLD + 2*SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}else if(difference > (PARALLEL_THRESHOLD + SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}else if(difference > PARALLEL_THRESHOLD){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}	
					}
					break;
				case TOO_CLOSE:
					vtLEDOff(0x01);	
					vtLEDOff(0x02);
					vtLEDOn(0x04);
					vtLEDOff(0x08);
					if(isFrontCloseToWall(roverControlData) == 1){
						stopRover(roverControlData);
						requestType = REQUEST_TYPE_ENCODER;
					}
					// front side is too close
					else if(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < TOO_CLOSE_THRESHOLD){
						fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						roverControlData->state = TOO_CLOSE;
					}
					if((roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]) > 0.7){
							moveRover(roverControlData);
							roverControlData->state = TOO_CLOSE_FRONT;
					}
					break;
				case TOO_CLOSE_FRONT:
					vtLEDOff(0x01);	
					vtLEDOff(0x02);
					vtLEDOn(0x04);
					vtLEDOff(0x08);
					if(isFrontCloseToWall(roverControlData) == 1){
						stopRover(roverControlData);
						requestType = REQUEST_TYPE_ENCODER;
					}
					else if(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < TOO_CLOSE_THRESHOLD
					 || roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] < TOO_CLOSE_THRESHOLD){
						moveRover(roverControlData);
						roverControlData->state = TOO_CLOSE_FRONT;
					}else{
						// leave this task
						moveRover(roverControlData);
					}
				break;
				//this state is called when rover is close to a front wall
				//at this point we request encoder readings from the rover
				//control PIC after that we move to turn state
				case STOP:
					//if(isRoverParallelToWall(roverControlData) == PARALLEL && isSensorInRange(roverControlData) == 1){
					/*if (totalExternalAngle >= 370.0){
						vtLEDOn(0x80);
					}
					else */
					// wait here till data is received
					if(encoderReceived != 0){
						//vtLEDOn(0x20);
						//vtLEDOn(0x20);
						newCorner.distFromSide = (roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] + roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR])/2;
						newCorner.angleCorner = 90;
						//totalExternalAngle += newCorner.angleCorner;
						//newCorner.distSide += ROVER_LENGTH;// + (roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR] + roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR])/2;

						if(xQueueSend(roverMap->inQ, &newCorner,portMAX_DELAY) != pdTRUE)	VT_HANDLE_FATAL_ERROR(0);
						//vtLEDOn(0x40);
						//turn rover and keep asking for turning status
						turnRover(roverControlData);
						requestType = REQUEST_TYPE_TURN_STATUS;
						encoderReceived = 0;
					}
					break;
				case TURN:	
					vtLEDOff(0x01);	
					vtLEDOff(0x02);
					vtLEDOff(0x04);
					vtLEDOn(0x08);
					/*if(isFrontCloseToWall(roverControlData) == 0 && isRoverParallelToWall(roverControlData) == PARALLEL){
						vtLEDOn(0x08);
						moveRover(roverControlData);
					}*/
					// wait here till rover turn by the specified angle
					if(turnStatusReceived != 0){
						moveRover(roverControlData);
						turnStatusReceived = 0;
					}
					break;
			}
		}
		//received encoder data
		else if (receivedMsg.message_type == PUB_MSG_T_ENCODER_DATA){
			encoderReceived = 1;
			newCorner.distSide = (receivedMsg.data[0] + (receivedMsg.data[1] << 8))/TICKS_PER_REVOLUTION;
			newCorner.distSide += receivedMsg.data[2];
			newCorner.distSide = newCorner.distSide * WHEEL_CIRCUMFERENCE;
			newCorner.distSide = newCorner.distSide + ROVER_LENGTH + FRONT_STOP_DISTANCE*1.0;
			//set request type to sensor distance 
			requestType = REQUEST_TYPE_DISTANCE;
			//getEncoderDistance(receivedMsg.data[2], );
		}
		//received turn status
		else if (receivedMsg.message_type ==  PUB_MSG_T_TURN_STATUS){
			if(receivedMsg.data[0]){
				turnStatusReceived = 1;
				//set request type to sensor distance 
				requestType = REQUEST_TYPE_DISTANCE;
			}
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
		else if (requestType == REQUEST_TYPE_ENCODER){
			// Request encoder data
			encoderRequestMsg.msgID = public_message_get_count(PUB_MSG_T_ENCODER_DATA);
			uartEnQ(roverControlData->uartDevice, encoderRequestMsg.msgType, encoderRequestMsg.msgID, encoderRequestMsg.txLen,
				encoderRequestMsg.data);
		} 
		else if (requestType == REQUEST_TYPE_TURN_STATUS){
			// Request turn status
			turnRequestMsg.msgID = public_message_get_count(PUB_MSG_T_TURN_STATUS);
			uartEnQ(roverControlData->uartDevice, turnRequestMsg.msgType, turnRequestMsg.msgID, turnRequestMsg.txLen,
				turnRequestMsg.data);
		}
	}
}
