#include "roverControl.h"
#include "sensorFunctions.h"
#include "roverFunctions.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define roverControlQLen (10)
#define REQUEST_TYPE_DISTANCE		0
#define REQUEST_TYPE_ENCODER		1
#define REQUEST_TYPE_TURN_STATUS	2
#define REQUEST_TYPE_NULL			3

float difference;
static RoverMapStruct *roverMap;
static int anglesSamples[ANGLE_SAMPLE_COUNT];
int sideAngleCount = 0;

int cmpfunc (const void * a, const void * b);

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart, RoverMapStruct *roverMapStruct, xTaskHandle* taskHandle) {
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

	int val = 0;

	MapCorner newCorner;
	newCorner.distSide = 0;
	//double totalExternalAngle = 0;

	// Angle polling averages
	double anglePollTotal = 0.0, thresholdAnglePollTotal = 0.0;
	uint8_t anglePollCount = 0, thresholdAnglePollCount = 0;

	double totalRevolve = 0;

	static double tempDist = 0.0;

	// Update the count and send the request
	sensorRequestMsg.msgID = public_message_get_count(PUB_MSG_T_SENS_DIST);
	uartEnQ(roverControlData->uartDevice, sensorRequestMsg.msgType, sensorRequestMsg.msgID, sensorRequestMsg.txLen,
		sensorRequestMsg.data);
	printf("Hello World!");

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
			
			/*printFloat("SiRear:", roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR], 0);
			printFloat("SiFront:", roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR], 0);
			printFloat("FroLeft:", roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR], 0);
			printFloat("FroRight:", roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR], 0);
			printFloat("Angle:", roverControlData->frontSensorAngle, 1);*/
			/*sprintf(buf, "%f\n%f", roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]
								 , roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR]);*/

			if(frontWallStatus(roverControlData, thresholdAnglePollTotal, thresholdAnglePollCount) == ACQUIRE_FRONT_ANGLE){
				double sideAngle = 0;
				if(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]){
		    		sideAngle = atanf((roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]-roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR])/DISTANCE_BETWEEN_SIDE_SHORT) * 180/M_PI;
				}
				else{
					sideAngle = -1*atanf((roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR]-roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR])/DISTANCE_BETWEEN_SIDE_SHORT) * 180/M_PI;
				}
				newCorner.tempBefore = sideAngle;
			}
			
			// Check if goto location is set from gui and do that if it is set
			if(roverMap->gotoX > 0.0 && roverMap->gotoY > 0.0){
				roverControlData->state = GOTO;
			}

			if(polygonComplete() > 0 && roverControlData->state == TRAVERSAL){
				stopRover(roverControlData);
				roverControlData->state = NULL_STATE;
				requestType = REQUEST_TYPE_DISTANCE;
			}

			switch(roverControlData->state){
				case INIT:
					if((roverMap->taskFlags)&REVOLVE){
						//vtLEDOn(0x40);
						// TODO: revolve the rover
						totalRevolve = 0;
						turnRover(roverControlData, REVOLVE_ANGLE_STEP - REVOLVE_ANGLE_FUDGE_DEV);
						roverControlData->state = REVOLVE;
					}
					else{
						//vtLEDOff(0x40);
						//send command to move rover
						moveRover(roverControlData);
					}
					break;
				case REVOLVE:
					// TODO: REVOLVE STATE
					// some calucations
					if(turnStatusReceived != 0){
						totalRevolve += REVOLVE_ANGLE_STEP;
						if(totalRevolve < (360)){
							turnRover(roverControlData, REVOLVE_ANGLE_STEP - REVOLVE_ANGLE_FUDGE_DEV);
							roverControlData->state = REVOLVE;
						}
						else{
							// TODO: add case to go strait until you reach the wall and resume normal operation
							moveRover(roverControlData);
							roverControlData->state = STRAIGHT;
							requestType = REQUEST_TYPE_DISTANCE;
						}
						turnStatusReceived = 0;
					}
					else{
						requestType = REQUEST_TYPE_TURN_STATUS;
					}

					break;
				case STRAIGHT:
					// Handle if too close to wall;
					if(frontWallStatus(roverControlData, thresholdAnglePollTotal, thresholdAnglePollCount) == CLOSE_FRONT_WALL){
						findAngle(roverControlData);
						turnRover(roverControlData, roverControlData->frontSensorAngle);
						requestType = REQUEST_TYPE_TURN_STATUS;
					}
					break;
				case GOTO:
					if(turnStatusReceived != 0){
						//TODO if the turn is complete go strait
						uint8_t revs = 0;
						uint16_t ticksOffset = 0;
						distanceToEncoder(tempDist, &revs, &ticksOffset);
						moveRoverDist(roverControlData, revs, ticksOffset);
						requestType = REQUEST_TYPE_NULL;
					}
					else{
						convertGotoCoordinates(roverMap);
						stopRover(roverControlData);
						tempDist = getGotoDistance(roverMap);
						//tempDist -= (roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] + roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR])/2;
						tempDist -= 2*ROVER_LENGTH;
						turnRover(roverControlData, getGotoAngle(roverMap));
						requestType = REQUEST_TYPE_TURN_STATUS;
						roverControlData->state = GOTO;
					}

					break;
				case TRAVERSAL:
					vtLEDOff(0x3F);
					vtLEDOn(0x01);	
					//sprintf(buf, "TRAVERSAL");
					//if close to front wall move to stop state
					if(frontWallStatus(roverControlData, thresholdAnglePollTotal, thresholdAnglePollCount) == CLOSE_FRONT_WALL){
						stopRover(roverControlData);
						//requestType = REQUEST_TYPE_ENCODER;
					}
					else if(isRoverParallelToWall(roverControlData) == TOO_CLOSE_SIDE){
						fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						roverControlData->state = TOO_CLOSE;	
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_LEFT){
						difference = roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR];
						/*if(difference > (PARALLEL_THRESHOLD + 2*SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);
						}else if(difference > (PARALLEL_THRESHOLD + SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);
						}else*/ if(difference > PARALLEL_THRESHOLD){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);	
						}
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_RIGHT){
						difference = roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR];
						/*if(difference > (PARALLEL_THRESHOLD + 2*SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}else if(difference > (PARALLEL_THRESHOLD + SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}else*/ if(difference > PARALLEL_THRESHOLD){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}	
					}
					else if (frontWallStatus(roverControlData, thresholdAnglePollTotal, thresholdAnglePollCount) == ACQUIRE_FRONT_ANGLE){
						
						if(thresholdAnglePollCount < 50){
							findAngle(roverControlData);
							thresholdAnglePollTotal += roverControlData->frontSensorAngle;
							++thresholdAnglePollCount;
						}
						//printFloat("DISTANCE:",  roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR], 0);
						//printFloat("\t",  roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR], 0);
						//printFloat("\tANGLE = ",  roverControlData->frontSensorAngle, 1);
						//sprintf(buf, "ANGLE=%f\n%f\n%f", roverControlData->frontSensorAngle, roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR], roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR]);
					}
					break;
				case FIX:
					vtLEDOff(0x3F);
					vtLEDOn(0x02);
					if(frontWallStatus(roverControlData, thresholdAnglePollTotal, thresholdAnglePollCount) == CLOSE_FRONT_WALL){
						stopRover(roverControlData);
						//requestType = REQUEST_TYPE_ENCODER;
					}
					else if(isRoverParallelToWall(roverControlData) == PARALLEL){
						moveRover(roverControlData);
					}
					else if(isRoverParallelToWall(roverControlData) == TOO_CLOSE_SIDE){
						fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						roverControlData->state = TOO_CLOSE;	
					}
					/*else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_LEFT){
						difference = roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR];
						
						sprintf(buf, "FIX_FRONT_LEFT: %f", difference);
						/*if(difference > (PARALLEL_THRESHOLD + 2*SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);
						}else if(difference > (PARALLEL_THRESHOLD + SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);
						}else
						if(difference > PARALLEL_THRESHOLD){
							fixRover(roverControlData, FIX_CMD_LEFT_SLOW);	
						}
					}
					else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_RIGHT){

						difference = roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR];
						sprintf(buf, "FIX_FRONT_RIGHT: %f", difference);
						/*if(difference > (PARALLEL_THRESHOLD + 2*SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}else if(difference > (PARALLEL_THRESHOLD + SPEED_RANGE)){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}else if(difference > PARALLEL_THRESHOLD){
							fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						}	
					}*/
					/*else{
						sprintf(buf, "FIX ELSE");
					}*/
					break;
				case TOO_CLOSE:
					vtLEDOff(0x7F);
					vtLEDOn(0x04);
					//sprintf(buf, "TOO_CLOSE");
					if(frontWallStatus(roverControlData, thresholdAnglePollTotal, thresholdAnglePollCount) == CLOSE_FRONT_WALL){
						stopRover(roverControlData);
						//requestType = REQUEST_TYPE_ENCODER;
					}
					// front side is too close
					/*else if(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < TOO_CLOSE_THRESHOLD){
						fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						roverControlData->state = TOO_CLOSE;
					}*/
					if((roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]) > 0.5){
							moveRover(roverControlData);
							roverControlData->state = TOO_CLOSE_FORWARD;
					}
					break;
				case TOO_CLOSE_FORWARD:
					vtLEDOff(0x7F);
					vtLEDOn(0x08);
					//sprintf(buf, "TOO_CLOSE_FORWARD");
					if(frontWallStatus(roverControlData, thresholdAnglePollTotal, thresholdAnglePollCount) == CLOSE_FRONT_WALL){
						stopRover(roverControlData);
						//requestType = REQUEST_TYPE_ENCODER;
					}
					//if front is still close, get back to too_close state
					/*else if(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < (TOO_CLOSE_THRESHOLD - 1.5)){
						fixRover(roverControlData, FIX_CMD_RIGHT_SLOW);
						roverControlData->state = TOO_CLOSE;
					}*/
					else if(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < TOO_CLOSE_THRESHOLD + 1
					 || roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] < TOO_CLOSE_THRESHOLD + 1){
						moveRover(roverControlData);
						roverControlData->state = TOO_CLOSE_FORWARD;
					}else{
						// leave this task
						moveRover(roverControlData);
					}
				break;
				//this state is called when rover is close to a front wall
				//at this point we request encoder readings from the rover
				//control PIC after that we move to turn state
				case STOP:
					//sprintf(buf, "STOP");
					vtLEDOff(0x7F);
					vtLEDOn(0x10);
					//if(isRoverParallelToWall(roverControlData) == PARALLEL && isSensorInRange(roverControlData) == 1){
					/*if (totalExternalAngle >= 370.0){
						vtLEDOn(0x80);
					}
					else */
					if(anglePollCount < 20){
						findAngle(roverControlData);
						anglePollTotal += roverControlData->frontSensorAngle;
						// Teja does the angle array adding here for the quicksort
						if(anglePollCount < ANGLE_SAMPLE_COUNT){
							anglesSamples[anglePollCount] = (int)roverControlData->frontSensorAngle;
						}
						++anglePollCount;
					}else{
						requestType = REQUEST_TYPE_ENCODER;
					}
					// wait here till data is received
					if(encoderReceived != 0){
						//vtLEDOn(0x20);
						//vtLEDOn(0x20);
						//printFloat("PrintTest", newCorner.distFromSide, 1);
						newCorner.distFromSide = roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] + roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR];
						//printFloat("PrintAdd", newCorner.distFromSide, 1);
						newCorner.distFromSide /= 2.0;
						//printFloat("PrintDev2", newCorner.distFromSide, 1);
						// Scale for compensating from the LENGTH WIDTH change
						newCorner.distFromSide += ROVER_LENGTH_WIDTH_OFFSET;
						//printFloat("PrintPoff", newCorner.distFromSide, 1);
						//printFloat("PrintDev90", newCorner.distFromSide, 1);
						newCorner.distFromSide *= pow(newCorner.angleCornerExterior/90.0, 5);
						// newCorner.distFromSide *= sin(newCorner.angleCornerExterior*M_PI/180)*sin(newCorner.angleCornerExterior*M_PI/180); // sin^2

						//totalExternalAngle += newCorner.angleCorner;
						//newCorner.distSide += ROVER_LENGTH;// + (roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR] + roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR])/2;
						printFloat("BeforeQ:", newCorner.distSide, 1);

						// Send the reading to the map task
						//if(xQueueSend(roverMap->inQ, &newCorner,portMAX_DELAY) != pdTRUE)	VT_HANDLE_FATAL_ERROR(0);

						//vtLEDOn(0x40);
						//turn rover and keep asking for turning status
						turnRover(roverControlData, anglePollTotal/anglePollCount);

						requestType = REQUEST_TYPE_TURN_STATUS;
						encoderReceived = 0;
					}
					break;
				case TURN:	
					vtLEDOff(0x7F);
					vtLEDOn(0x20);
					//sprintf(buf, "TURN");
					// wait here till rover turn by the specified angle
					if(turnStatusReceived != 0){
						requestType = REQUEST_TYPE_ENCODER;
						turnStatusReceived = 0;
					}
					else if(encoderReceived != 0){
						double sideAngle = 0;
						sideAngleCount++;
						//wait till 5 values of side is available 
						if(sideAngleCount>=5){
							//find the adjustment to encoderAngle
							if(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]){
					    		sideAngle = atanf((roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]-roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR])/DISTANCE_BETWEEN_SIDE_SHORT) * 180/M_PI;
							}
							else{
								sideAngle = -1*atanf((roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR]-roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR])/DISTANCE_BETWEEN_SIDE_SHORT) * 180/M_PI;
							}
							newCorner.tempSide = sideAngle;
							newCorner.tempEncoder = newCorner.angleCornerExterior;
							//newCorner.angleCornerExterior = (newCorner.angleCornerExterior + encoderAngle)/2.0;
							newCorner.angleCornerExterior += sideAngle;
							if(xQueueSend(roverMap->inQ, &newCorner,portMAX_DELAY) != pdTRUE)	VT_HANDLE_FATAL_ERROR(0);	
							moveRover(roverControlData);
							sideAngleCount = 0;
							encoderReceived = 0;
						}
					}
					break;
				case NULL_STATE:
					break;
			}
		}
		//received encoder data
		else if (receivedMsg.message_type == PUB_MSG_T_ENCODER_DATA && roverControlData->state == STOP){
			// Variables to hold the distance readings for each encoder
			double distSide1, distSide2, frontDist;

			encoderReceived = 1;
			// Calculate the distance for the first encoder
			distSide1 = (receivedMsg.data[0] + (receivedMsg.data[1] << 8))/TICKS_PER_REVOLUTION;
			distSide1 += receivedMsg.data[2];
			//distSide1 = distSide1 * WHEEL_CIRCUMFERENCE;
			//distSide1 = distSide1 + ROVER_LENGTH + roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR];

			// Calculate the distance for the second encoder			
			distSide2 = (receivedMsg.data[3] + (receivedMsg.data[4] << 8))/TICKS_PER_REVOLUTION;
			distSide2 += receivedMsg.data[5];
			//distSide2 = distSide2 * WHEEL_CIRCUMFERENCE;
			//distSide2 = distSide2 + ROVER_LENGTH + FRONT_STOP_DISTANCE*1.0;

			// Average the two readings
			newCorner.distSide = (distSide1 + distSide2)/2.0;
			

			// Teja added the angle stuff to the turn
			roverControlData->frontSensorAngle = anglePollTotal/anglePollCount; // This gets overwritten bellow with the qsort value
			
			// New quicksort implementation added here:
			qsort(anglesSamples, anglePollCount, sizeof(int), cmpfunc);

			roverControlData->frontSensorAngle = 0.0;
			uint8_t i = anglePollCount;
			for((i = anglePollCount/2 - 2); i <= (anglePollCount/2 + 2); ++i){
				roverControlData->frontSensorAngle += anglesSamples[i];
			}
			roverControlData->frontSensorAngle /= 5.0;
			// sprintf(buf, "%d  %d  %d    %d  %d  %d \nSide: %f \nAngle: %f", receivedMsg.data[0], receivedMsg.data[1], receivedMsg.data[2], receivedMsg.data[3], receivedMsg.data[4], receivedMsg.data[5], newCorner.distSide, roverControlData->frontSensorAngle);
			 //roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR]);
			//printFloat("\t",  roverControlData->frontSensorAngle, 1);
			anglePollTotal = 0.0;
			anglePollCount = 0;
			thresholdAnglePollTotal = 0.0;
			thresholdAnglePollCount = 0;

			//newCorner.angleCornerExterior = roverControlData->frontSensorAngle;
			newCorner.tempFront = roverControlData->frontSensorAngle;
			// TODO: Uncomment this block if you want to add the wheel circumference and the rover length
			newCorner.distSide = newCorner.distSide*WHEEL_CIRCUMFERENCE;

			printFloat("Average Encoders:", newCorner.distSide, 1);

			newCorner.distSide += ROVER_LENGTH;

			printFloat("With RoverLength:", newCorner.distSide, 1);

			// This is where we will set a calculated value for the angle if it is a regular shape with known number of sides
			if(roverMap->taskFlags == REGULAR && roverMap->numberSides != 0){
				newCorner.angleCornerExterior = 360.0/roverMap->numberSides;
			}

			// Adds the front distance
			//if(newCorner.angleCornerExterior > 59.0){
				frontDist = (roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR])/tan(newCorner.angleCornerExterior*M_PI/180.0);
				frontDist = roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR] - frontDist;
				printFloat("Front Dist:", frontDist, 1);
			if(frontDist > 0.0)
				newCorner.distSide += frontDist;
			//}
			//set request type to sensor distance 
			requestType = REQUEST_TYPE_DISTANCE;
			//getEncoderDistance(receivedMsg.data[2], );
		}
		//encoder recieved with angle
		else if (receivedMsg.message_type == PUB_MSG_T_ENCODER_DATA && roverControlData->state == TURN){
			double encoderAngle, arc, arc1, arc2;
			encoderReceived = 1;
			arc1 = (receivedMsg.data[0] + (receivedMsg.data[1] << 8))/TICKS_PER_REVOLUTION;
			arc1 += receivedMsg.data[2];		
			arc2 = (receivedMsg.data[3] + (receivedMsg.data[4] << 8))/TICKS_PER_REVOLUTION;
			arc2 += receivedMsg.data[5];

			// Average the two angles
			arc = (arc1 + arc2)*WHEEL_CIRCUMFERENCE/2.0;
			// convert arc distance to angle
			// angle = s/r
			encoderAngle = (arc/ROVER_RADIUS) * 180 / M_PI;

			//newCorner.angleCornerExterior = (newCorner.angleCornerExterior + encoderAngle)/2.0;
			newCorner.angleCornerExterior = encoderAngle;
			requestType = REQUEST_TYPE_DISTANCE;
		}
		//received turn status
		else if (receivedMsg.message_type ==  PUB_MSG_T_TURN_STATUS){
			if(roverControlData->state == REVOLVE)
				requestType = REQUEST_TYPE_DISTANCE;
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

int cmpfunc (const void * a, const void * b)
{
   return ( *(int*)a - *(int*)b );
}