#include "roverControl.h"
#include "public_messages.h"
#include "uartDriver.h"

#include <math.h>

#define roverControlQLen (10)

#define baseStack 2
#if PRINTF_VERSION == 1
#define roverSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define roverSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

static portTASK_FUNCTION_PROTO( roverControlTask, param );

// functions for sensors calculations
void readNewMsg(RoverControlStruct *roverControlData, public_message_t *receivedMsg);
void averageValues(RoverControlStruct *roverControlData);
void convertToDistance(RoverControlStruct *roverControlData);
void checkSensorsRange(RoverControlStruct *roverControlData);
void findAngles(RoverControlStruct *roverControlData);
int isRoverParallelToWall(RoverControlStruct *roverControlData);
// functinos for controlling rover
void moveRover(RoverControlStruct *roverControlData);
void stopRover(RoverControlStruct *roverControlData);
//helper functions
void printFloat(char* buf, float number, int newLine);

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart) {
	roverControlData->uartDevice = uart;

	// Create the queue that will be used to talk to this task
	if ((roverControlData->inQ = xQueueCreate(roverControlQLen,sizeof(public_message_t))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	if (xTaskCreate( roverControlTask, ( signed char * ) "Rover Control", roverSTACK_SIZE, (void *) roverControlData, uxPriority, ( xTaskHandle * ) NULL ) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
}

static portTASK_FUNCTION( roverControlTask, param ) {
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

		//read a new message
		readNewMsg(roverControlData, &receivedMsg);
		averageValues(roverControlData);
		convertToDistance(roverControlData);
		checkSensorsRange(roverControlData);
		findAngles(roverControlData);
		printFloat("Left:", roverControlData->sensorDistance[LEFT_SHORT_SENSOR], 0);
		printFloat("Right:", roverControlData->sensorDistance[RIGHT_SHORT_SENSOR], 0);
		printFloat("LeftMedium:", roverControlData->sensorDistance[LEFT_MEDIUM_SENSOR], 0);
		printFloat("RightMedium:", roverControlData->sensorDistance[RIGHT_MEDIUM_SENSOR], 0);
		printFloat("Angle:", roverControlData->shortSensorAngle, 1);

		switch(roverControlData->state){
			case INIT:
				//send command to move rover
				moveRover(roverControlData);
				roverControlData->state = TRAVERSAL;
				break;
			case TRAVERSAL:
				if(isRoverParallelToWall(roverControlData) == 0){
					//send command to stop
					printf("move to stop # ");
					stopRover(roverControlData);
					roverControlData->state = STOP;
				}
				break;
			case STOP:
				if(isRoverParallelToWall(roverControlData) == 1){
					//send command to move
					printf("stop to move # ");
					moveRover(roverControlData);
					roverControlData->state = TRAVERSAL;
				}
				break;
		}
	}
}

void readNewMsg(RoverControlStruct *roverControlData, public_message_t *receivedMsg){
	int i=0;
	//store each sesnor a sampling array for averaging later on
	for(i=0; i<NUMBER_OF_SENSORS;i++)
		roverControlData->sensorDistanceSamples[i][roverControlData->samplingCounter] = receivedMsg->data[2*i+1] << 8 | receivedMsg->data[2*i];
	// increment for the next sample
	roverControlData->samplingCounter = (roverControlData->samplingCounter+1)%NUMBER_OF_SAMPLES; 
}


void averageValues(RoverControlStruct *roverControlData){
	int sensorCounter=0, sampleCounter=0;
	//init sensor distance before averaging
	for(sensorCounter=0; sensorCounter<NUMBER_OF_SENSORS; sensorCounter++)
		roverControlData->sensorDistance[sensorCounter]=0;

	//average all samples for every sensor
	for(sensorCounter=0; sensorCounter<NUMBER_OF_SENSORS; sensorCounter++)
		for(sampleCounter=0; sampleCounter<NUMBER_OF_SAMPLES; sampleCounter++)
			roverControlData->sensorDistance[sensorCounter] += roverControlData->sensorDistanceSamples[sensorCounter][sampleCounter]/NUMBER_OF_SAMPLES;
}

void convertToDistance(RoverControlStruct *roverControlData){
	roverControlData->sensorDistance[LEFT_SHORT_SENSOR] = 12317*pow(roverControlData->sensorDistance[LEFT_SHORT_SENSOR],-1.337);
    roverControlData->sensorDistance[RIGHT_SHORT_SENSOR] = 12317*pow(roverControlData->sensorDistance[RIGHT_SHORT_SENSOR],-1.337);
    roverControlData->sensorDistance[LEFT_MEDIUM_SENSOR] = 5864*pow(roverControlData->sensorDistance[LEFT_MEDIUM_SENSOR],-1.099);
    roverControlData->sensorDistance[RIGHT_MEDIUM_SENSOR] = 5864*pow(roverControlData->sensorDistance[RIGHT_MEDIUM_SENSOR],-1.099);
    
    //TODO: convert other sensors
}

void checkSensorsRange(RoverControlStruct *roverControlData){
	//TODO: find if sensors are in range
}

void findAngles(RoverControlStruct *roverControlData){
	//TODO: find angle between sensors
	if(roverControlData->sensorDistance[LEFT_SHORT_SENSOR] > roverControlData->sensorDistance[RIGHT_SHORT_SENSOR]){
    	roverControlData->shortSensorAngle = atanf(DISTANCE_BETWEEN_IR/(roverControlData->sensorDistance[LEFT_SHORT_SENSOR]-roverControlData->sensorDistance[RIGHT_SHORT_SENSOR])) * 180/M_PI;
	}else{
	    roverControlData->shortSensorAngle = atan((roverControlData->sensorDistance[RIGHT_SHORT_SENSOR]-roverControlData->sensorDistance[LEFT_SHORT_SENSOR])/DISTANCE_BETWEEN_IR) * 180/M_PI;
	}
}

int isRoverParallelToWall(RoverControlStruct *roverControlData){
	//float difference = roverControlData->sensorDistance[LEFT_SHORT_SENSOR] - roverControlData->sensorDistance[RIGHT_SHORT_SENSOR];
	if(roverControlData->sensorDistance[LEFT_SHORT_SENSOR] > (roverControlData->sensorDistance[RIGHT_SHORT_SENSOR]+1)
	|| roverControlData->sensorDistance[RIGHT_SHORT_SENSOR] > (roverControlData->sensorDistance[LEFT_SHORT_SENSOR]+1)){
		//not parallel
		vtLEDOff(0x04);
		return 0;
	}
	else{
		//parallel 
		vtLEDOn(0x04);
		return 1;
	}
}

void printFloat(char* buf, float number, int newLine){
	int intPart = (int)number;
	int decimalPart = (number - (int)number)*1000;
	printf("%s %d.%d, ",buf, intPart,decimalPart);
	if(newLine)
		printf("\n");
}

void moveRover(RoverControlStruct *roverControlData){
	UARTmsg roverRequestMsg;
	portBASE_TYPE result;
	roverRequestMsg.msgType = PUB_MSG_T_MOV_CMD;
	roverRequestMsg.msgID = 0; // The count will be updated before sending each request
	roverRequestMsg.txLen = public_message_data_size[PUB_MSG_T_MOV_CMD];
	roverRequestMsg.data[0] = 0x55; // move
	roverRequestMsg.data[1] = 0x00;

	roverRequestMsg.msgID = public_message_get_count(PUB_MSG_T_MOV_CMD);
	result = uartEnQ(roverControlData->uartDevice, roverRequestMsg.msgType, roverRequestMsg.msgID, roverRequestMsg.txLen,
		roverRequestMsg.data);

	if(result == pdTRUE){
		printf(" success\n");
	}
	else if (result == pdFALSE){
		printf(" failure\n");
	}
}

void stopRover(RoverControlStruct *roverControlData){
	UARTmsg roverRequestMsg;
	portBASE_TYPE result;
	roverRequestMsg.msgType = PUB_MSG_T_MOV_CMD;
	roverRequestMsg.msgID = 0; // The count will be updated before sending each request
	roverRequestMsg.txLen = public_message_data_size[PUB_MSG_T_MOV_CMD];
	roverRequestMsg.data[0] = 0xAA; // stop
	roverRequestMsg.data[1] = 0x00;

	roverRequestMsg.msgID = public_message_get_count(PUB_MSG_T_MOV_CMD);
	result = uartEnQ(roverControlData->uartDevice, roverRequestMsg.msgType, roverRequestMsg.msgID, roverRequestMsg.txLen,
		roverRequestMsg.data);

	if(result == pdTRUE){
		printf(" success\n");
	}
	else if (result == pdFALSE){
		printf(" failure\n");
	}
}