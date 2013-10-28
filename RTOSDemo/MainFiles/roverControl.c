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

// defined functions for sensors calculations
void readNewMsg(RoverControlStruct *roverControlData, public_message_t *receivedMsg);
void convertToDistance(RoverControlStruct *roverControlData);
void checkSensorsRange(RoverControlStruct *roverControlData);
void findAngles(RoverControlStruct *roverControlData);

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

		// 2. Request new sensor data.
		// Update the count and send the request
		sensorRequestMsg.msgID = public_message_get_count(PUB_MSG_T_SENS_DIST);
		uartEnQ(roverControlData->uartDevice, sensorRequestMsg.msgType, sensorRequestMsg.msgID, sensorRequestMsg.txLen,
			sensorRequestMsg.data);

		// 3. Process message received in step 1, contained in receivedMsg.
		// See public_messages.h for the structure of receivedMsg (it is type "public_message_t").

		//read a new message
		readNewMsg(roverControlData, &receivedMsg);
		convertToDistance(roverControlData);
		checkSensorsRange(roverControlData);
		findAngles(roverControlData);
	}
}

void readNewMsg(RoverControlStruct *roverControlData, public_message_t *receivedMsg){
	//assign received values
	roverControlData->leftShortSensor = receivedMsg->data[1] << 8 | receivedMsg->data[0];
	roverControlData->rightShortSensor = receivedMsg->data[3] << 8 | receivedMsg->data[2];
	roverControlData->leftMediumSensor = receivedMsg->data[5] << 8 | receivedMsg->data[4];
	roverControlData->rightMediumSensor = receivedMsg->data[7] << 8 | receivedMsg->data[6];
	roverControlData->leftLongSensor = receivedMsg->data[9] << 8 | receivedMsg->data[8];
	roverControlData->rightLongSensor = receivedMsg->data[11] << 8 | receivedMsg->data[10];
}

void convertToDistance(RoverControlStruct *roverControlData){
	roverControlData->leftShortSensor = 12317*pow(roverControlData->leftShortSensor,-1.337);
    roverControlData->rightShortSensor = 12317*pow(roverControlData->rightShortSensor,-1.337);
    //TODO: convert other sensors
}

void checkSensorsRange(RoverControlStruct *roverControlData){
	//TODO: find if sensors are in range
}

void findAngles(RoverControlStruct *roverControlData){
	//TODO: find if sensors are in range
	if(roverControlData->leftShortSensor > roverControlData->rightShortSensor){
    	roverControlData->shortSensorAngle = atanf(DISTANCE_BETWEEN_IR/(roverControlData->leftShortSensor-roverControlData->rightShortSensor)) * 180/M_PI;
	}else{
	    roverControlData->shortSensorAngle = atan((roverControlData->rightShortSensor-roverControlData->leftShortSensor)/DISTANCE_BETWEEN_IR) * 180/M_PI;
	}
}