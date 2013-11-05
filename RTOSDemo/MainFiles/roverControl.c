#include "roverControl.h"
#include "sensorFunctions.h"
#include "roverFunctions.h"
#include <math.h>

#define roverControlQLen (10)

#define baseStack 2
#if PRINTF_VERSION == 1
#define roverSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define roverSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

static portTASK_FUNCTION_PROTO( roverControlTask, param );

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
				roverControlData->state = TRAVERSAL;
				break;
			case TRAVERSAL:
				if(isRoverParallelToWall(roverControlData) == FIX_FRONT_LEFT){
					//fix to Left
					fixRover(roverControlData, FIX_FRONT_LEFT);
					roverControlData->state = FIX;
				}
				else if(isRoverParallelToWall(roverControlData) == FIX_FRONT_RIGHT){
					//fix to Right
					fixRover(roverControlData, FIX_FRONT_RIGHT);
					roverControlData->state = FIX;
				}
				break;
			case FIX:
				if(isRoverParallelToWall(roverControlData) == PARALLEL){
					//send command to move
					moveRover(roverControlData);
					roverControlData->state = TRAVERSAL;
				}
			/*case STOP:
				if(isRoverParallelToWall(roverControlData) == PARALLEL && isSensorInRange(roverControlData) == 1){
					//send command to move
					moveRover(roverControlData);
					roverControlData->state = TRAVERSAL;
				}
				break;*/
		}
	}
}
