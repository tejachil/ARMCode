#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "vtUtilities.h"
#include "vtI2C.h"
#include "LCDtask.h"
#include "i2cTemp.h"
#include "I2CTaskMsgTypes.h"

#define I2C_ADDR 0x4F
#define I2C_ADC_LOW 0x61
#define I2C_ADC_HIGH 0x62

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtTempQLen 10 
// actual data structure that is sent in a message
typedef struct __vtTempMsg {
	uint8_t msgType;
	uint8_t	length;	 // Length of the message to be printed
	uint8_t buf[vtTempMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} vtTempMsg;

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define baseStack 3
#if PRINTF_VERSION == 1
#define i2cSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define i2cSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

// end of defs
/* *********************************************** */

/* The i2cTemp task. */
static portTASK_FUNCTION_PROTO( vi2cTempUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStarti2cTempTask(vtTempStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtTempQLen,sizeof(vtTempMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	if ((retval = xTaskCreate( vi2cTempUpdateTask, ( signed char * ) "i2cTemp", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendTempTimerMsg(vtTempStruct *tempData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (tempData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtTempMsg tempBuffer;
	tempBuffer.length = sizeof(ticksElapsed);
	if (tempBuffer.length > vtTempMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(tempBuffer.length);
	}
	memcpy(tempBuffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
	tempBuffer.msgType = TempMsgTypeTimer;
	return(xQueueSend(tempData->inQ,(void *) (&tempBuffer),ticksToBlock));
}

portBASE_TYPE SendTempValueMsg(vtTempStruct *tempData,uint8_t msgType,uint8_t value,portTickType ticksToBlock)
{
	vtTempMsg tempBuffer;

	if (tempData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	tempBuffer.length = sizeof(value);
	if (tempBuffer.length > vtTempMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(tempBuffer.length);
	}
	memcpy(tempBuffer.buf,(char *)&value,sizeof(value));
	tempBuffer.msgType = msgType;
	return(xQueueSend(tempData->inQ,(void *) (&tempBuffer),ticksToBlock));
}

// End of Public API
/*-----------------------------------------------------------*/
int getMsgType(vtTempMsg *Buffer)
{
	return(Buffer->msgType);
}
uint8_t getValue(vtTempMsg *Buffer)
{
	uint8_t *ptr = (uint8_t *) Buffer->buf;
	return(*ptr);
}

// I2C commands for the temperature sensor
	const uint8_t i2cAdcReadLow[]= {I2C_ADC_LOW};
	const uint8_t i2cAdcReadHigh[]= {I2C_ADC_HIGH};
// end of I2C command definitions

// Definitions of the states for the FSM below
const uint8_t fsmStateAdcReadLow = 0;
const uint8_t fsmStateAdcReadHigh = 1;
// This is the actual task that is run
static portTASK_FUNCTION( vi2cTempUpdateTask, pvParameters )
{
	int adc = 0;
	// Get the parameters
	vtTempStruct *param = (vtTempStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;
	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];
	// Buffer for receiving messages
	vtTempMsg msgBuffer;
	uint8_t currentState;

	// Assumes that the I2C device (and thread) have already been initialized

	// This task is implemented as a Finite State Machine.  The incoming messages are examined to see
	//   whether or not the state should change.

	currentState = fsmStateAdcReadLow;
	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Now, based on the type of the message and the state, we decide on the new state and action to take
		switch(getMsgType(&msgBuffer)) {
		case TempMsgTypeTimer: {
			// Timer messages never change the state, they just cause an action (or not) 
				// Read in the values from the temperature sensor
				// We have three transactions on i2c to read the full temperature 
				//   we send all three requests to the I2C thread (via a Queue) -- responses come back through the conductor thread
				// Temperature read -- use a convenient routine defined above
				if (vtI2CEnQ(devPtr,vtI2CMsgTypeTempRead1,I2C_ADDR,sizeof(i2cAdcReadLow),i2cAdcReadLow,1) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				// Read in the read counter
				if (vtI2CEnQ(devPtr,vtI2CMsgTypeTempRead2,I2C_ADDR,sizeof(i2cAdcReadHigh),i2cAdcReadHigh,1) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			break;
		}
		case vtI2CMsgTypeTempRead1: {
			if (currentState == fsmStateAdcReadLow) {
				currentState = fsmStateAdcReadHigh;
				adc = getValue(&msgBuffer);
			} else {
				// unexpectedly received this message
				VT_HANDLE_FATAL_ERROR(0);
			}
			break;
		}
		case vtI2CMsgTypeTempRead2: {
			if (currentState == fsmStateAdcReadHigh) {
				currentState = fsmStateAdcReadLow;
				adc |= getValue(&msgBuffer) << 8;
				printf("ADC = %d\n",adc);
				if (lcdData != NULL) {
					if (SendLCDADCValue(lcdData, adc,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
				}
			} else {
				// unexpectedly received this message
				VT_HANDLE_FATAL_ERROR(0);
			}
			break;
		}
		default: {
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		}


	}
}

