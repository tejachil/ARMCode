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
#include "conductor.h"
#include "public_messages.h"
#include "uartDriver.h"

/* *********************************************** */
// definitions and data structures that are private to this file

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define INSPECT_STACK 1
#define baseStack 2
#if PRINTF_VERSION == 1
#define conSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define conSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif
// end of defs
/* *********************************************** */

/* The i2cTemp task. */
static portTASK_FUNCTION_PROTO( vConductorUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartConductorTask(vtConductorStruct *conductorData,unsigned portBASE_TYPE uxPriority, UARTstruct *uart)
{
	/* Start the task */
	portBASE_TYPE retval;
	conductorData->uartDevice = uart;
	if ((retval = xTaskCreate( vConductorUpdateTask, ( signed char * ) "UART Conductor", conSTACK_SIZE, (void *) conductorData, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

// End of Public API
/*-----------------------------------------------------------*/

// This is the actual task that is run
static portTASK_FUNCTION( vConductorUpdateTask, pvParameters )
{
	// Get the parameters
	vtConductorStruct *param = (vtConductorStruct *) pvParameters;
	// Latest received message
	public_message_t message;

	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from UART
#warning "Still using the old UARTMsg type"
		UARTmsg driver_message;
		if (uartDeQ(param->uartDevice, &driver_message) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Convert the UART driver struct to the public message struct
		message.message_type = (public_message_type_t) driver_message.msgType;
		message.message_count = driver_message.msgID;
		message.data_length = driver_message.rxLen; // rxLen is actually the data length, I think
		if (message.data_length <= PUB_MSG_MAX_DATA_SIZE) {
			memcpy(message.data, driver_message.data, message.data_length);
		} else {
			VT_HANDLE_FATAL_ERROR(message.data_length);
		}

		// Decide where to send the message based on the message type
		switch(message.message_type) {
		default: {
			// Unknown message type
			VT_HANDLE_FATAL_ERROR(message.message_type);
			break;
		}
		}


	}
}

