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
	UARTmsg message;

	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from UART
		if (uartDeQ(param->uartDevice, &message) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Convert the UART driver struct to a public message struct
#warning "Need to convert message to public message type"

		// Decide where to send the message based on the message type
		switch(message.msgType) {
		default: {
			VT_HANDLE_FATAL_ERROR(message.msgType);
			break;
		}
		}


	}
}

