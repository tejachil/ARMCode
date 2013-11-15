#ifndef ROVERCONTROL_H_
#define ROVERCONTROL_H_

#include "roverHeader.h"
#include "roverMap.h"

#define baseStack 2

#if PRINTF_VERSION == 1
#define roverSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define roverSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

typedef struct __RoverDataStruct {
	RoverMapStruct* map;
	RoverControlStruct* control;
	UARTstruct* uart;
} RoverDataStruct;

//static portTASK_FUNCTION_PROTO( roverControlTask, param );
void roverControlTask( void *param );

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart, RoverMapStruct *roverMapStruct, xTaskHandle taskHandle);

#endif