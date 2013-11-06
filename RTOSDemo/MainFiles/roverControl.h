#ifndef ROVERCONTROL_H
#define ROVERCONTROL_H

#include "roverHeader.h"

#define baseStack 2
#if PRINTF_VERSION == 1
#define roverSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define roverSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

//static portTASK_FUNCTION_PROTO( roverControlTask, param );
void roverControlTask( void *param );

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart);

#endif