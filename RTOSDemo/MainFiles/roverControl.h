#ifndef ROVERCONTROL_H
#define ROVERCONTROL_H

#include "uartDriver.h"

typedef struct __RoverControlStruct {
	UARTstruct *uartDevice;
	xQueueHandle inQ;
} RoverControlStruct;

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart);

#endif