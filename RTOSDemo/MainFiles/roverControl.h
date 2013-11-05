#ifndef ROVERCONTROL_H
#define ROVERCONTROL_H

#include "roverHeader.h"

void startRoverControlTask(RoverControlStruct *roverControlData, unsigned portBASE_TYPE uxPriority, UARTstruct *uart);

#endif