#ifndef CONDUCTOR_H
#define CONDUCTOR_H
#include "vtI2C.h"
#include "uartDriver.h"
// Structure used to pass parameters to the task
typedef struct __ConductorStruct {
	UARTstruct *uartDevice;
} vtConductorStruct;

// Public API
//
// The job of this task is to read from the message queue that is output by the UART thread and to distribute the messages to the right
//   threads.

// Start the task
// Args:
//   conductorData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   uart: pointer to the data structure for a UART task
void vStartConductorTask(vtConductorStruct *conductorData,unsigned portBASE_TYPE uxPriority, UARTstruct *uart);

#endif //ifndef CONDUCTOR_H