#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_

#include "lpc17xx_uart.h"
#include <stdlib.h>
#include <stdio.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "vtUtilities.h"

#include "lpc17xx_libcfg_default.h"
#include "lpc17xx_pinsel.h"


#define UART_INIT_ERR -1
#define UART_INIT_SUCCESS 0

typedef struct __UARTStruct {
	uint8_t devNum;	  						// Number of the I2C peripheral (0,1,2 on the 1768)
	LPC_UART_TypeDef *devAddr;	 			// Memory address of the I2C peripheral
	unsigned portBASE_TYPE taskPriority;   	// Priority of the I2C task
	xSemaphoreHandle binSemaphore;		   	// Semaphore used between I2C task and I2C interrupt handler
	xQueueHandle inQ;					   	// Queue used to send messages from other tasks to the I2C task
	xQueueHandle outQ;						// Queue used by the I2C task to send out results
} UARTStruct;


// Args:
//   dev: pointer to the vtI2CStruct data structure
//   i2cDevNum: The number of the i2c device -- 0, 1, or 2
//   taskPriority: At what priority should this task be run?
//   i2cSpeed: Clock speed of the i2c bus
// Return:
//   if successful, returns vtI2CInitSuccess
//   if not, should return vtI2CErrInit
// Must be called for each I2C device initialized (0, 1, or 2) and used
int initUART(UARTStruct *devPtr, uint8_t uartDevNum, unsigned portBASE_TYPE taskPriority, UART_CFG_Type* uartConfig, UART_FIFO_CFG_Type* uartFIFOConfig);

#endif /* UARTDRIVER_H_ */