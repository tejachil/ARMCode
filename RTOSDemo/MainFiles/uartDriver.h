#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_

#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"
#include "vtUtilities.h"

#define UART_INIT_ERR 			-1
#define UART_INIT_SUCCESS 		0
#define UARTQ_LEN				10
#define UART_INT_PRIORITY		7
#define UART_MSG_MIN_SIZE		3
#define UART_MSG_MAX_SIZE		15
#define UART_MSG_MAX_DATA_SIZE	(UART_MSG_MAX_SIZE - UART_MSG_MIN_SIZE)
#define UART_STACK_SIZE			(7*configMINIMAL_STACK_SIZE)


typedef struct __UARTstruct {
	uint8_t devNum;	  						// Number of the I2C peripheral (0,1,2 on the 1768)
	LPC_UART_TypeDef *devAddr;	 			// Memory address of the I2C peripheral
	unsigned portBASE_TYPE taskPriority;   	// Priority of the I2C task
	xQueueHandle inQ;					   	// Queue used to send messages from other tasks to the I2C task
	xQueueHandle outQ;						// Queue used by the I2C task to send out results
} UARTstruct;

typedef struct __UARTmsg {
	uint8_t msgType; // A field you will likely use in your communications between processors (and for debugging)
	uint8_t msgID;	 // message ID
	uint8_t txLen;   // Length of the message you want to sent (or, on the way back, the length that *was* sent)
	uint8_t	rxLen;	 // Length of the message you *expect* to receive (or, on the way back, the length that *was* received)
	uint8_t status;  // status of the completed operation -- I've not done anything much here, you probably should...
	uint8_t data[UART_MSG_MAX_DATA_SIZE]; // On the way in, message to be sent, on the way out, message received (if any)
} UARTmsg;

// Args:
//   dev: pointer to the UARTstruct data structure
//   i2cDevNum: The number of the uart device -- 0, 1, or 2
//   taskPriority: At what priority should this task be run?
//   i2cSpeed: Clock speed of the i2c bus
// Return:
//   if successful, returns vtI2CInitSuccess
//   if not, should return vtI2CErrInit
// Must be called for each I2C device initialized (0, 1, or 2) and used
int initUART(UARTstruct *devPtr, uint8_t uartDevNum, unsigned portBASE_TYPE taskPriority, uint32_t baud, 
	UART_PARITY_Type parity, UART_DATABIT_Type dataBits, UART_STOPBIT_Type stopBits);

// A simple routine to use for filling out and sending a message to the UART thread
//   You may want to make your own versions of these as they are not suited to all purposes
// Args
//   dev: pointer to the UARTstruct data structure
//   msgType: The message type value -- does not get sent on the wire, but is included in the response in the message queue
//	 msgID: counting ID to keep track of message
//   txLen: The number of bytes you want to send
//   txBuf: The buffer holding the bytes you want to send
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE uartEnQ(UARTstruct *dev, uint8_t msgType, uint8_t msgID, uint8_t txLen, uint8_t *txBuf);

// A simple routine to use for retrieving a message from the UART thread
// Args
//   dev: pointer to the UARTstruct data structure
//   message: populates the referance of the UARTmsg
// Return:
//   Result of the call to xQueueReceive()
portBASE_TYPE uartDeQ(UARTstruct *dev, UARTmsg *message);

#endif /* UARTDRIVER_H_ */