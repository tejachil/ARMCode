#include <stdlib.h>
#include <stdio.h>
#include "LCDtask.c"
#include "uartDriver.h"
#include "lpc17xx_pinsel.h"

#define UARTQ_LEN 10
#define UART_INT_PRIORITY 7

static 	UARTStruct *devStaticPtr[4];

int initUART(UARTStruct *devPtr, uint8_t uartDevNum, unsigned portBASE_TYPE taskPriority, UART_CFG_Type* uartConfig, UART_FIFO_CFG_Type* uartFIFOConfig){
	devPtr->devNum = uartDevNum;
	devPtr->taskPriority = taskPriority;
	UART_ConfigStructInit(uartConfig);
	UART_FIFOConfigStructInit(uartFIFOConfig);

	PINSEL_CFG_Type pinConfig;
	switch(uartDevNum){
		case 1:
			NVIC_SetPriority(UART1_IRQn, UART_INT_PRIORITY);	
			NVIC_DisableIRQ(UART1_IRQn);
			
			devPtr->devAddr = LPC_UART1;
			devStaticPtr[1] = devPtr;
			
			pinConfig.OpenDrain = 0;
			pinConfig.Pinmode = 0;
			pinConfig.Funcnum = 1;
			pinConfig.Pinnum = 15;	// UART1 TX Pin
			pinConfig.Portnum = 0;
			PINSEL_ConfigPin(&pinConfig);
			pinConfig.Pinnum = 16;
			PINSEL_ConfigPin(&pinConfig);
			
			break;
		default:
			return(UART_INIT_ERR);
			break;			
	}
	
	// Create semaphore to communicate with interrupt handler
	/*vSemaphoreCreateBinary(devPtr->binSemaphore);
	if (devPtr->binSemaphore == NULL)		return(UART_INIT_ERR);
	
	// Need to do an initial "take" on the semaphore to ensure that it is initially blocked
	if (xSemaphoreTake(devPtr->binSemaphore,0) != pdTRUE) {
		// free up everyone and go home
		vQueueDelete(devPtr->binSemaphore);
		return(UART_INIT_ERR);
	}*/

	// Allocate the two queues to be used to communicate with other tasks
	if ((devPtr->inQ = xQueueCreate(UARTQ_LEN,sizeof(uint8_t))) == NULL) {
		// free up everyone and go home
		vQueueDelete(devPtr->binSemaphore);
		return(UART_INIT_ERR);
	}
	if ((devPtr->outQ = xQueueCreate(UARTQ_LEN,sizeof(uint8_t))) == NULL) {
		// free up everyone and go home
		vQueueDelete(devPtr->binSemaphore);
		vQueueDelete(devPtr->outQ);
		return(UART_INIT_ERR);
	}
		
	// Initialize  UART peripheral
	UART_Init(devPtr->devAddr, uartConfig);
	
	//Setup FIFO
	UART_FIFOConfig(devPtr->devAddr, uartFIFOConfig);

	/* Enable UART Rx interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART1, UART_INTCFG_RBR, ENABLE);
	/* Enable UART line status interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART1, UART_INTCFG_RLS, ENABLE);

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x01));
	/* Enable Interrupt for UART0 channel */
	NVIC_EnableIRQ(UART1_IRQn);
	
	// Enable  UART operation
	UART_TxCmd(devPtr->devAddr, ENABLE);
	
	return UART_INIT_SUCCESS;	
}

// i2c interrupt handler
static __INLINE void UARTIsr(LPC_UART_TypeDef *devAddr) {
	/*char lcdBuffer[50];
	sprintf(lcdBuffer,"hi: %d",data);
	if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
		VT_HANDLE_FATAL_ERROR(0);
	}*/
	vtLEDOn(0x20);
	if (UART_GetIntId(LPC_UART0) == UART_IIR_INTID_RLS){
		vtLEDOn(0x10);
	}
	
/*	if (I2C_MasterTransferComplete(devAddr)) {
		static signed portBASE_TYPE xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(*binSemaphore,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}*/
}

// Simply pass on the information to the real interrupt handler above (have to do this to work for multiple i2c peripheral units on the LPC1768
void UART1Isr(void) {
	// Log the I2C status code
	//vtITMu8(vtITMPortI2C1IntHandler,((devStaticPtr[1]->devAddr)->I2STAT & I2C_STAT_CODE_BITMASK));
	UARTIsr(devStaticPtr[1]->devAddr);
}