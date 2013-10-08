#include <stdlib.h>
#include <stdio.h>
#include "LCDtask.c"
#include "uartDriver.h"
#include "lpc17xx_pinsel.h"


static 	UARTStruct *devStaticPtr[4];

static portTASK_FUNCTION_PROTO( uartMonitorTask, pvParameters );

void UART1_IRQHandler(void);

int initUART(UARTStruct *devPtr, uint8_t uartDevNum, unsigned portBASE_TYPE taskPriority, uint32_t baud, 
	UART_PARITY_Type parity, UART_DATABIT_Type dataBits, UART_STOPBIT_Type stopBits) {
	
	UART_CFG_Type uartConfig;
	UART_FIFO_CFG_Type uartFIFOConfig;
		
	devPtr->devNum = uartDevNum;
	devPtr->taskPriority = taskPriority;
	
	PINSEL_CFG_Type pinConfig;
	IRQn_Type uartIRQn;
	
	uint8_t retval = UART_INIT_SUCCESS;
	switch(uartDevNum){
		case 1:
			NVIC_SetPriority(UART1_IRQn, UART_INT_PRIORITY);	
			uartIRQn = UART1_IRQn;
			NVIC_DisableIRQ(uartIRQn);
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
	
	/*// Create semaphore to communicate with interrupt handler
	vSemaphoreCreateBinary(devPtr->binSemaphore);
	if (devPtr->binSemaphore == NULL)		return(UART_INIT_ERR);
	
	// Need to do an initial "take" on the semaphore to ensure that it is initially blocked
	if (xSemaphoreTake(devPtr->binSemaphore,0) != pdTRUE) {
		// free up everyone and go home
		vQueueDelete(devPtr->binSemaphore);
		return(UART_INIT_ERR);
	}*/

	// Allocate the two queues to be used to communicate with other tasks
	if ((devPtr->inQ = xQueueCreate(UARTQ_LEN,sizeof(UARTmsg))) == NULL) {
		// free up everyone and go home
		vQueueDelete(devPtr->binSemaphore);
		return(UART_INIT_ERR);
	}
	if ((devPtr->outQ = xQueueCreate(UARTQ_LEN,sizeof(UARTmsg))) == NULL) {
		// free up everyone and go home
		vQueueDelete(devPtr->binSemaphore);
		vQueueDelete(devPtr->outQ);
		return(UART_INIT_ERR);
	}
		
	// Initialize  UART peripheral
	//UART_ConfigStructInit(&uartConfig);
	uartConfig.Baud_rate = baud;
	uartConfig.Databits = dataBits;
	uartConfig.Parity = parity;
	uartConfig.Stopbits = stopBits;
	UART_Init(devPtr->devAddr, &uartConfig);
	
	//Setup FIFO
	UART_FIFOConfigStructInit(&uartFIFOConfig);
	UART_FIFOConfig(devPtr->devAddr, &uartFIFOConfig);

	/* Enable UART Rx interrupt on Rx Data Received*/	
	UART_IntConfig(devPtr->devAddr, UART_INTCFG_RBR, ENABLE);
	//UART_IntConfig(devPtr->devAddr, UART_INTCFG_RLS, ENABLE);
	/* Enable Interrupt for UART1 channel */
	NVIC_EnableIRQ(uartIRQn);
	
	// Enable  UART operation
	UART_TxCmd(devPtr->devAddr, ENABLE);
	
	char taskLabel[8];
	sprintf(taskLabel,"UART%d",devPtr->devNum);
	if ((retval = xTaskCreate(uartMonitorTask, (signed char*) taskLabel, UART_STACK_SIZE,(void *) devPtr, devPtr->taskPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
		return(UART_INIT_ERR) ; // return is just to keep the compiler happy, we will never get here
	}

	return UART_INIT_SUCCESS;	
}

portBASE_TYPE uartEnQ(UARTStruct *dev,uint8_t msgType, uint8_t msgID, uint8_t txLen, uint8_t *txBuf){
	UARTmsg msgBuf;
	
	msgBuf.msgType = msgType;
	msgBuf.msgID = msgID;
	msgBuf.txLen = txLen;
	msgBuf.rxLen = 0;
	msgBuf.status = 0;
	
	memcpy(msgBuf.data, txBuf, txLen);
	
	return(xQueueSend(dev->inQ,(void *)(&msgBuf),portMAX_DELAY));
}

portBASE_TYPE uartDeQ(UARTStruct *dev, uint8_t msgType, uint8_t msgID, uint8_t rxLen, uint8_t *rxBuf){

}




// uart interrupt handler
static __INLINE void UARTIsr(LPC_UART_TypeDef *devAddr, xQueueHandle outQ, xSemaphoreHandle *binSemaphore) {
    static int recByteCount = 0;
	//static int debugLED = 0x80;
	static UARTmsg msgBuf;
	//vtLEDOn(debugLED);
	//debugLED -= 0x20;
	// Receive Data Available
    if ((UART_GetIntId((LPC_UART_TypeDef*)devAddr)&UART_IIR_INTID_MASK) == UART_IIR_INTID_RDA){
		/*static signed portBASE_TYPE xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;*/
		switch(recByteCount){
			case 0:
				vtLEDOff(0x80);
				//vtLEDOn(0x80);
				msgBuf.msgType = UART_ReceiveByte(devAddr);
				msgBuf.rxLen = 0;
				recByteCount = 1;
				break;
			case 1:
				//vtLEDOn(0x40);
				msgBuf.msgID = UART_ReceiveByte(devAddr);
				recByteCount = 2;
				break;
			case 2:
				//vtLEDOn(0x20);
				msgBuf.rxLen = UART_ReceiveByte(devAddr);
				//if (msgBuf.rxLen == 0x01)	vtLEDOn(0x02);
				recByteCount = 3;
				break;
			default:
				if (recByteCount >= 3 && (recByteCount < msgBuf.rxLen + UART_MSG_MIN_SIZE)){
					//vtLEDOn(0x10);
					msgBuf.data[recByteCount-UART_MSG_MIN_SIZE] = UART_ReceiveByte(devAddr);
					++recByteCount;
				}
				if (recByteCount >= msgBuf.rxLen + UART_MSG_MIN_SIZE){
					static int LEDstate = 0;
					msgBuf.status = 1;
					msgBuf.txLen = 0;
					
					(xQueueSend(outQ,(void *)(&msgBuf),portMAX_DELAY));
					recByteCount = 0;
					if (LEDstate == 0)	{
						vtLEDOn(0x02);
						LEDstate = 1;
					}
					else{
						vtLEDOff(0x02);
						LEDstate = 0;
					}
				}
				break;
		}	
		/*xSemaphoreGiveFromISR(*binSemaphore,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);*/
	}
	vtLEDOff(0x04);
}

// Simply pass on the information to the real interrupt handler above (have to do this to work for multiple i2c peripheral units on the LPC1768
void UART1_IRQHandler(void) {
	// Log the I2C status code
	//vtITMu8(vtITMPortI2C1IntHandler,((devStaticPtr[1]->devAddr)->I2STAT & I2C_STAT_CODE_BITMASK));
	vtLEDOn(0x04);
	UARTIsr(devStaticPtr[1]->devAddr, devStaticPtr[1]->outQ, &(devStaticPtr[0]->binSemaphore));
}


static portTASK_FUNCTION( uartMonitorTask, pvParameters ){
	UARTStruct *devPtr = (UARTStruct *) pvParameters;
	UARTmsg msgBuffer;
	uint8_t packet[UART_MSG_MAX_SIZE];
	//vtLEDOn(0x80);
	for (;;){
		if (xQueueReceive(devPtr->inQ,(void *)(&msgBuffer),portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		static int LEDstate = 0;
		if (LEDstate == 0)	{
			vtLEDOn(0x01);
			LEDstate = 1;
		}
		else{
			vtLEDOff(0x01);
			LEDstate = 0;
		}
		if(UART_CheckBusy((LPC_UART_TypeDef *)LPC_UART1)==RESET){
			packet[0] = msgBuffer.msgType;
			packet[1] = msgBuffer.msgID;
			packet[2] = msgBuffer.txLen;
			memcpy(&packet[3], msgBuffer.data, msgBuffer.txLen);
			UART_Send(devPtr->devAddr, packet, msgBuffer.txLen + UART_MSG_MIN_SIZE, BLOCKING);
			//UART_SendByte((LPC_UART_TypeDef *)LPC_UART1, data);
			//++data;
			
		}
		/*if (xSemaphoreTake(devPtr->binSemaphore,portMAX_DELAY) != pdTRUE) {
			// something went wrong 
			VT_HANDLE_FATAL_ERROR(0);
		}*/
	}
}