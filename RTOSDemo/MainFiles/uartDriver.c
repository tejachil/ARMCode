#include <stdlib.h>
#include <stdio.h>

#include "uartDriver.h"

static 	UARTstruct *devStaticPtr[4];

static portTASK_FUNCTION_PROTO( uartMonitorTask, pvParameters );

void UART1_IRQHandler(void);

int initUART(UARTstruct *devPtr, uint8_t uartDevNum, unsigned portBASE_TYPE taskPriority, uint32_t baud, 
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
			pinConfig.Pinnum = 16;	// UART1 RX Pin
			PINSEL_ConfigPin(&pinConfig);
			break;
		default:
			return(UART_INIT_ERR);
			break;			
	}
	
	// Allocate the two queues to be used to communicate with other tasks
	if ((devPtr->inQ = xQueueCreate(UARTQ_LEN,sizeof(UARTmsg))) == NULL) {
		return(UART_INIT_ERR);
	}
	if ((devPtr->outQ = xQueueCreate(UARTQ_LEN,sizeof(UARTmsg))) == NULL) {
		vQueueDelete(devPtr->outQ);
		return(UART_INIT_ERR);
	}
		
	// Initialize  UART peripheral
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
		return(UART_INIT_ERR); // return is just to keep the compiler happy, we will never get here
	}

	return UART_INIT_SUCCESS;	
}

portBASE_TYPE uartEnQ(UARTstruct *dev,uint8_t msgType, uint8_t msgID, uint8_t txLen, uint8_t *txBuf){
	UARTmsg msgBuf;
	
	msgBuf.msgType = msgType;
	msgBuf.msgID = msgID;
	msgBuf.txLen = txLen;
	msgBuf.rxLen = 0;
	msgBuf.status = 0;
	
	memcpy(msgBuf.data, txBuf, txLen);
	
	return(xQueueSend(dev->inQ,(void *)(&msgBuf),portMAX_DELAY));
}

portBASE_TYPE uartDeQ(UARTstruct *dev, UARTmsg *message){
	if (xQueueReceive(dev->outQ,(void *) (&message),portMAX_DELAY) != pdTRUE)
		return(pdFALSE);
	return(pdTRUE);
}


// uart interrupt handler
static __INLINE void UART_ISR(LPC_UART_TypeDef *devAddr, xQueueHandle outQ) {
    static int recByteCount = 0;
	static UARTmsg msgBuf;
    if ((UART_GetIntId((LPC_UART_TypeDef*)devAddr)&UART_IIR_INTID_MASK) == UART_IIR_INTID_RDA){
		switch(recByteCount){
			case 0:
				msgBuf.msgType = UART_ReceiveByte(devAddr);
				msgBuf.rxLen = 0;
				recByteCount = 1;
				break;
			case 1:
				msgBuf.msgID = UART_ReceiveByte(devAddr);
				recByteCount = 2;
				break;
			case 2:
				msgBuf.rxLen = UART_ReceiveByte(devAddr);
				recByteCount = 3;
				break;
			default:
				if (recByteCount >= 3 && (recByteCount < msgBuf.rxLen + UART_MSG_MIN_SIZE)){
					msgBuf.data[recByteCount-UART_MSG_MIN_SIZE] = UART_ReceiveByte(devAddr);
					++recByteCount;
				}
				if (recByteCount >= msgBuf.rxLen + UART_MSG_MIN_SIZE){
					msgBuf.status = 1;
					msgBuf.txLen = 0;
					
					(xQueueSend(outQ,(void *)(&msgBuf),portMAX_DELAY));
					recByteCount = 0;
				}
				break;
		}
	}
}

// Simply pass on the information to the real interrupt handler above (have to do this to work for multiple i2c peripheral units on the LPC1768
void UART1_IRQHandler(void) {
	UART_ISR(devStaticPtr[1]->devAddr, devStaticPtr[1]->outQ);
}


static portTASK_FUNCTION( uartMonitorTask, pvParameters ){
	UARTstruct *devPtr = (UARTstruct *) pvParameters;
	UARTmsg msgBuffer;
	uint8_t packet[UART_MSG_MAX_SIZE];
	for (;;){
		if (xQueueReceive(devPtr->inQ,(void *)(&msgBuffer),portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		if(UART_CheckBusy((LPC_UART_TypeDef *)LPC_UART1)==RESET){
			packet[0] = msgBuffer.msgType;
			packet[1] = msgBuffer.msgID;
			packet[2] = msgBuffer.txLen;
			memcpy(&packet[3], msgBuffer.data, msgBuffer.txLen);
			UART_Send(devPtr->devAddr, packet, msgBuffer.txLen + UART_MSG_MIN_SIZE, BLOCKING);
			
		}
	}
}