#include "roverFunctions.h"
void moveRover(RoverControlStruct *roverControlData){
	UARTmsg roverRequestMsg;
	portBASE_TYPE result;
	roverRequestMsg.msgType = PUB_MSG_T_MOV_CMD;
	roverRequestMsg.msgID = 0; // The count will be updated before sending each request
	roverRequestMsg.txLen = public_message_data_size[PUB_MSG_T_MOV_CMD];
	roverRequestMsg.data[0] = 0x55; // move
	roverRequestMsg.data[1] = 0x00;

	roverRequestMsg.msgID = public_message_get_count(PUB_MSG_T_MOV_CMD);
	result = uartEnQ(roverControlData->uartDevice, roverRequestMsg.msgType, roverRequestMsg.msgID, roverRequestMsg.txLen,
		roverRequestMsg.data);

	if(result == pdTRUE){
		printf(" success\n");
	}
	else if (result == pdFALSE){
		printf(" failure\n");
	}
}

void stopRover(RoverControlStruct *roverControlData){
	UARTmsg roverRequestMsg;
	portBASE_TYPE result;
	roverRequestMsg.msgType = PUB_MSG_T_MOV_CMD;
	roverRequestMsg.msgID = 0; // The count will be updated before sending each request
	roverRequestMsg.txLen = public_message_data_size[PUB_MSG_T_MOV_CMD];
	roverRequestMsg.data[0] = 0xAA; // stop
	roverRequestMsg.data[1] = 0x00;

	roverRequestMsg.msgID = public_message_get_count(PUB_MSG_T_MOV_CMD);
	result = uartEnQ(roverControlData->uartDevice, roverRequestMsg.msgType, roverRequestMsg.msgID, roverRequestMsg.txLen,
		roverRequestMsg.data);

	if(result == pdTRUE){
		printf(" success\n");
	}
	else if (result == pdFALSE){
		printf(" failure\n");
	}
}