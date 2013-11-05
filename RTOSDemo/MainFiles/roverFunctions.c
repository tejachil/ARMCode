#include "roverFunctions.h"

void sendToRover(RoverControlStruct *roverControlData, uint8_t cmd);

void moveRover(RoverControlStruct *roverControlData){
	sendToRover(roverControlData, 0x55);
}

void stopRover(RoverControlStruct *roverControlData){
	sendToRover(roverControlData, 0xAA);
}

void sendToRover(RoverControlStruct *roverControlData, uint8_t cmd){
	UARTmsg roverRequestMsg;
	portBASE_TYPE result;
	roverRequestMsg.msgType = PUB_MSG_T_MOV_CMD;
	roverRequestMsg.msgID = 0; // The count will be updated before sending each request
	roverRequestMsg.txLen = public_message_data_size[PUB_MSG_T_MOV_CMD];
	roverRequestMsg.data[0] = cmd; // move
	roverRequestMsg.data[1] = 0x00;

	roverRequestMsg.msgID = public_message_get_count(PUB_MSG_T_MOV_CMD);
	result = uartEnQ(roverControlData->uartDevice, roverRequestMsg.msgType, roverRequestMsg.msgID, roverRequestMsg.txLen,
		roverRequestMsg.data);
}