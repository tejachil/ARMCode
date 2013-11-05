#include "roverFunctions.h"

void sendToRover(RoverControlStruct *roverControlData, public_message_type_t msg_type, uint8_t cmd);

void moveRover(RoverControlStruct *roverControlData){
	sendToRover(roverControlData, PUB_MSG_T_MOV_CMD, 0x55);
}

void stopRover(RoverControlStruct *roverControlData){
	sendToRover(roverControlData, PUB_MSG_T_MOV_CMD, 0xAA);
}

void fixRover(RoverControlStruct *roverControlData, int fixDir){
	if(fixDir == FIX_FRONT_LEFT)
		sendToRover(roverControlData, PUB_MSG_T_FIX_CMD, 0x5A);
	else if(fixDir == FIX_FRONT_RIGHT)
		sendToRover(roverControlData, PUB_MSG_T_FIX_CMD, 0xA5);
}

void sendToRover(RoverControlStruct *roverControlData, public_message_type_t msg_type, uint8_t cmd){
	UARTmsg roverRequestMsg;
	portBASE_TYPE result;
	roverRequestMsg.msgType = msg_type;
	roverRequestMsg.msgID = 0; // The count will be updated before sending each request
	roverRequestMsg.txLen = public_message_data_size[msg_type];
	roverRequestMsg.data[0] = cmd; // move
	//roverRequestMsg.data[1] = 0x00;

	roverRequestMsg.msgID = public_message_get_count(msg_type);
	result = uartEnQ(roverControlData->uartDevice, roverRequestMsg.msgType, roverRequestMsg.msgID, roverRequestMsg.txLen,
		roverRequestMsg.data);
}