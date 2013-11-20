#include "roverFunctions.h"

void sendToRover(RoverControlStruct *roverControlData, public_message_type_t msg_type, uint8_t cmd);

void moveRover(RoverControlStruct *roverControlData){
	roverControlData->state = TRAVERSAL;
	sendToRover(roverControlData, PUB_MSG_T_MOV_CMD, MOV_CMD_GO);
}

void stopRover(RoverControlStruct *roverControlData){
	roverControlData->state = STOP;
	sendToRover(roverControlData, PUB_MSG_T_MOV_CMD, MOV_CMD_STOP);
}

void turnRover(RoverControlStruct *roverControlData){
	roverControlData->state = TURN;
	//TODO: change this to turn later
	sendToRover(roverControlData, PUB_MSG_T_TURN_CMD, (uint8_t)roverControlData->frontSensorAngle);
}

void fixRover(RoverControlStruct *roverControlData, int fixDir){
	roverControlData->state = FIX;
	sendToRover(roverControlData, PUB_MSG_T_FIX_CMD, fixDir);
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