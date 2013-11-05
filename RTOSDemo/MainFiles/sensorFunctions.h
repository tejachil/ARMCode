#ifndef SENSORFUNCTIONS_H
#define SENSORFUNCTIONS_H

#include "roverControl.h"

void readNewMsg(RoverControlStruct *roverControlData, public_message_t *receivedMsg);
void averageValues(RoverControlStruct *roverControlData);
void convertToDistance(RoverControlStruct *roverControlData);
void checkSensorsRange(RoverControlStruct *roverControlData);
void findAngles(RoverControlStruct *roverControlData);
int isRoverParallelToWall(RoverControlStruct *roverControlData);

void printFloat(char* buf, float number, int newLine);

#endif
