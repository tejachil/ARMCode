#ifndef SENSORFUNCTIONS_H
#define SENSORFUNCTIONS_H

#include "roverHeader.h"

void readNewMsg(RoverControlStruct *roverControlData, public_message_t *receivedMsg);
void averageValues(RoverControlStruct *roverControlData);
void convertToDistance(RoverControlStruct *roverControlData);
void findAngle(RoverControlStruct *roverControlData);
int isSensorInRange(RoverControlStruct *roverControlData);
int isRoverParallelToWall(RoverControlStruct *roverControlData);
int frontWallStatus(RoverControlStruct *roverControlData, double anglePollTotal, uint8_t anglePollCount);

void printFloat(char* buf, float number, int newLine);

void distanceToEncoder(double distance, uint8_t* revolutions, uint16_t* ticksOffset);

#endif
