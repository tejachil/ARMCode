#ifndef ROVERFUNCTIONS_H
#define ROVERFUNCTIONS_H
#include "roverHeader.h"

void moveRover(RoverControlStruct *roverControlData);
void stopRover(RoverControlStruct *roverControlData);
void turnRover(RoverControlStruct *roverControlData);
void fixRover(RoverControlStruct *roverControlData, int fixDir);

#endif