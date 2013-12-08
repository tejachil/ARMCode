#ifndef ROVERMAP_H_
#define ROVERMAP_H_

#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"
#include "vtUtilities.h"
//#include "LCDtask.h"

#define MAXIMUM_CORNERS	10
#define ROVERMAP_QLEN	10

#define baseStack 2

#if PRINTF_VERSION == 1
#define mapSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define mapSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

typedef struct __RoverMapStruct {
	xQueueHandle inQ;
	xQueueHandle outQ;
	uint8_t taskFlags;
	uint8_t numberSides;
	double gotoX, gotoY;
} RoverMapStruct;

typedef struct __MapCornerStruct {
	double angleCornerExterior;
	double distSide;
	double distFromSide;
} MapCorner;

void startRoverMapping(RoverMapStruct *roverMapStruct, unsigned portBASE_TYPE uxPriority, xTaskHandle* taskHandle);

void convertGotoCoordinates(RoverMapStruct *map);

uint8_t getGotoAngle(RoverMapStruct *map);

double getGotoDistance(RoverMapStruct *map);

uint8_t polygonComplete();

#endif