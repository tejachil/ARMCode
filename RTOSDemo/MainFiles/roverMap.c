#include "roverMap.h"
#include "sensorFunctions.h"
#include <stdio.h>

static MapCorner mapCorners[MAXIMUM_CORNERS];
//static uint8_t cornersCount;
//vtLCDStruct *lcdStruct;

void mapRoverTask( void *param );
//static portTASK_FUNCTION_PROTO( mapRoverTask, pvParameters );

void startRoverMapping(RoverMapStruct *roverMapStruct, unsigned portBASE_TYPE uxPriority, xTaskHandle taskHandle){
	
	if ((roverMapStruct->inQ = xQueueCreate(ROVERMAP_QLEN,sizeof(MapCorner))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	
	if ((roverMapStruct->outQ = xQueueCreate(ROVERMAP_QLEN,sizeof(MapCorner))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

/*	char lcdBuffer[10];
	
	sprintf(lcdBuffer,"Hello!");
	if (lcdStruct != NULL) {
		
	if (SendLCDPrintMsg(lcdStruct,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
	}*/

	if (xTaskCreate(mapRoverTask, ( signed char * ) "Rover Map", mapSTACK_SIZE, (void *) roverMapStruct, uxPriority, ( xTaskHandle * ) taskHandle ) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
}

void mapRoverTask( void *param ){
//static portTASK_FUNCTION( mapRoverTask, pvParameters )
	RoverMapStruct *roverMapStruct = (RoverMapStruct *) param;
	
	MapCorner receivedCorner;

	double totalAngle = 0;
	double area;
	
	char lcdBuffer[19];
	double number;
	//int intPart, decimalPart;
	uint8_t cornersCount = 0;
	for(;;){
		if (xQueueReceive(roverMapStruct->inQ, (void *) &receivedCorner, portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Print the received distance reading
		vtLEDToggle(0x40);
		printFloat("Angle: ", receivedCorner.angleCornerExterior, 0);
		//printFloat("Side Dist: ", receivedCorner.distSide, 1);
		


		if(cornersCount != 0){
			totalAngle += receivedCorner.angleCornerExterior;
			receivedCorner.distSide += mapCorners[cornersCount-1].distFromSide;

			if ((totalAngle + mapCorners[0].angleCornerExterior) >= 350.0){
				//calculate area;
			}
		}

		if(cornersCount < MAXIMUM_CORNERS){
			mapCorners[cornersCount] = receivedCorner;
			++cornersCount;
		}

		
		/*if(totalAngle >= 370.0){
			area = (mapCorners[1].distSide + mapCorners[0].distFromSide) * (mapCorners[2].distSide + mapCorners[1].distFromSide);

			number = area;
			intPart = (int)number;
			decimalPart = (number - (int)number)*1000;
			sprintf(lcdBuffer, "Area=%d.%d, ", intPart,decimalPart);
			if (SendLCDPrintMsg(lcdStruct,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
		}
		else{*/
			/*number = receivedCorner.distSide;
			intPart = (int)number;
			decimalPart = (number - (int)number)*1000;
			sprintf(lcdBuffer, "%d.%d, ", intPart,decimalPart);
			if (SendLCDPrintMsg(lcdStruct,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}

			number = receivedCorner.distFromSide;
			intPart = (int)number;
			decimalPart = (number - (int)number)*1000;
			sprintf(lcdBuffer, "%d.%d", intPart,decimalPart);
			if (SendLCDPrintMsg(lcdStruct,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}*/

		//}
	}
}