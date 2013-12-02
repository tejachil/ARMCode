#include "roverMap.h"
#include "sensorFunctions.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static MapCorner mapCorners[MAXIMUM_CORNERS];
static double xPoints[MAXIMUM_CORNERS];
static double yPoints[MAXIMUM_CORNERS];

static char guiMapCoordinates[500];
static char debugBuf[500];

//static uint8_t cornersCount;
//vtLCDStruct *lcdStruct;

void mapRoverTask( void *param );
double calculateArea(uint8_t sides, double *x, double *y);

void startRoverMapping(RoverMapStruct *roverMapStruct, unsigned portBASE_TYPE uxPriority, xTaskHandle taskHandle){

	setMapCoordinatesPointer(guiMapCoordinates);
	//setDebugTextAreaPointer(guiMapCoordinates);
	
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

	setDebugTextAreaPointer(debugBuf);
	sprintf(debugBuf, "Hello from Map \n");
}

void mapRoverTask( void *param ){
//static portTASK_FUNCTION( mapRoverTask, pvParameters )
	RoverMapStruct *roverMapStruct = (RoverMapStruct *) param;
	
	MapCorner receivedCorner;

	double totalAngle = 0;
	double totalCalcAngle = 90.0;
	double area;
	
	char buf[20];
	double number;
	//int intPart, decimalPart;
	uint8_t cornersCount = 0;

	sprintf(guiMapCoordinates, "");
	
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

			// Yasir's conversion to coordinates
			xPoints[cornersCount] = xPoints[cornersCount - 1] + receivedCorner.distSide*cos(totalCalcAngle*M_PI/180.0);
			yPoints[cornersCount] = yPoints[cornersCount - 1] + receivedCorner.distSide*sin(totalCalcAngle*M_PI/180.0);
			totalCalcAngle -= receivedCorner.angleCornerExterior;

			sprintf(buf, "(%f,%f) \n%f %f\n-------------------\n", xPoints[cornersCount], yPoints[cornersCount], receivedCorner.angleCornerExterior, receivedCorner.distSide);
			strcat(debugBuf, buf);

			if ((totalAngle + mapCorners[0].angleCornerExterior) >= 340.0){
				// TODO: calculate area;
				// param for calculateArea (side) is 1 minus the number of sides
				sprintf(buf, "Side=%dArea=%f\n", cornersCount, calculateArea(5, xPoints, yPoints));
				strcat(debugBuf, buf);
			}
		}
		else{
			totalCalcAngle = 90;
			xPoints[0] = 0;
			xPoints[0] = 0;
		}

		sprintf(buf, "%d,%d ", (int)(xPoints[cornersCount]*5 + 100), (int)(yPoints[cornersCount]*-5 + 400));
		strcat(guiMapCoordinates, buf);

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

double calculateArea(uint8_t sides, double *x, double *y){
	uint8_t i = 0;
	//double sum1 = 0.0;
	//double sum2 = 0.0;
	double sum = 0;
	for(i; i < sides; ++i){
		//sum1 += x[i]*y[(i+1)%sides];
		//sum2 += y[i]*x[(i+1)%sides];
		sum += x[i]*y[(i+1)%sides] - y[i]*x[(i+1)%sides];
	}
	
	if (sum < 0.0)	sum *= -1.0;
	return sum;
}