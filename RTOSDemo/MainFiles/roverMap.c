#include "roverMap.h"
#include "sensorFunctions.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static MapCorner mapCorners[MAXIMUM_CORNERS];
static double xPoints[MAXIMUM_CORNERS];
static double yPoints[MAXIMUM_CORNERS];

#define BUFFER_SIZE		500

static char guiMapCoordinates[BUFFER_SIZE];
static char debugBuf[BUFFER_SIZE];

//static uint8_t cornersCount;
//vtLCDStruct *lcdStruct;

void mapRoverTask( void *param );
double calculateArea(uint8_t sides, double *x, double *y);

double calculateRegularArea(uint8_t sides, uint8_t totalSides);

void startRoverMapping(RoverMapStruct *roverMapStruct, unsigned portBASE_TYPE uxPriority, xTaskHandle* taskHandle){

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

		if(strlen(guiMapCoordinates) >= (BUFFER_SIZE - 10)){
			sprintf(guiMapCoordinates, "");
		}
		if(strlen(debugBuf) >= (BUFFER_SIZE - 10)){
			sprintf(debugBuf, "");
		}
		printFloat("QReced:", receivedCorner.distSide, 1);
		// Print the received distance reading
		vtLEDToggle(0x40);
		printFloat("Angle: ", receivedCorner.angleCornerExterior, 0);
		//printFloat("Side Dist: ", receivedCorner.distSide, 1);
		
		totalAngle += receivedCorner.angleCornerExterior;

		// For a regular polygon
		if((roverMapStruct->taskFlags)&REGULAR){
			if(cornersCount != 0){
				receivedCorner.distSide += mapCorners[cornersCount-1].distFromSide;
				sprintf(buf, "Deg=%.1f Len=%.1f S=%d\n", receivedCorner.angleCornerExterior, receivedCorner.distSide, cornersCount);
				strcat(debugBuf, buf);

				sprintf(buf, "A=%f\n", cornersCount, calculateRegularArea(cornersCount+1, roverMapStruct->numberSides));
				strcat(debugBuf, buf);
				
				if ((totalAngle) >= 350.0){
					// param for calculateArea (side) is 1 minus the number of sides
					sprintf(buf, "** Greater than 350 **, S=%d A=%f\n", cornersCount, calculateRegularArea(cornersCount+1, roverMapStruct->numberSides));
					strcat(debugBuf, buf);
				}
			}
		}

		// If it is not a regular polygon
		else{
			if(cornersCount != 0){
				//totalAngle += receivedCorner.angleCornerExterior;
				receivedCorner.distSide += mapCorners[cornersCount-1].distFromSide;
				
				// Yasir's conversion to coordinates
				xPoints[cornersCount] = xPoints[cornersCount - 1] + receivedCorner.distSide*cos(totalCalcAngle*M_PI/180.0);
				yPoints[cornersCount] = yPoints[cornersCount - 1] + receivedCorner.distSide*sin(totalCalcAngle*M_PI/180.0);
				totalCalcAngle -= receivedCorner.angleCornerExterior;

				sprintf(buf, "Deg=%.1f Len=%.1f S=%d\n", receivedCorner.angleCornerExterior, receivedCorner.distSide, cornersCount);
				strcat(debugBuf, buf);

				sprintf(buf, "A=%f\n", calculateArea(cornersCount+1, xPoints, yPoints));
				strcat(debugBuf, buf);
				
				if ((totalAngle) >= 350.0){
					// param for calculateArea (side) is 1 minus the number of sides
					sprintf(buf, "** Greater than 350 **, A=%f\n", calculateArea(cornersCount+1, xPoints, yPoints));
					strcat(debugBuf, buf);
				}
			}
			else{
				totalCalcAngle = 90;
				xPoints[0] = 0;
				xPoints[0] = 0;
			}
		}


		if(cornersCount < MAXIMUM_CORNERS){
			mapCorners[cornersCount] = receivedCorner;
			++cornersCount;
		}
	}
}

double calculateArea(uint8_t sides, double *x, double *y){
	uint8_t i = 0;
	double sum = 0.0;
	for(i; i < sides; ++i){
		sum += x[i]*y[(i+1)%sides] - y[i]*x[(i+1)%sides];
	}
	sum  /= 2.0;
	
	if (sum < 0.0)	sum *= -1.0;
	return sum;
}

double calculateRegularArea(uint8_t sides, uint8_t totalSides){
	double length = 0.0;
	double angle = 0.0;
	char buf[20];
	uint8_t i = 1;
	for(i; i < sides; ++i){
		length += mapCorners[i].distSide; 
		angle += mapCorners[i].angleCornerExterior;
	}
	sprintf(buf, "TOT L=%f AN=%f\n", length, angle);
	strcat(debugBuf, buf);
	length /= (sides-1);
	angle /= (sides-1);
	sprintf(buf, "AV L=%f AN=%f\n", length, angle);
	strcat(debugBuf, buf);

	if(totalSides == 0)		totalSides = 360/angle;

	double area;
	area = 4*tan(M_PI/totalSides);
	
	sprintf(buf, "TAN %f\n", area);
	strcat(debugBuf, buf);

	area = totalSides*length*length/area;

	return area;
}