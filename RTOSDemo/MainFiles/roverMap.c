#include "roverMap.h"
#include "sensorFunctions.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static MapCorner mapCorners[MAXIMUM_CORNERS];
static double xPoints[MAXIMUM_CORNERS];
static double yPoints[MAXIMUM_CORNERS];

#define BUFFER_SIZE		500
#define TOTAL_ANGLE_THRESHOLD 330

static char guiMapCoordinates[BUFFER_SIZE];
static char debugBuf[BUFFER_SIZE];
static uint8_t polyComp;

//static uint8_t cornersCount;
//vtLCDStruct *lcdStruct;

void mapRoverTask( void *param );
double calculateArea(uint8_t sides, double *x, double *y);

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
	polyComp = 0;
	
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
		if(cornersCount != 0){
			//totalAngle += receivedCorner.angleCornerExterior;
			receivedCorner.distSide += mapCorners[cornersCount-1].distFromSide;
			printFloat("PrevSide:", mapCorners[cornersCount-1].distFromSide, 1);
			printFloat("WithPrevSide:", receivedCorner.distSide, 1);
			// Yasir's conversion to coordinates
			xPoints[cornersCount] = xPoints[cornersCount - 1] + receivedCorner.distSide*cos(totalCalcAngle*M_PI/180.0);
			yPoints[cornersCount] = yPoints[cornersCount - 1] + receivedCorner.distSide*sin(totalCalcAngle*M_PI/180.0);
			totalCalcAngle -= receivedCorner.angleCornerExterior;

			sprintf(buf, "Deg=%.1f Len=%.1f S=%d\n", receivedCorner.angleCornerExterior, receivedCorner.distSide, cornersCount);
			strcat(debugBuf, buf);

			sprintf(buf, "A=%f\n", calculateArea(cornersCount+1, xPoints, yPoints));
			strcat(debugBuf, buf);
			if ((totalAngle) >= TOTAL_ANGLE_THRESHOLD){
				// TODO: calculate area;
				// param for calculateArea (side) is 1 minus the number of sides
				polyComp = cornersCount;
				sprintf(buf, "** Polygon complete **, A=%f\n", calculateArea(cornersCount+1, xPoints, yPoints));
				strcat(debugBuf, buf);
			}
		}
		else{
			totalCalcAngle = 90;
			xPoints[0] = 0;
			xPoints[0] = 0;
		}

		sprintf(buf, "%d,%d ", (int)(xPoints[cornersCount]*MAP_SCALE_FACTOR + MAP_X_OFFSET), (int)(yPoints[cornersCount]*-MAP_SCALE_FACTOR + MAP_Y_OFFSET));
		strcat(guiMapCoordinates, buf);

		/*sprintf(buf, "Deg=%f Len=%f S=%d\n", receivedCorner.angleCornerExterior, receivedCorner.distSide, cornersCount);
		strcat(debugBuf, buf);
		
		sprintf(buf, "%f, %f\n", xPoints[cornersCount], yPoints[cornersCount]);
		strcat(debugBuf, buf);

		sprintf(buf, "A=%f\n", calculateArea(cornersCount+1, xPoints, yPoints));
		strcat(debugBuf, buf);
*/
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
	double sum = 0.0;
	for(i; i < sides; ++i){
	//	sum1 += x[i]*y[(i+1)%sides];
	//	sum2 += y[i]*x[(i+1)%sides];
		sum += x[i]*y[(i+1)%sides] - y[i]*x[(i+1)%sides];
	}
	sum  /= 2.0;
	
	if (sum < 0.0)	sum *= -1.0;
	return sum;
}

void convertGotoCoordinates(RoverMapStruct *map){
	map->gotoX = (map->gotoX - MAP_X_OFFSET)/MAP_SCALE_FACTOR;
	map->gotoY = (map->gotoY - MAP_Y_OFFSET)/-MAP_SCALE_FACTOR;

	sprintf(debugBuf, "GOTOX=%.2f GOTOY=%.2f\n", map->gotoX, map->gotoY);
}

uint8_t getGotoAngle(RoverMapStruct *map){
	uint8_t returnAngle = (uint8_t)(atanf(map->gotoX/map->gotoY)*180/M_PI);

	char buf[20];
	sprintf(buf, "gotoAng=%d\n", returnAngle);
	strcat(debugBuf, buf);

	return returnAngle;
}

double getGotoDistance(RoverMapStruct *map){
	double returnDist = sqrt(map->gotoX*map->gotoX + map->gotoY*map->gotoY);

	char buf[20];
	sprintf(buf, "gotoDist=%f\n", returnDist);
	strcat(debugBuf, buf);
	return returnDist;
}

uint8_t polygonComplete(){
	return polyComp;
}