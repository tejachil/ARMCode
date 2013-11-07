#include "roverMap.h"

static MapCorner mapCorners[MAXIMUM_CORNERS];
static uint8_t cornersCount;
vtLCDStruct *lcdStruct;

void mapRoverTask( void *param );
//static portTASK_FUNCTION_PROTO( mapRoverTask, pvParameters );

void startRoverMapping(RoverMapStruct *roverMapStruct, unsigned portBASE_TYPE uxPriority, vtLCDStruct *lcd){
	cornersCount = 0;
	lcdStruct = lcd;
	
	if ((roverMapStruct->inQ = xQueueCreate(ROVERMAP_QLEN,sizeof(MapCorner))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	
	if ((roverMapStruct->outQ = xQueueCreate(ROVERMAP_QLEN,sizeof(MapCorner))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	char lcdBuffer[10];
	
	sprintf(lcdBuffer,"Hello!");
	if (lcdStruct != NULL) {
		vtLEDToggle(0x40);
		if (SendLCDPrintMsg(lcdStruct,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
	}

	if (xTaskCreate(mapRoverTask, ( signed char * ) "Rover Map", mapSTACK_SIZE, (void *) roverMapStruct, uxPriority, ( xTaskHandle * ) NULL ) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
}

void mapRoverTask( void *param ){
//static portTASK_FUNCTION( mapRoverTask, pvParameters )
	RoverMapStruct *roverMapStruct = (RoverMapStruct *) param;
	
	MapCorner receivedCorner;
	
	char lcdBuffer[10];

	for(;;){
		if (xQueueReceive(roverMapStruct->inQ, (void *) &receivedCorner, portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}


		sprintf(lcdBuffer,"%d, %d, %d", (uint8_t)(receivedCorner.distFromSide*100), (uint8_t)(receivedCorner.distSide*100), (uint8_t)(receivedCorner.angleCorner*100));
		if (lcdStruct != NULL) {
			vtLEDToggle(0x40);
			if (SendLCDPrintMsg(lcdStruct,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
				VT_HANDLE_FATAL_ERROR(0);
			}
		}

		if(cornersCount < MAXIMUM_CORNERS){
			mapCorners[cornersCount] = receivedCorner;
			++cornersCount;
		}
	}
}