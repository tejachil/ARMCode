#include "roverMap.h"

static MapCorner mapCorners[MAXIMUM_CORNERS];
static uint8_t cornersCount;

void mapRoverTask( void *param );
//static portTASK_FUNCTION_PROTO( mapRoverTask, pvParameters );

void startRoverMapping(RoverMapStruct *roverMapStruct, unsigned portBASE_TYPE uxPriority){
	cornersCount = 0;
	
	if ((roverMapStruct->inQ = xQueueCreate(ROVERMAP_QLEN,sizeof(MapCorner))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	
	if ((roverMapStruct->outQ = xQueueCreate(ROVERMAP_QLEN,sizeof(MapCorner))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	
	if (xTaskCreate(mapRoverTask, ( signed char * ) "Rover Map", mapSTACK_SIZE, (void *) roverMapStruct, uxPriority, ( xTaskHandle * ) NULL ) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
}

void mapRoverTask( void *param ){
//static portTASK_FUNCTION( mapRoverTask, pvParameters )
	RoverMapStruct *roverMapStruct = (RoverMapStruct *) param;
	
	MapCorner receivedCorner;
	
	for(;;){
		if (xQueueReceive(roverMapStruct->inQ, (void *) &receivedCorner, portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		vtLEDToggle(0x40);
		if(cornersCount < MAXIMUM_CORNERS){
			mapCorners[cornersCount] = receivedCorner;
			++cornersCount;
		}
	}
}