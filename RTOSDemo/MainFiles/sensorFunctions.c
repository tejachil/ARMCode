#include "sensorFunctions.h"
#include <math.h>

void readNewMsg(RoverControlStruct *roverControlData, public_message_t *receivedMsg){
	int i=0;
	//store each sesnor a sampling array for averaging later on
	for(i=0; i<NUMBER_OF_SENSORS;i++)
		roverControlData->sensorDistanceSamples[i][roverControlData->samplingCounter] = receivedMsg->data[2*i+1] << 8 | receivedMsg->data[2*i];
	// increment for the next sample
	roverControlData->samplingCounter = (roverControlData->samplingCounter+1)%NUMBER_OF_SAMPLES; 
}


void averageValues(RoverControlStruct *roverControlData){
	int sensorCounter=0, sampleCounter=0;
	//init sensor distance before averaging
	for(sensorCounter=0; sensorCounter<NUMBER_OF_SENSORS; sensorCounter++)
		roverControlData->sensorDistance[sensorCounter]=0;

	//average all samples for every sensor
	for(sensorCounter=0; sensorCounter<NUMBER_OF_SENSORS; sensorCounter++)
		for(sampleCounter=0; sampleCounter<NUMBER_OF_SAMPLES; sampleCounter++)
			roverControlData->sensorDistance[sensorCounter] += roverControlData->sensorDistanceSamples[sensorCounter][sampleCounter]/NUMBER_OF_SAMPLES;
}

void convertToDistance(RoverControlStruct *roverControlData){
	roverControlData->sensorDistance[LEFT_SHORT_SENSOR] = 12317*pow(roverControlData->sensorDistance[LEFT_SHORT_SENSOR],-1.337);
    roverControlData->sensorDistance[RIGHT_SHORT_SENSOR] = 12317*pow(roverControlData->sensorDistance[RIGHT_SHORT_SENSOR],-1.337);
    roverControlData->sensorDistance[LEFT_MEDIUM_SENSOR] = 5864*pow(roverControlData->sensorDistance[LEFT_MEDIUM_SENSOR],-1.099);
    roverControlData->sensorDistance[RIGHT_MEDIUM_SENSOR] = 5864*pow(roverControlData->sensorDistance[RIGHT_MEDIUM_SENSOR],-1.099);
    
    //TODO: convert other sensors
}

void checkSensorsRange(RoverControlStruct *roverControlData){
	//TODO: find if sensors are in range
}

void findAngles(RoverControlStruct *roverControlData){
	//TODO: find angle between sensors
	if(roverControlData->sensorDistance[LEFT_SHORT_SENSOR] > roverControlData->sensorDistance[RIGHT_SHORT_SENSOR]){
    	roverControlData->shortSensorAngle = atanf(DISTANCE_BETWEEN_IR/(roverControlData->sensorDistance[LEFT_SHORT_SENSOR]-roverControlData->sensorDistance[RIGHT_SHORT_SENSOR])) * 180/M_PI;
	}else{
	    roverControlData->shortSensorAngle = atan((roverControlData->sensorDistance[RIGHT_SHORT_SENSOR]-roverControlData->sensorDistance[LEFT_SHORT_SENSOR])/DISTANCE_BETWEEN_IR) * 180/M_PI;
	}
}

int isRoverParallelToWall(RoverControlStruct *roverControlData){
	//float difference = roverControlData->sensorDistance[LEFT_SHORT_SENSOR] - roverControlData->sensorDistance[RIGHT_SHORT_SENSOR];
	if(roverControlData->sensorDistance[LEFT_SHORT_SENSOR] > (roverControlData->sensorDistance[RIGHT_SHORT_SENSOR]+1)
	|| roverControlData->sensorDistance[RIGHT_SHORT_SENSOR] > (roverControlData->sensorDistance[LEFT_SHORT_SENSOR]+1)){
		//not parallel
		vtLEDOff(0x04);
		return 0;
	}
	else{
		//parallel 
		vtLEDOn(0x04);
		return 1;
	}
}

void printFloat(char* buf, float number, int newLine){
	int intPart = (int)number;
	int decimalPart = (number - (int)number)*1000;
	printf("%s %d.%d, ",buf, intPart,decimalPart);
	if(newLine)
		printf("\n");
}