#include "sensorFunctions.h"
#include <math.h>
#include <stdio.h> //for printf()

double getEncoderDistance(uint8_t revolutions, uint16_t ticksOffset);

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
	roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] = 6653.3*pow(roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR],-1.251);
    roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] = 11276*pow(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR],-1.325);
    roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR] = 5680.3*pow(roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR],-1.103);
    roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR] = 7540.4*pow(roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR],-1.15);
    roverControlData->sensorDistance[FRONT_LEFT_LONG_SENSOR] = 14130*pow(roverControlData->sensorDistance[FRONT_LEFT_LONG_SENSOR], -1.119);
	//TODO: convert other sensors
}

//check short range only
int isSensorInRange(RoverControlStruct *roverControlData){
	if(roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] < SIDE_SENSOR_RANGE || roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < SIDE_SENSOR_RANGE)
		return 1; // in range
	else
		return 0;
}

void findAngle(RoverControlStruct *roverControlData){
	//TODO: find angle between sensors
	double sideAngle;
    if(roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]){
    	sideAngle = atanf((roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]-roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR])/DISTANCE_BETWEEN_SIDE_SHORT) * 180/M_PI;
    	roverControlData->frontSensorAngle = atanf(DISTANCE_BETWEEN_FRONT_MEDIUM/(roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR]-roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR])) * 180/M_PI - sideAngle;
	}
	else{
		sideAngle = atanf((roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR]-roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR])/DISTANCE_BETWEEN_SIDE_SHORT) * 180/M_PI;
    	roverControlData->frontSensorAngle = atanf(DISTANCE_BETWEEN_FRONT_MEDIUM/(roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR]-roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR])) * 180/M_PI + sideAngle;

	}
    if(roverControlData->frontSensorAngle <= 0.0)
    	roverControlData->frontSensorAngle *= -1.0;
    /*if(roverControlData->frontSensorAngle >= 90.0)
    	roverControlData->frontSensorAngle = 90.0;*/
    /*if(roverControlData->frontSensorAngle <= 0.0  || roverControlData->frontSensorAngle >= 90.0){
    	roverControlData->frontSensorAngle = 90.0;
    }*/
    	//roverControlData->frontSensorAngle+=5.0;
}

int isRoverParallelToWall(RoverControlStruct *roverControlData){
	//float difference;
	//difference = roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR];
	//if(!isSensorInRange(roverControlData))
	//	return TOO_FAR_SIDE;
	if(roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] < TOO_CLOSE_THRESHOLD
	|| roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] < TOO_CLOSE_THRESHOLD){
		//vtLEDOff(0x04);
		return TOO_CLOSE_SIDE;
	}
	else if((roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR]) > PARALLEL_THRESHOLD){
		//vtLEDOff(0x04);
		return FIX_FRONT_LEFT;
	}
	else if((roverControlData->sensorDistance[SIDE_REAR_SHORT_SENSOR] - roverControlData->sensorDistance[SIDE_FRONT_SHORT_SENSOR]) > PARALLEL_THRESHOLD){
		//vtLEDOff(0x04);
		return FIX_FRONT_RIGHT;
	}
	else{
		//vtLEDOn(0x04);
		return PARALLEL;
	}	
}

int frontWallStatus(RoverControlStruct *roverControlData, double anglePollTotal, uint8_t anglePollCount){
	float average = roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR] + roverControlData->sensorDistance[FRONT_RIGHT_MEDIUM_SENSOR];
	average /= 2;
	//find the adjustment necessary for the front threshold based on the front angle
	int thresholdAdjustment  = 0;
	if(anglePollCount > 7){
		thresholdAdjustment = (90 - anglePollTotal/anglePollCount) * FRONT_THRESHOLD_ADJUSTMENT;
		if(thresholdAdjustment < 0) thresholdAdjustment = 0;
	}
	//Using average is not a good idea, we are using threshold adjustment for now.
	//if(average < (FRONT_STOP_DISTANCE + thresholdAdjustment)){
	if(roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR] < (FRONT_STOP_DISTANCE + thresholdAdjustment)){
		return CLOSE_FRONT_WALL;
	}
	else if(roverControlData->sensorDistance[FRONT_LEFT_MEDIUM_SENSOR] < FRONT_AQUIRE_ANGLE_DISTANCE){ // between 10 inches and 15 inches
		return ACQUIRE_FRONT_ANGLE;
	}
	else{
		return FAR_FRONT_WALL;
	}
}

void printFloat(char* buf, float number, int newLine){
	int intPart = (int)number;
	int decimalPart = (number - (int)number)*1000;
	printf("%s %4d.%-4d, ",buf, intPart, decimalPart);
	if(newLine)
		printf("\n");
}

double getEncoderDistance(uint8_t revolutions, uint16_t ticksOffset){
	double distance = WHEEL_CIRCUMFERENCE * (revolutions + ((ticksOffset)/TICKS_PER_REVOLUTION));
	return distance;
}