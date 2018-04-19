/*
 * Project.h
 *
 * Created: 15/01/2018 11:05:09 AM
 *  Author: Owner
 */ 


#ifndef PROJECT_H_
#define PROJECT_H_

void custom_delay(uint32_t ticks);



typedef struct info {
	/* Variables used for tracking system information */
	
	uint16_t pwmSpeed1;
	uint16_t pwmSpeed2;
	uint16_t pwmSpeed3;
	uint16_t pwmSpeed4;

	uint8_t motorDir1;
	uint8_t motorDir2;
	uint8_t motorDir3;
	uint8_t motorDir4;

	char direction;
	
	/*	lengths	*/
	float maxStep;
	float endStep1;
	float endStep2;
	float endStep3;	
	float endStep4;
	float startStep1;
	float startStep2;
	float startStep3;
	float startStep4;
	
	
	/*	conversions and dimensions	*/
	float Kt;
	float x_01;
	float y_01;
	float x_00;
	float y_00;
	float x_11;
	float y_11;
	float x_10;
	float y_10;
	float l_00;
	float l_01;
	float l_10;
	float l_11;
	
	/*	parameter of area for spidercam	*/
	float height;
	float width;
	
	float globalTrigger1;
	float globalTrigger2;
	float globalTrigger3;
	float globalTrigger4;
	
	uint8_t avePwm;
	uint8_t doInputList;
	uint8_t nextInputIndex;
	uint8_t doRoute;
	
	/*	desired velocity	*/
	float desVel1;
	float desVel2;
	/*	time for change in time	*/
	uint32_t startTime;
	
	float changePos;
	
	float changeInTrigger;
	
} info;



#endif /* PROJECT_H_ */