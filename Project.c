/*
 * SpiderCam.c
 *
 * Created: 15/01/2018 8:13:31 AM
 * Author : Callum Rohweder
 */ 


/*	standard libraries	*/
#include <avr/io.h>
#include <avr/interrupt.h>			//enables interrupts
#include <avr/pgmspace.h>			//for storing / accessing data in flash program instead of SRAM -> variable manipulation
#include <stdio.h>					//input / output
#include <stdlib.h>					//standard function library; malloc / sort / rand etc
#include <math.h>
#include <string.h>

/*	custom libraries	*/
#include "Project.h"
//#include "encoder.h"
//#include "enco.h"
#include "trigger.h"
#include "serialio.h"
#include "tcnt0.h"
#include "tcnt5.h"
#include "tcnt4.h"


/*	set the UBRR0 register to BAUDREG (12 for 9600k baudrate) */
#define F_CPU 8000000L						//Internal Calibrated RC Oscillator 8MHz
#define BAUDRATE 9600L						//from bluetooth datasheet 9600
#define BAUDREG ((F_CPU)/(BAUDRATE*16UL)-1)


/*	function declarations	*/
info* makeInfo(void);
void initialize(void);
void input(info* info_ptr, char c);
void custom_delay(uint32_t ticks);
void check_movement(info* info_ptr);
void get_desired_triggers(info* info_ptr);
void check_movement(info* info_ptr);
void get_position(info* info_ptr);
void adjust_position(info* info_ptr);
void pwm_control(info* info_ptr);
void calculate_route(info* info_ptr, uint8_t** route);
void define_route(info* info_ptr, uint8_t** route);
void parameters(info* info_ptr);

void define_route(info* info_ptr, uint8_t** route){
	/*	writes to the route buffer a desired route	
	*	route[row][coloumn]
	*	route[x, y][0,1,2,3,4,5,6,7,8,9,10]
	*/
	
	int x[10] = {info_ptr->x_00 + 20,	info_ptr->x_00 + 50,	info_ptr->x_00 + 50,	info_ptr->x_00 + 50,	info_ptr->x_00,			info_ptr->x_00,	info_ptr->x_00 - 20,	info_ptr->x_00 - 50,	info_ptr->x_00 - 50,	info_ptr->x_00};
	int y[10] = {info_ptr->y_00,		info_ptr->y_00 - 50,	info_ptr->y_00 - 60,	info_ptr->y_00 - 80,	info_ptr->y_00 - 80,	info_ptr->y_00,	info_ptr->y_00,			info_ptr->y_00 - 50,	info_ptr->y_00 - 60,	info_ptr->y_00};
	for (int j = 0; j < 10; j++){
		for (int i = 0; i < 2; i++){
			if (i == 0){
				route[i][j] = x[j];
			} else {
				route[i][j] = y[j];
			}
		}
	}
}

void parameters(info* info_ptr){
	/*	sets all of the constants for the rest of the code	*/
	
	/*	set parameter of spidercam	*/
	info_ptr->width = 185.0;
	info_ptr->height = 185.0 * 2;	/*	times two for two motor prototype	*/

	/*	set motor triggers as forward	*/
	info_ptr->motorDir1 = 0xFF;
	info_ptr->motorDir2 = 0xFF;
	info_ptr->motorDir3 = 0xFF;
	info_ptr->motorDir4 = 0xFF;
	/*	no signal for direction yet	*/
	info_ptr->direction = 'N';
	
	/*	set initial and next position as centre of parameter	*/
	info_ptr->x_00 = info_ptr->width / 2.0;
	info_ptr->y_00 = info_ptr->height / 2.0;
	info_ptr->x_01 = info_ptr->width / 2.0;
	info_ptr->y_01 = info_ptr->height / 2.0;
	
	/*	set length of motor1 and next length of motor1	*/
	info_ptr->l_00 = sqrt(pow(info_ptr->x_00, 2) + pow(info_ptr->y_00, 2));
	info_ptr->l_01 = sqrt(pow(info_ptr->x_01, 2) + pow(info_ptr->y_01, 2));
	
	/*	set initial and next position as center of parameter	*/
	info_ptr->x_10 = info_ptr->width / 2.0;
	info_ptr->y_10 = info_ptr->height / 2.0;
	info_ptr->x_11 = info_ptr->width / 2.0;
	info_ptr->y_11 = info_ptr->height / 2.0;
	info_ptr->changePos = 0.0;
	
	/*	set length of motor1 and next length of motor1	*/
	info_ptr->l_10 = sqrt(pow(info_ptr->x_10, 2) + pow(info_ptr->y_10, 2));
	info_ptr->l_11 = sqrt(pow(info_ptr->x_11, 2) + pow(info_ptr->y_11, 2));
	
	/*	triggers per cm	*/
	info_ptr->Kt = 10.0/28.0;
	
	/*	Offset encoder values	*/
	info_ptr->globalTrigger1 = (info_ptr->Kt * info_ptr->l_00);
	info_ptr->globalTrigger2 = (info_ptr->Kt * info_ptr->l_10);
	info_ptr->globalTrigger3 = (info_ptr->Kt * info_ptr->l_00);
	info_ptr->globalTrigger4 = (info_ptr->Kt * info_ptr->l_00);
	info_ptr->endStep1 = info_ptr->globalTrigger1;
	info_ptr->endStep2 = info_ptr->globalTrigger2;
	
	/*	maximum # of trigger steps per movement command	*/
	//info_ptr->maxStep = 5;
	
	/*	initialise step position	*/
	//int16_t startStep1 = 0;
	info_ptr->avePwm = 70;
	info_ptr->doInputList = 0x00;
	info_ptr->doRoute = 0x00;
	info_ptr->nextInputIndex = 0;

}

/* functional code	*/
void initialize(void){
	
	/*	Serial	*/
	init_serial_stdio(BAUDRATE, 0);
	
	/*	Timer in milliseconds	*/
	init_tcnt0();
	
	/*	program PWM's	*/
	init_tcnt5();
	init_tcnt4();
	
	/*	setup triggers	*/
	init_triggers();

	/*	enable interrupts and wait	*/
	sei();
	
	/*	Serial echo mode	*/
	while (1){
		
		if (serial_input_available()){
			char in = fgetc(stdin);
			
			if ((in != '\r') || (in != '\n')){
				fputc(in - 32, stdout);
				} else {
				fputc(in, stdout);
			}
		}
	}
	
	
	
	/*	Wait for serial	*/
	uint8_t check = 0;
	while (check < 6){
		if (serial_input_available()){
			char in = fgetc(stdin);
			fputc(in, stdout);
			if (in == '\n'){
				check = 10;
				break;
			}
			++check;
		}
	}
	
	custom_delay(200);
	fputs("System prepared\n", stdout);
	// Motor output pins
	DDRA |= (1<<PA0)|(1<<PA1)|(1<<PA2)|(1<<PA3)|(1<<PA4);

}


int main(void)
{
	/*	Initialise information structure	*/
	info* info_ptr = makeInfo();

	parameters(info_ptr);
	
	/*	buffer to hold inputs	*/
	char* inputBuffer = malloc(sizeof(char)*19);
	inputBuffer = "wwwwddssaaaawddsss";
	
	/*	Define route
	*	2 rows, 10 columns, route[i][j] where [i] is row and [j] is column	*/
	uint8_t** route = malloc(sizeof(uint8_t*)*2);
	for (int i = 0; i < 2; ++i){
		route[i] = malloc(sizeof(uint8_t) * 10);
	}
	define_route(info_ptr, route);
	
	
	/*	Initialise all registers	*/
	initialize();
	
	/*	check initialised variables over serial	*/
	char buff[100];
	sprintf(buff, "some variables: %f, %f, %f \n", info_ptr->Kt, info_ptr->x_00, info_ptr->y_00);
	fputs(buff, stdout);
	char buff2[200];
	sprintf(buff2, "%f, %f, %f, %f \n", info_ptr->width, info_ptr->height, info_ptr->x_01, info_ptr->y_01);
	fputs(buff2, stdout);
	char buff3[200];
	sprintf(buff3, "%f\n", info_ptr->l_00);
	fputs(buff3, stdout);
	char buff4[200];
	sprintf(buff4, "trig1: %f, %f\n", info_ptr->globalTrigger1, info_ptr->globalTrigger2);
	fputs(buff4, stdout);
    
	/*	adjust position for accuracy	*/
	adjust_position(info_ptr);

	/* Replace with your application code */
    while (1) {
		/* Executes commands and transmits serial back */
		if(serial_input_available()){

			input(info_ptr, 'X');
		}
		
		if (info_ptr->doInputList && (info_ptr->direction == 'N')){
			/*	the system has moved to the desired position, can process next step from input list	*/	
			fputs("getting input from buffer\n", stdout);
			input(info_ptr, inputBuffer[info_ptr->nextInputIndex]);
			++info_ptr->nextInputIndex;
			if (info_ptr->nextInputIndex == 18){
				/*	end of list reached, stop reading from list	*/
				info_ptr->doInputList = 0x00;
				info_ptr->nextInputIndex = 0;
			}
		}
		
		if (info_ptr->doRoute && (info_ptr->direction == 'N')){
			/*	get coordinate from list	*/
			calculate_route(info_ptr, route);
			if (info_ptr->nextInputIndex == 10){
				/*	end of list reached, stop reading from list	*/
				info_ptr->doRoute = 0x00;
				info_ptr->nextInputIndex = 0;
			}	
		}
		
		/*	update encoder/trigger values	*/
		get_triggers(info_ptr);
		
		if (info_ptr->direction != 'N'){
			/*	received command to move a number of trigger values, check if reached target	*/
			check_movement(info_ptr);	
		}
		if (info_ptr->direction != 'N'){
			/*	motors are rolling, check speed	*/
			pwm_control(info_ptr);	
		}
	}
	
	free(inputBuffer);
	
}


info* makeInfo(void) {
	/* initialise the submersible's information struct,
	*  which will contain all the necessary information to operate */
	info* info_ptr = malloc(sizeof(info));
	return info_ptr;
}


/*	
*	Inputs: info_ptr is a pointer to a struct containing information about the dynamics of the spidercam
*			c determines whether the next input comes from stdin 0xFF or a list containing 'movements' 0x00
*/
void input(info* info_ptr, char c){
	
	
	if (c == 'X'){
		if(!serial_input_available()){
			return;
		}
		c = fgetc(stdin);
	} else {
		char buffness[50];
		sprintf(buffness, "char: %c\n", c);
		fputs(buffness, stdout);
	}
	/* echo character */
	fputc(':', stdout);
	fputc(c, stdout);
	fputc(':', stdout);

	/*	output buffer	*/
	char buffer[50];
	int16_t count;
	/*	check received command	*/
	switch(c){
		case 'a' :
			// do something
			PORTA |= (1<<PA1)|(1<<PA4);
			PORTA &= ~((1<<PA0)|(1<<PA3));
			info_ptr->motorDir1 = 0x00;
			info_ptr->motorDir2 = 0xFF;
			info_ptr->direction = 'L';
			
			/*	want to move to the left 28cm from current pos	*/
			info_ptr->x_00 = info_ptr->x_01;
			info_ptr->x_01 += 28.0;
			info_ptr->y_00 = info_ptr->y_01;
			
			info_ptr->x_10 = info_ptr->x_11;
			info_ptr->x_11 -= 28.0;
			info_ptr->y_10 = info_ptr->y_11;
			
			info_ptr->changePos = 28.0;
			/*	update position information */
			get_desired_triggers(info_ptr);
			
			break;
		case 's' :

			PORTA |= (1<<PA0)|(1<<PA4);
			PORTA &= ~((1<<PA1)|(1<<PA3));
			info_ptr->motorDir1 = 0x00;
			info_ptr->motorDir2 = 0x00;
			info_ptr->direction = 'D';
			
			info_ptr->x_00 = info_ptr->x_01;
			info_ptr->y_00 = info_ptr->y_01;
			info_ptr->y_01 += 28.0;
			
			info_ptr->x_10 = info_ptr->x_11;
			info_ptr->y_10 = info_ptr->y_11;
			info_ptr->y_11 += 28.0;
			info_ptr->changePos = 28.0;
			
			get_desired_triggers(info_ptr);
			break;
		case 'w' :
			PORTA &= ~((1<<PA0)|(1<<PA4));
			PORTA |= (1<<PA1)|(1<<PA3);
			info_ptr->motorDir1 = 0xFF;
			info_ptr->motorDir2 = 0xFF;
			info_ptr->direction = 'U';
			
			info_ptr->y_00 = info_ptr->y_01;
			info_ptr->x_00 = info_ptr->x_01;
			info_ptr->y_01 -= 28.0;
			
			info_ptr->y_10 = info_ptr->y_11;
			info_ptr->x_10 = info_ptr->x_11;
			info_ptr->y_11 -= 28.0;
			
			info_ptr->changePos = 28.0;
			get_desired_triggers(info_ptr);
			
			break;
		case 'd' :
			PORTA |= (1<<PA0)|(1<<PA3);
			PORTA &= ~((1<<PA1)|(1<<PA4));
			info_ptr->motorDir1 = 0xFF;
			info_ptr->motorDir2 = 0x00;
			info_ptr->direction = 'R';
			/*	want to move to the left 20cm from current pos	*/
			info_ptr->x_00 = info_ptr->x_01;
			info_ptr->y_00 = info_ptr->y_01;
			info_ptr->x_01 -= 28.0;
			
			info_ptr->x_10 = info_ptr->x_11;
			info_ptr->y_10 = info_ptr->y_11;
			info_ptr->x_11 += 28.0;
			
			info_ptr->changePos = 28.0;
			/*	update position information */
			get_desired_triggers(info_ptr);
			
			break;
		case '=' :
			/*	Increase motor speed	*/
			if ((OCR5A < 180) && (OCR5B < 180)){
				OCR4A += 10;
				OCR4B += 10;
				OCR5A += 10;
				OCR5B += 10;
				info_ptr->pwmSpeed1 = OCR5B;
				info_ptr->pwmSpeed2 = OCR5A;
			}
			sprintf(buffer, "pwm: %d\n", info_ptr->pwmSpeed1);
			fputs(buffer, stdout);
			break;
		case '-' :
			/*	decrease motor speed	*/
			if ((OCR5A > 9) && (OCR5B > 9)){
				OCR4A -= 10;
				OCR4B -= 10;
				OCR5A -= 10;
				OCR5B -= 10;
				info_ptr->pwmSpeed1 = OCR5B;
				info_ptr->pwmSpeed2 = OCR5A;
			}
			sprintf(buffer, "pwm: %d\n", info_ptr->pwmSpeed1);
			fputs(buffer, stdout);
			break;
		case ' ' :
			OCR4A = 0;												/*	reset pwm's	*/
			OCR4B = 0;
			OCR5A = 0;
			OCR5B = 0;
			PORTA &= ~((1<<PA0)|(1<<PA1)|(1<<PA3)|(1<<PA4));		/*	stop motors from running	*/
			info_ptr->direction = 'N';								/*	stop checking if encoder value is reached	*/
			
			get_position(info_ptr);									/*	tell the system where it is using the current encoder values	*/
			
			info_ptr->x_01 = info_ptr->x_00;						/*	set next position as current position	*/
			info_ptr->y_01 = info_ptr->y_00;						/*	which is the same as telling the system not to move	*/
			info_ptr->x_11 = info_ptr->x_10;						/*		*/
			info_ptr->y_11 = info_ptr->y_10;						/*		*/
			info_ptr->doInputList = 0x00;							/*	stop reading from input list, serial only	*/
			info_ptr->doRoute = 0x00;
			break;
		case 'e' :
			/*	Move motor 1	*/
			
			break;
		case 'q' :
			/*	Move motor 2	*/
			
			break;
		case 'r':
			/*	route mode	*/
			info_ptr->nextInputIndex = 0;	
			info_ptr->doRoute = 0xFF;
			info_ptr->direction = 'N';
			fputs("doing a routine route\n", stdout);
			break;
		case 'R' :
			/*	reset variables to default or current values	*/
			info_ptr->doInputList = 0x00;
			info_ptr->doRoute = 0x00;
			info_ptr->changeInTrigger = 0;
			info_ptr->direction = 'N';
			info_ptr->nextInputIndex = 0;
			info_ptr->endStep1 = info_ptr->globalTrigger1;
			info_ptr->endStep2 = info_ptr->globalTrigger2;
			info_ptr->changePos = 28.0;
			break;
		case 'g' :
			/*	go and get inputs from list and just go for it	*/
			info_ptr->doInputList = 0xFF;
			info_ptr->direction = 'N';
			info_ptr->nextInputIndex = 0;
			fputs("going\n", stdout);
			break;
		case 'p' :
			count = 0;
			
			float currentCount = info_ptr->globalTrigger2;
			
			PORTA |= (1<<PA0)|(1<<PA4);
			PORTA &= ~((1<<PA1)|(1<<PA3));
			info_ptr->motorDir1 = 0x00;
			info_ptr->motorDir2 = 0x00;
			OCR5A = 80;
			OCR5B = 80;

			while ( currentCount+10 > info_ptr->globalTrigger2 ){
				if (serial_input_available()){
					char d = fgetc(stdin);
					if (d == ' '){
						count = 1;
						break;
					}
				}
				get_triggers(info_ptr);
				sprintf(buffer, "P: trig1: %f, trig2: %f\n", info_ptr->globalTrigger1, info_ptr->globalTrigger2);
				fputs(buffer, stdout);
				
			}
			PORTA &= ~((1<<PA1)|(1<<PA3)|(1<<PA0)|(1<<PA4));
			
			OCR5A = 0;
			OCR5B = 0;
			break;
		
		case 'm' :
			count = 0;

			PORTA |= (1<<PA3);
			PORTA &= ~(1<<PA4);
			info_ptr->motorDir2 = 0xFF;
		
			OCR5A = 100;
			OCR5B = 100;

			while ( 1 ){
				if (serial_input_available()){
					char d = fgetc(stdin);
					if (d == ' '){
						count = 1;
						break;
					}
				}
				
				sprintf(buffer, "P: trig1: %f, trig2: %f\n", info_ptr->globalTrigger1, info_ptr->globalTrigger2);
				fputs(buffer, stdout);
				
			}
			PORTA &= ~((1<<PA1)|(1<<PA3)|(1<<PA0)|(1<<PA4));
		
			OCR5A = 0;
			OCR5B = 0;
			break;
		
		case 'n' :
			count = 0;
			
			PORTA &= ~(1<<PA0);
			PORTA |= (1<<PA1);
			info_ptr->motorDir1 = 0xFF;
			
			OCR5A = 80;
			OCR5B = 80;

			while ( 1 ){
				if (serial_input_available()){
					char d = fgetc(stdin);
					if (d == ' '){
						count = 1;
						break;
					}
				}

				sprintf(buffer, "P: trig1: %f, trig2: %f\n", info_ptr->globalTrigger1, info_ptr->globalTrigger2);
				fputs(buffer, stdout);

			}
			PORTA &= ~((1<<PA1)|(1<<PA3)|(1<<PA0)|(1<<PA4));
			
			OCR5A = 0;
			OCR5B = 0;
			break;
		default:
			fputc('\n', stdout);
			fputc(c, stdout);
			fputs("wrong char\n", stdout);
			break;
	}
}


void calculate_route(info* info_ptr, uint8_t** route){
	/*	Calculates the position to move to next
	*	and actuates the motors in the correct direction	*/
	
	//info_ptr->x_00 = info_ptr->x_01;
	//info_ptr->y_00 = info_ptr->y_01;
	info_ptr->x_01 = (float)(route[0][info_ptr->nextInputIndex]);
	info_ptr->y_01 = (float)(route[1][info_ptr->nextInputIndex]);
	info_ptr->x_11 = info_ptr->width - info_ptr->x_01;
	info_ptr->y_11 = info_ptr->y_01;
	
	float change = pow(info_ptr->x_00 - info_ptr->x_01, 2) + pow(info_ptr->y_00 - info_ptr->y_01, 2);
	info_ptr->changePos = sqrt(change);

	++info_ptr->nextInputIndex;
	
	/*	get the current length of motor1 cable	*/
	info_ptr->l_00 = sqrt(pow(info_ptr->x_00, 2) + pow(info_ptr->y_00, 2));
	info_ptr->l_01 = sqrt(pow(info_ptr->x_01, 2) + pow(info_ptr->y_01, 2));
	/*	get the current length of motor2 cable	*/
	info_ptr->l_10 = sqrt(pow(info_ptr->x_10, 2) + pow(info_ptr->y_10, 2));
	info_ptr->l_11 = sqrt(pow(info_ptr->x_11, 2) + pow(info_ptr->y_11, 2));
	/*	get the change in length to be performed	*/
	float change1 = info_ptr->l_01 - info_ptr->l_00;
	float change2 = info_ptr->l_11 - info_ptr->l_10;
	char buffer[200];
	sprintf(buffer, "\n at: %f, %f		moving to: %f, %f\n", info_ptr->x_00, info_ptr->y_00, info_ptr->x_01, info_ptr->y_01);
	fputs(buffer, stdout);
	if (change1 < 0){
		/*	motor1 has to decrease in length	*/
		PORTA |= (1<<PA3);
		PORTA &= ~(1<<PA4);
		info_ptr->motorDir1 = 0xFF;
		info_ptr->direction = 'R';
		fputs("motor1 in\n", stdout);
	} else {
		/*	motor1 has to increase in length	*/
		PORTA |= (1<<PA4);
		PORTA &= ~(1<<PA3);
		info_ptr->motorDir1 = 0x00;
		info_ptr->direction = 'L';
		fputs("motor1 out\n", stdout);
	}
	if (change2 < 0){
		/*	motor2 has to decrease in length	*/
		PORTA |= (1<<PA1);
		PORTA &= ~(1<<PA0);
		info_ptr->motorDir2 = 0xFF;
		fputs("motor2 in\n", stdout);
	} else {
		/*	motor2 has to increase in length	*/
		PORTA |= (1<<PA0);
		PORTA &= ~(1<<PA1);
		info_ptr->motorDir2 = 0x00;
		fputs("motor2 out\n", stdout);
	}
	
	/*	check that direction is correct	*/
	if ((change1 < 0) && (change2 < 0) && (change1 < change2)){
		info_ptr->direction = 'L';
	} else if ((change1 < 0) && (change2 < 0) && (change1 > change2)){
		info_ptr->direction = 'R';	
	}

	/*	work out where to move to based on new coordinates	*/
	get_desired_triggers(info_ptr);
}


void get_desired_triggers(info* info_ptr){

	fputs("\ngetting desired triggers\n", stdout);	
	/*	get the current length of motor1 cable	*/	
	info_ptr->l_00 = sqrt(pow(info_ptr->x_00, 2) + pow(info_ptr->y_00, 2));
	info_ptr->l_01 = sqrt(pow(info_ptr->x_01, 2) + pow(info_ptr->y_01, 2));
	
	info_ptr->l_10 = sqrt(pow(info_ptr->x_10, 2) + pow(info_ptr->y_10, 2));
	info_ptr->l_11 = sqrt(pow(info_ptr->x_11, 2) + pow(info_ptr->y_11, 2));
	/*	get the change in length to be performed	*/
	float change1 = info_ptr->l_01 - info_ptr->l_00;
	
	//float ch_0 = sqrt(pow(info_ptr->width - info_ptr->x_1, 2) + pow(info_ptr->height - info_ptr->y_1, 2));
	//float ch_1 = sqrt(pow(info_ptr->width - info_ptr->x_0, 2) + pow(info_ptr->height - info_ptr->y_0, 2));
	float change2 =  info_ptr->l_11 - info_ptr->l_10;
	
	/*	Update end step to move to */
	info_ptr->endStep1 = info_ptr->globalTrigger1 + info_ptr->Kt * change1;
	info_ptr->endStep2 = info_ptr->globalTrigger2 + info_ptr->Kt * change2;
	
	char buffertoprint[300];
	sprintf(buffertoprint, "change1: %f, change2: %f, step: %f %f \n", change1, change2, info_ptr->endStep1, info_ptr->endStep2);
	fputs(buffertoprint, stdout);

	/*	Extra compensation for motor under the most load	*/
	float xRatio = info_ptr->x_11 / info_ptr->x_01;


	/*	set pwm based on lengths to move	*/

	if (fabsf(change1) < fabsf(change2)){
		/*	Motor 2 has more distance to cover, -> faster pwm	*/
		float ratio = fabsf(change2 / change1);
		char buph[100];
		sprintf(buph, "ratio %f\n", ratio);
		fputs(buph, stdout);
		
		uint8_t pwmMotorTwo;
		pwmMotorTwo = (uint8_t)(ratio * info_ptr->avePwm);
		OCR5A = pwmMotorTwo;
		OCR5B = info_ptr->avePwm;
		info_ptr->pwmSpeed2 = pwmMotorTwo;
		info_ptr->pwmSpeed1 = info_ptr->avePwm;
		
	} else if (fabsf(change1) > fabsf(change2)){
		float ratio = fabsf(change1 / change2);
		char buph[150];
		sprintf(buph, "ratio %f\n", ratio);
		fputs(buph, stdout);
		
		uint8_t pwmMotorOne;
		pwmMotorOne = (uint8_t)(ratio * info_ptr->avePwm);
		OCR5B = pwmMotorOne;
		OCR5A = info_ptr->avePwm;
		info_ptr->pwmSpeed1 = pwmMotorOne;
		info_ptr->pwmSpeed2 = info_ptr->avePwm;		
		
	} else {
		/*	equal speeds	*/
		OCR5A = info_ptr->avePwm;
		OCR5B = info_ptr->avePwm;
		info_ptr->pwmSpeed1 = info_ptr->avePwm;
		info_ptr->pwmSpeed1 = info_ptr->pwmSpeed2;
	}

	/*	set desired velocity	*/
	info_ptr->desVel1 = change1 / (2.0 * info_ptr->changePos / 28.0);
	info_ptr->desVel2 = change2 / (2.0 * info_ptr->changePos / 28.0);
	info_ptr->startTime = get_tcnt0_ticks();
	info_ptr->startStep1 = info_ptr->globalTrigger1;
	info_ptr->startStep2 = info_ptr->globalTrigger2;
		
	/*	ensure the velocities are above 10 pwm	*/
	if ((info_ptr->pwmSpeed2 < 10) || (info_ptr->pwmSpeed1 < 10)){
			
		OCR5A += 20;
		OCR5B += 20;
		info_ptr->pwmSpeed1 = OCR5B;
		info_ptr->pwmSpeed2 = OCR5A;
	}
	
	if (xRatio > 1.05){
		/*	motor more over the motor1 side, as it's length of x coord is smaller	*/
		uint16_t addedSpeed = (uint16_t)(xRatio * 10);
		if ((info_ptr->pwmSpeed1 + addedSpeed) < 250){
			info_ptr->pwmSpeed1 += addedSpeed;
			OCR5B += addedSpeed;
		} else {
			OCR5B = 200;
			info_ptr->pwmSpeed1 = 200;
		}
		char xbuff[100];
		sprintf(xbuff, "increased pwm: %d	%d\n", info_ptr->pwmSpeed1, info_ptr->pwmSpeed2);
		fputs(xbuff, stdout);

		
	} else if (xRatio < 0.95){
		
		uint16_t addedSpeed = (uint16_t)((1.0 / xRatio) * 10);
		if ((info_ptr->pwmSpeed2 + addedSpeed) < 250){
			info_ptr->pwmSpeed2 += addedSpeed;
			OCR5A += addedSpeed;
		} else {
			OCR5A = 200;
			info_ptr->pwmSpeed2 = 200;
		}
		
		char xbuff[100];
		sprintf(xbuff, "increased pwm: %d	%d\n", info_ptr->pwmSpeed1, info_ptr->pwmSpeed2);
		fputs(xbuff, stdout);
	}
	char buff[150];
	sprintf(buff, "---- speed set %d %d\n", OCR5A, OCR5B);
	fputs(buff, stdout);
	
	char buff2[200];
	sprintf(buff2, "max1: %f, trig1: %f, max2: %f trig2: %f\n", info_ptr->endStep1, info_ptr->globalTrigger1, info_ptr->endStep2, info_ptr->globalTrigger2);
	fputs(buff2, stdout);
	
}



void check_movement(info* info_ptr){
	
	/*	Checks if the desired encoder value has been reached
	*	if it has, stop the motor and quicken up the other	*/
	
	/*	Check motor 1's position	*/
	if ( ((info_ptr->motorDir1) && (info_ptr->globalTrigger1 <= info_ptr->endStep1)) 
			|| ((!info_ptr->motorDir1) && (info_ptr->globalTrigger1 >= info_ptr->endStep1)) ){
		if (info_ptr->pwmSpeed1 != 0){
			OCR5B = 0;
			PORTA &= ~((1<<PA3)|(1<<PA4));
			info_ptr->pwmSpeed1 = 0;
			info_ptr->changeInTrigger = info_ptr->globalTrigger2;	
		}
	}
	
	/*	Check motor 2's position	*/
	if ( ((info_ptr->motorDir2) && (info_ptr->globalTrigger2 <= info_ptr->endStep2))
			|| ((!info_ptr->motorDir2) && (info_ptr->globalTrigger2 >= info_ptr->endStep2)) ){
		if(info_ptr->pwmSpeed2 != 0){
			/*	The motor has reached it targer and needs to stop	*/
			OCR5A = 0;
			PORTA &= ~((1<<PA0)|(1<<PA1));
			info_ptr->changeInTrigger = info_ptr->globalTrigger1;
			info_ptr->pwmSpeed2 = 0;
		}
	}
	/*	Check if both have reached their target, if so, end the command sequence and update position	*/
	if ( ( ((info_ptr->motorDir1) && (info_ptr->globalTrigger1 <= info_ptr->endStep1))
			|| ((!info_ptr->motorDir1) && (info_ptr->globalTrigger1 >= info_ptr->endStep1)) ) 
			&& ( ((info_ptr->motorDir2) && (info_ptr->globalTrigger2 <= info_ptr->endStep2))
			|| ((!info_ptr->motorDir2) && (info_ptr->globalTrigger2 >= info_ptr->endStep2)) )){
		
		OCR5A = 0;
		OCR5B = 0;
		PORTA &= ~((1<<PA3)|(1<<PA4)|(1<<PA0)|(1<<PA1));
		info_ptr->direction = 'N';
		info_ptr->pwmSpeed1 = 0;
		info_ptr->pwmSpeed2 = 0;
		get_position(info_ptr);
	}
	
	/*	Check that motor2 has enough speed to reach its 
	*	position as motor 1 has already stopped	*/
	if ((info_ptr->pwmSpeed1 == 0) && (info_ptr->pwmSpeed2 < 70)){
		OCR5A = 100;
		info_ptr->pwmSpeed2 = 70;		
		if (info_ptr->direction == 'L'){
			PORTA |= (1<<PA1);
			PORTA &= ~(1<<PA0);
		} else if (info_ptr->direction == 'R'){
			PORTA |= (1<<PA0);
			PORTA &= ~(1<<PA1);
			
		}
		fputs("OCR5A is 100\n", stdout);
		char buff[200];
		sprintf(buff, "max1: %f, trig1: %f, max2: %f trig2: %f\n", info_ptr->endStep1, info_ptr->globalTrigger1, info_ptr->endStep2, info_ptr->globalTrigger2);
		fputs(buff, stdout);
	
	/*	Check that motor2 has enough speed to reach its 
	*	position as motor 1 has already stopped	*/	
	} else if ((info_ptr->pwmSpeed2 == 0) && (info_ptr->pwmSpeed1 < 70)){
		OCR5B = 100;
		info_ptr->pwmSpeed1 = 70;
		if (info_ptr->direction == 'L'){
			PORTA |= (1<<PA4);
			PORTA &= ~(1<<PA3);
		} else if (info_ptr->direction == 'R'){
			PORTA |= (1<PA1);
			PORTA &= ~(1<<PA0);
		}
		fputs("OCR5B is 100\n", stdout);
		char buff[200];
		sprintf(buff, "max1: %f, trig1: %f, max2: %f trig2: %f\n", info_ptr->endStep1, info_ptr->globalTrigger1, info_ptr->endStep2, info_ptr->globalTrigger2);
		fputs(buff, stdout);
	}
}


void pwm_control(info* info_ptr){
	/*	Controls the speed of the motors to ensure they aren't falling behind.	
	*	Delay before running this loop. The proportionality between velocity error and pwm,
	*	and the resolution of the encoders (which is due to change)
	*	are distinctive properties which may affect the accuracy of this process */
	
	if (info_ptr->startTime + 600 < get_tcnt0_ticks()){
		char buff1[120];
		char buff2[120];
		int checkMotor1 = 0x00;
		int checkMotor2 = 0x00;
		if (info_ptr->pwmSpeed1 == 0){ checkMotor1 = 0xFF; }
		if (info_ptr->pwmSpeed2 == 0){ checkMotor2 = 0xFF; }
		
		uint32_t ticks = get_tcnt0_ticks() - info_ptr->startTime;
		float timeElapsed = (float)(ticks);
		timeElapsed = timeElapsed / 1000.0;
		float currentVel1 = (info_ptr->globalTrigger1 - info_ptr->startStep1) / (timeElapsed * info_ptr->Kt);
		float currentVel2 = (info_ptr->globalTrigger2 - info_ptr->startStep2) / (timeElapsed * info_ptr->Kt);
	
		float errorVel1 = info_ptr->desVel1 - currentVel1;
		float errorVel2 = info_ptr->desVel2 - currentVel2;
		sprintf(buff1, "t: %ld %f	v: %f %f	errorVel:	%f	%f\n", ticks, timeElapsed, info_ptr->desVel1, currentVel1, errorVel1, errorVel2);
		
		/*	calculate pwm = current + P * error	*/
		if (errorVel1 >= 0){
			info_ptr->pwmSpeed1 -= (uint8_t)(fabsf(errorVel1) * 1.8);
		} else {
			info_ptr->pwmSpeed1 += (uint8_t)(fabsf(errorVel1) * 1.8);
		}
		if (errorVel2 >= 0){
			info_ptr->pwmSpeed2 -= (uint8_t)(fabsf(errorVel2) * 1.8);
		} else {
			info_ptr->pwmSpeed2 += (uint8_t)(fabsf(errorVel2) * 1.8);
		}
		
		sprintf(buff2, "calculated pwm:	%d	%d \n", info_ptr->pwmSpeed1, info_ptr->pwmSpeed2);
	
		/*	Set limits	*/
		if (info_ptr->pwmSpeed1 > 250){
			info_ptr->pwmSpeed1 = 250;
		}
		if (info_ptr->pwmSpeed2 > 250){
			info_ptr->pwmSpeed2 = 250;
		}
		if (info_ptr->pwmSpeed1 < 1){
			info_ptr->pwmSpeed1 = 0;
		}
		if (info_ptr->pwmSpeed2 < 1){
			info_ptr->pwmSpeed2 = 0;
		}
	
		/*	set pwm	*/
		if (!checkMotor2){	OCR5A = info_ptr->pwmSpeed2;}
		if (!checkMotor1){	OCR5B = info_ptr->pwmSpeed1; }
		info_ptr->startTime = get_tcnt0_ticks();
		info_ptr->startStep1 = info_ptr->globalTrigger1;
		info_ptr->startStep2 = info_ptr->globalTrigger2;
		fputs(buff1, stdout);
		fputs(buff2, stdout);
		fputc('\n', stdout);
	}
	
}




void adjust_position(info* info_ptr){
	/*	moves one motor at a time until trigger clicked	
	*	used for ensuring each motor starts from an encoder trigger */
	
	fputs("adjusting\n", stdout);
	
	float desired1 = info_ptr->globalTrigger1 - 1;
	float desired2 = info_ptr->globalTrigger2 - 1;
	
	OCR5A = 50;
	OCR5B = 50;
	PORTA &= ~((1<<PA0)|(1<<PA4));
	PORTA |= (1<<PA1)|(1<<PA3);
	
	while ((info_ptr->globalTrigger1 > desired1) || (info_ptr->globalTrigger2 > desired2)){
		
		if (info_ptr->globalTrigger2 <= desired2){
			OCR5A = 0x00;
			PORTA &= ~((1<<PA1)|(1<<PA0));	
		}
		
		if (info_ptr->globalTrigger1 <= desired1) {
			OCR5B = 0x00;
			PORTA &= ~((1<<PA3)|(1<<PA4));
		}
		
		info_ptr->motorDir1 = 0xFF;
		info_ptr->motorDir2 = 0xFF;
		info_ptr->direction = 'U';
		get_triggers(info_ptr);
		char buffer[100];
		sprintf(buffer, "trig1: %f, trig2: %f\n", info_ptr->globalTrigger1, info_ptr->globalTrigger2);
		fputs(buffer, stdout);
	}
	
	OCR5A = 0x00;
	OCR5B = 0x00;
	PORTA &= ~((1<<PA0)|(1<<PA1)|(1<<PA3)|(1<<PA4));
	--info_ptr->globalTrigger1;
	--info_ptr->globalTrigger2;
	info_ptr->direction = 'N';
	fputs("ready\n", stdout);
}

void get_position(info* info_ptr){
	/*	Calculates the position of each motor in their Cartesian coordinates
	*	using the current encoder counts and inverse kinematics	*/
	
	fputc('\n', stdout);
	
	/*	get lengths from encoder values	*/
	float l1 = info_ptr->globalTrigger1 / info_ptr->Kt;
	float l2 = info_ptr->globalTrigger2 / info_ptr->Kt;
	
	char buff[100];
	sprintf(buff, "\n lengths: l1: %f, l2: %f \n", l1, l2);
	fputs(buff, stdout);
	
	float trigCoeff = (powf(l2, 2) - powf(info_ptr->width, 2) - powf(l1, 2)) / (-2 * info_ptr->width * l1);
	char trigBuff[100];
	sprintf(trigBuff, "\n trigCoeff: %f\n", trigCoeff);
	fputs(trigBuff, stdout);
	
	/*	Angle of incidence	*/
	float alpha = acosf(trigCoeff);
	
	char buff1[100];
	sprintf(buff1, "\n alpha: %f\n", alpha);
	fputs(buff1, stdout);
	
	/*	using the angle and current lengths, determine the x,y position	*/
	info_ptr->x_00 = l1 * fabsf(cosf(alpha));
	info_ptr->y_00 = l1 * fabsf(sinf(alpha));
	
	info_ptr->x_10 = info_ptr->width - info_ptr->x_00;
	info_ptr->y_10 = info_ptr->y_00;

	char buffer[300];
	sprintf(buffer, "got pos to be: %f %f %f %f\n", info_ptr->x_00, info_ptr->y_00, info_ptr->x_10, info_ptr->y_10);
	fputs(buffer, stdout);

}



void custom_delay(uint32_t ticks){
	/*	Custom delay function, waits for timer to change by 'ticks'
	*	note that tcnt1 updates every 0.002 seconds */
	
	uint32_t current_time;
	current_time = get_tcnt0_ticks();
	while((current_time + ticks) > get_tcnt0_ticks()){
		;
	}
}

