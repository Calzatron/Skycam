/*
 * trigger.c
 *
 * Created: 18/01/2018 3:51:02 PM
 *  Author: Owner
 */ 
/*	standard libraries	*/
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "Project.h"
#include "tcnt0.h"

// counter incrementing on trigger (interrupt pin)
static volatile float trig1;
static volatile float trig2;
static volatile float trig3;
static volatile float trig4;
static volatile uint32_t timer;
static volatile uint32_t endTime1;
static volatile uint32_t endTime2;
static volatile uint32_t endTime3;
static volatile uint32_t endTime4;

void reset_trig(void){
	/*	reset the interrupt counter for each button		*/
	trig1 = 0;
	trig2 = 0;
	trig3 = 0;
	trig4 = 0;
}


void init_triggers(void){
	/* Initializes the interrupt pins and registers for the rotary encoder
	*  There are 8 
	* set PD2 and PD3 as input */
	DDRD &= ~((1 << PD0)|(1 << PD1)|(1 << PD2)|(1 << PD3));				/* PD2 and PD3 as input */
	DDRE &= ~((1 << PE4)|(1 << PE5)|(1 << PE6)|(1 << PE7));
	PORTD &= ~((1 << PD0)|(1 << PD1)|(1 << PD2)|(1 << PD3));   /* PD2 and PD3 pull-up enabled   */
	PORTE &= ~((1 << PE4)|(1 << PE5)|(1 << PE6)|(1 << PE7));   /* PE4 and PE5 pull-up enabled   */
	
	EIMSK |= (1<<INT0)|(1<<INT1)|(1<<INT2)|(1<<INT3);		/* enable INT Pins */
	
	EICRA |= (1<<ISC01);	/*	Falling edge	*/				//|(1<<ISC11)|(1<<ISC10); /* INT0 - falling edge, INT1 - rising edge */
	EICRA |= (1<<ISC11);										//|(1<<ISC31)|(1<<ISC30); /* INT2 - falling edge, INT3 - rising edge */
	EICRA |= (1<<ISC21);										//|(1<<ISC11)|(1<<ISC10); /* INT0 - falling edge, INT1 - rising edge */
	EICRA |= (1<<ISC31);										//|(1<<ISC31)|(1<<ISC30); /* INT2 - falling edge, INT3 - rising edge */
	
	reset_trig();
	
	timer =	50;													//190;
	
	endTime1 = 0;
	endTime2 = 0;
	endTime3 = 0;
	endTime4 = 0;
	
	
}




void get_triggers(info* info_ptr) {
	/* internal reference clock, times how long the system has been on for	*/
	uint8_t interrupts_on = bit_is_set(SREG, SREG_I);
	cli();
	
	/*	Update the global position variables for each motor with the clocked # of interrupts	*/
	if (info_ptr->motorDir1){ info_ptr->globalTrigger1 -= trig1; }
	else {info_ptr->globalTrigger1 += trig1; }
	
	if (info_ptr->motorDir2){ info_ptr->globalTrigger2 -= trig2; }
	else {info_ptr->globalTrigger2 += trig2; }
		
	if (info_ptr->motorDir3){ info_ptr->globalTrigger3 -= trig3; }
	else {info_ptr->globalTrigger3 += trig3; }
	
	if (info_ptr->motorDir4){ info_ptr->globalTrigger4 -= trig4; }
	else {info_ptr->globalTrigger4 += trig4; }

	reset_trig();

	if(interrupts_on) {
		sei();
	}
}


ISR(INT0_vect)
{
	if (endTime1 < get_tcnt0_ticks()){
		++trig1;
		endTime1 = get_tcnt0_ticks() + timer;
	}
	
}
ISR(INT1_vect)
{
	// Falling edge
	if (endTime2 < get_tcnt0_ticks()){
		++trig2;
		endTime2 = get_tcnt0_ticks() + timer;
	}
}
ISR(INT2_vect)
{
	// Falling edge
	++trig3;
}
ISR(INT3_vect)
{
	// Falling edge
	++trig4;
}