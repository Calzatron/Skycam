/*
 * enco.c
 *
 * Created: 17/01/2018 10:34:55 AM
 *  Author: Owner
 */ 
/*	standard libraries	*/
#include <avr/io.h>
#include <avr/interrupt.h>			//enables interrupts
#include <avr/pgmspace.h>			//for storing / accessing data in flash program instead of SRAM -> variable manipulation
#include "Project.h"

volatile int16_t enc;
volatile uint8_t encoder_state;
volatile uint8_t encoder_buffer[4];

// Sets INT0 & INT1 to fire interrupts on falling edge.
void INT01_init() {
	DDRD &= ~((1 << PD0)|(1 << PD1)|(1 << PD2)|(1 << PD3));				/* PD2 and PD3 as input */
	DDRE &= ~((1 << PE4)|(1 << PE5)|(1 << PE6)|(1 << PE7));
	
	EICRA|=((1<<ISC11)|(1<<ISC01)); // Trigger INT0 & INT1 on falling edge
	EIMSK|=((1<<INT1)|(1<<INT0));   // Enable INT0 & INT1
	
	enc = 0;
	encoder_state=0;
	encoder_buffer[0] = 'N';
	encoder_buffer[1] = 'N';
	encoder_buffer[2] = 'N';
	encoder_buffer[3] = 'N';
}

void init_enc(void){
	enc = 0;
}

void get_encoder_pos(info* info_ptr) {
	/* internal reference clock, times how long the system has been on for	*/
	uint8_t interrupts_on = bit_is_set(SREG, SREG_I);
	cli();
	info_ptr->encoder1 = enc;

	if(interrupts_on) {
		sei();
	}
}

ISR(INT0_vect)
{
	EIFR|=(1<<INTF0); // Clear INT0 flag
	encoder_state++;
	
	switch(encoder_state)
	{
		case 1: EICRA|=(1<<ISC00); // Trig. INT0 rising edge
		encoder_buffer[encoder_state]='R';
		break;
		case 2: EICRA|=(1<<ISC00); // Trig. INT0 rising edge
		encoder_buffer[encoder_state]='L';
		break;
		case 3: EICRA&=~(1<<ISC00); // Trig. INT0 falling edge
		encoder_buffer[encoder_state]='R';
		break;
		case 4: EICRA&=~(1<<ISC00); // Trig. INT0 falling edge
		encoder_state=0;
		encoder_buffer[encoder_state]='L';

		if((encoder_buffer[0]=='L')&&(encoder_buffer[1]=='L')&&(encoder_buffer[2]=='L')&&(encoder_buffer[3]=='L')){ --enc;}
	}
}

ISR(INT1_vect)
{
	EIFR|=(1<<INTF1); // Clear INT1 flag
	encoder_state++;
	
	switch(encoder_state)
	{
		case 1: EICRA|=(1<<ISC10); // Trig. INT1 rising edge
		encoder_buffer[encoder_state]='L';
		break;
		case 2: EICRA|=(1<<ISC10); // Trig. INT1 rising edge
		encoder_buffer[encoder_state]='R';
		break;
		case 3: EICRA&=~(1<<ISC10); // Trig. INT1 falling edge
		encoder_buffer[encoder_state]='L';
		break;
		case 4: EICRA&=~(1<<ISC10); // Trig. INT1 falling edge
		encoder_state=0;
		encoder_buffer[encoder_state]='R';

		if((encoder_buffer[0]=='R')&&(encoder_buffer[1]=='R')&&(encoder_buffer[2]=='R')&&(encoder_buffer[3]=='R')){ ++enc;}
	}
}