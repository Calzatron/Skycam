/*
 * CFile1.c
 *
 * Created: 17/04/2017 6:45:21 PM
 *  Author: s4357594
 */ 
 
 #include <avr/io.h>
 #include <avr/interrupt.h>			//enables interrupts
 #include <avr/pgmspace.h>			//for storing / accessing data in flash program instead of SRAM -> variable manipulation
 #include <stdio.h>					//input / output
 #include <stdlib.h>				//standard function library; malloc / sort / rand etc
#include "tcnt4.h"
	

void init_tcnt4(void){ 
   /*
   The below settings are for
   Timer Clock = CPU Clock (No Prescalling)
   Mode        = Fast PWM
   PWM Output  = Non Inverted

   */
   
   DDRH |= (1<<PH3)|(1<<PH4);		//OCR4A and OCR4B on pins 6 and 7, EN on pin 9
   DDRA |= (1<<PA0)|(1<<PA1);
	// set timer / counter
	//Set Initial Timer value
	TCNT4 = 0;
	
	OCR4A = 0;
	OCR4B = 0;
	
	//set top value to ICR1
	ICR4 = 0x00FF;
	
	TCCR4A |= (1<<COM4A1)|(1<<COM4B1);		// non-inverted PWM on OC3A and OC3B
	TCCR4B |= _BV(WGM43);					// PWM, Phase and Freq Correct, ICRn Bottom
	TCCR4B |= _BV(CS40)|_BV(CS41);			// clk/64
}
