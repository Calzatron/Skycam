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
#include "tcnt5.h"
	

void init_tcnt5(void){ 
   /*
   The below settings are for
   Timer Clock = CPU Clock (No Prescalling)
   Mode        = Fast PWM
   PWM Output  = Non Inverted
	*/
   
	DDRL |= (1<<PL3)|(1<<PL4);			//OC5A and OC5B, Pin 46 and 35
	DDRA |= (1<<PA3)|(1<<PA4);
	// set timer / counter
	//Set Initial Timer value
	TCNT5 = 0;
	
	OCR5A = 0;
	OCR5B = 0;
	
	//set top value to ICR1
	ICR5 = 0x00FF;
	
	TCCR5A |= (1<<COM5A1)|(1<<COM5B1);		// non-inverted PWM on OC3A and OC3B
	TCCR5B |= _BV(WGM53);					// PWM, Phase and Freq Correct, ICRn Bottom
	TCCR5B |= _BV(CS50)|_BV(CS51);			// clk/64
}
