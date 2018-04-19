/*
 * tcnt0.c
 *
 * Created: 17/04/2017 6:43:58 PM
 *  Author: s4357594
 */ 

 #include <avr/io.h>
 #include <avr/interrupt.h>			//enables interrupts
 #include <avr/pgmspace.h>			//for storing / accessing data in flash program instead of SRAM -> variable manipulation
 #include <stdio.h>					//input / output
 #include <stdlib.h>					//standard function library; malloc / sort / rand etc
 //#include <util/delay.h>
 #include "tcnt0.h"

 // counter incrementing on OCRA
 static volatile uint32_t tcnt0_ticks;
//static volatile uint8_t check;

 void init_tcnt0(void){
	 
	 /* Output compare register 0A is on PB7 */
	 DDRB &= ~(1 << PB7);
	 
	 /* initialises 8-bit timer to output compare at f(clk)/64 and OCRA @ 124 */
	 // set global counter
	 tcnt0_ticks = 0L;
	 //check = 0;
	 // set timer / counter
	 TCNT0 = 0;
	 // set output compare resgister; value to execute ISR
	 //OCR0A = 123;// 124;			// good with +=1 and mod 8 with div 8
	 OCR0A = 123;
	 // timer/counter control register A - set to clear when TCNT0 matches OCR0A
	 TCCR0A = (1<<WGM01);
	 // set the timer to update at a fraction of a clock cycle
	 //TCCR0B = (1<<CS01);// div 8      |(1<<CS00);	// set to f(clk) / 64
	 TCCR0B = (1<<CS01)|(1<<CS00);
	 // timer/counter 1 interrupt mask register - enable OCR0A as output compare register, only works if OCF0A is set in TIFR0
	 TIMSK0 |= (1<<OCIE0A);
	 // if it isn't already, clear the interrupt output compare flag by writing a 1 to OCF0A; switches when TCNT0 matches OCR0A
	 TIFR0 &= (1<<OCF0A);
 }


 uint32_t get_tcnt0_ticks(void) {
	 /* internal reference clock, times how long the system has been on for	*/
	 uint8_t interrupts_on = bit_is_set(SREG, SREG_I);
	 cli();
	 uint32_t return_value = tcnt0_ticks;
	 if(interrupts_on) {
		 sei();
	 }
	 return return_value;
 }


 ISR(TIMER0_COMPA_vect) {
	 /* Increment our clock tick count, check if pin value has changed */
	 //if (! (check % 8)){
	//if (!(check % 2)){	 
	tcnt0_ticks += 1;
		//check = 0;
	 //}
	 //++check;
 }


