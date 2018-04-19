#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "Project.h"


// counter incrementing on OCRA
static volatile int16_t enc1;
static volatile int16_t enc2;
static volatile int16_t enc3;
static volatile int16_t enc4;

void init_encoder(void){
	/* Initializes the interrupt pins and registers for the rotary encoder
	*  There are 8 
	* set PD2 and PD3 as input */
	DDRD &= ~((1 << PD0)|(1 << PD1)|(1 << PD2)|(1 << PD3));				/* PD2 and PD3 as input */
	DDRE &= ~((1 << PE4)|(1 << PE5)|(1 << PE6)|(1 << PE7));
	PORTD |= (1 << PD0)|(1 << PD1)|(1<<PD2)|(1<<PD3);   /* PD2 and PD3 pull-up enabled   */
	PORTE |= (1 << PE4)|(1 << PE5)|(1<<PE6)|(1<<PE7);   /* PE4 and PE5 pull-up enabled   */
	
	EIMSK |= (1<<INT0)|(1<<INT1)|(1<<INT2)|(1<<INT3)|(1<<INT4)|(1<<INT5)|(1<<INT6)|(1<<INT7);		/* enable INT Pins */
	
	EICRA |= (1<<ISC01)|(1<<ISC11)|(1<<ISC10); /* INT0 - falling edge, INT1 - rising edge */
	EICRA |= (1<<ISC21)|(1<<ISC31)|(1<<ISC30); /* INT2 - falling edge, INT3 - rising edge */
	EICRA |= (1<<ISC41)|(1<<ISC51)|(1<<ISC50); /* INT4 - falling edge, INT5 - rising edge */
	EICRA |= (1<<ISC61)|(1<<ISC71)|(1<<ISC70); /* INT6 - falling edge, INT7 - rising edge */
	
	
	enc1 = 0;
	enc2 = 0;
	enc3 = 0;
	enc4 = 0;
	
}


void init_enc(void){
	enc1 = 0;
	enc2 = 0;
	enc3 = 0;
	enc4 = 0;
}


void get_encoder_pos(info* info_ptr) {
	 /* internal reference clock, times how long the system has been on for	*/
	 uint8_t interrupts_on = bit_is_set(SREG, SREG_I);
	 cli();
	 info_ptr->encoder1 = enc1;
	 info_ptr->encoder2 = enc2;
	 info_ptr->encoder3 = enc3;
	 info_ptr->encoder4 = enc4;
	 if(interrupts_on) {
		 sei();
	 } 
 }
//
//void get_encoder2_pos(info* info_ptr) {
	///* internal reference clock, times how long the system has been on for	*/
	//uint8_t interrupts_on = bit_is_set(SREG, SREG_I);
	//cli();
	//info_ptr->encoder2 = enc2;
	//if(interrupts_on) {
		//sei();
	//}
//}
//void get_encoder3_pos(info* info_ptr) {
	///* internal reference clock, times how long the system has been on for	*/
	//uint8_t interrupts_on = bit_is_set(SREG, SREG_I);
	//cli();
	//info_ptr->encoder3 = enc3;
	//if(interrupts_on) {
		//sei();
	//}
//}
//void get_encoder4_pos(info* info_ptr) {
	///* internal reference clock, times how long the system has been on for	*/
	//uint8_t interrupts_on = bit_is_set(SREG, SREG_I);
	//cli();
	//info_ptr->encoder4 = enc4;
	//if(interrupts_on) {
		//sei();
	//}
//}


ISR(INT0_vect)
{
	// Falling edge
	uint8_t temp = PIND & (1<<PD1);		//ignore all but 1st switch
	if(temp != 0x00){
		// D1 high, CW
		++enc1;
	}
	else{
		// D1 low, CCW
		--enc1;
	}
	
}

//INT1 interrupt
ISR(INT1_vect)
{
	// Rising edge
	uint8_t temp = PIND & (1<<PD0);		//ignore all but 1st switch
	
	if(temp != 0x00){
		// D0 high, CW
		++enc1;
	}
	
	else{
		// CCW
		--enc1;
	}
}
//
//ISR(INT2_vect)
//{
	//// falling edge on PIN2, check state of PIN3 to determine rotation
	//if(!bit_is_clear(PIND, PD3))
	//{
		//// clockwise
		//++enc2;
	//}
	//else
	//{
		//// anti-clockwise
		//--enc2;
	//}
//}
//
////INT1 interrupt
//ISR(INT3_vect)
//{
	//if(!bit_is_clear(PIND, PD2))
	//{
		//++enc2;
	//}
	//else
	//{
		//--enc2;
	//}
//}
//
//ISR(INT4_vect)
//{
	//// falling edge on PIN2, check state of PIN3 to determine rotation
	//if(!bit_is_clear(PINE, PE5))
	//{
		//// clockwise
		//++enc3;
	//}
	//else
	//{
		//// anti-clockwise
		//--enc3;
	//}
//}
//
////INT1 interrupt
//ISR(INT5_vect)
//{
	//if(!bit_is_clear(PINE, PE4))
	//{
		//++enc3;
	//}
	//else
	//{
		//--enc3;
	//}
//}
//
//ISR(INT6_vect)
//{
	//// falling edge on PIN2, check state of PIN3 to determine rotation
	//if(!bit_is_clear(PINE, PE7))
	//{
		//// clockwise
		//++enc4;
	//}
	//else
	//{
		//// anti-clockwise
		//--enc4;
	//}
//}
//
////INT1 interrupt
//ISR(INT7_vect)
//{
	//if(!bit_is_clear(PINE, PE6))
	//{
		//++enc4;
	//}
	//else
	//{
		//--enc4;
	//}
//}