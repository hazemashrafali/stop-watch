/*
 * stopwatch.c
 *
 *  Created on: Jan 18, 2022
 *      Author: HAZEM-PC
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "util/delay.h"

#define SEC1 sec%10
#define SEC2 sec/10
#define MIN1 min%10
#define MIN2 min/10
#define HOUR1 hour%10
#define HOUR2 hour/10
#define setport(NUM)   ((PORTC & 0xF0) | NUM)

volatile unsigned char sec=0;
volatile unsigned char min=0;
volatile unsigned char hour=0;


void TIMER1_CTC_inti (void)
{
		TCNT1=0;										//step 1- set timer initial value
		OCR1A=15625; 	 	 	 	 	 	 			//step 2- set timer compare value
		TIMSK |= (1<<OCIE1A); 	 	 					//step 3- enable timer1 compare interrupt
		TCCR1A = (1<<FOC1A);							//step 4- configure timer control register TCCR1A & TCCR1B
		TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS11);	//Select pre scaler=F_CPU/64 and set mode to CTC mode4
}

ISR(TIMER1_COMPA_vect)
{
	sec++;
	if(sec==60)
	{
		min++;
		sec=0;
	}
	if(min==60)
	{
		hour++;
		min=0;
	}
	if(hour==24)
	{
		hour=0;
	}
}

ISR(INT0_vect)  //stop watch reset
{
	sec = 0;
	min = 0;
	hour = 0;
}

ISR(INT1_vect)  //stop watch paused
{
	TCCR1B =0;
}

ISR(INT2_vect) //stop watch resume
{
	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS11);
}

void SEG_inti (void)
{
	DDRC |= (1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC3);       					 		//7segment BCD
	DDRA |= (1<<PA0) | (1<<PA1) | (1<<PA2) | (1<<PA3) | (1<< PA4) | (1<<PA5); 			//7segments enable pins
	PORTC &= ~(1<<PC0) & ~(1<<PC1) & ~(1<<PC2) & ~(1<<PC3);   							//clear 7 segments digits
	PORTA &= ~(1<<PA0) & ~(1<<PA1) & ~(1<<PA2) & ~(1<<PA3) & ~(1<< PA4) & ~(1<<PA5);    //disable 7 segments
}

void INT0_init(void)
{
	DDRD &= ~(1<<PD2);
	PORTD |= (1<<PD2);
	MCUCR |= (1<<ISC01);
	GICR |= (1<<INT0);
}

void INT1_init(void)
{
	DDRD &= ~(1<<PD3);
	MCUCR |= (1<<ISC11) | (1<<ISC10);
	GICR |= (1<<INT1);
}

void INT2_init(void)
{
	DDRB &= ~(1<<PB2);
	PORTB |= (1<<PB2);
	GICR |= (1<<INT2);
}

int main(void)
{
	SEG_inti();
	SREG |= (1<<7);
	TIMER1_CTC_inti();
	INT0_init();
	INT1_init();
	INT2_init();
	while(1)
	{
		PORTA = (1<<0);
		PORTC = setport(SEC1);
		_delay_us(50);
		PORTA = (1<<1);
		PORTC = setport(SEC2);
		_delay_us(50);
		PORTA = (1<<2);
		PORTC = setport(MIN1);
		_delay_us(50);
		PORTA = (1<<3);
		PORTC = setport(MIN2);
		_delay_us(50);
		PORTA = (1<<4);
		PORTC = setport(HOUR1);
		_delay_us(50);
		PORTA = (1<<5);
		PORTC = setport(HOUR2);
		_delay_us(50);
	}
}
