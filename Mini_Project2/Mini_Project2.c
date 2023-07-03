/*
 * Mini_Project2.c
 *
 *  Created on: Sep 15, 2022
 *      Author: Clara Isaac
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/******** Types Definitions ********/

/* Creating a Structure to hold the time */
typedef struct{
	unsigned char seconds;
	unsigned char minutes;
	unsigned char hours;
} TIME;

/***** Variable Initialization *****/

TIME timer = {0, 0, 0};

/****** Functions Prototypes *******/

void Display_Time(void); // display the time on the 7-segments
void Check_Time(void); // check the seconds and minutes are correct (from 0 to 59)

/**** Interrupt Service Routine ****/

/* ISR of Interrupt 0 */
ISR(INT0_vect)
{
	/* Reset */
	TCNT1 = 0;
	timer.seconds = 0;
	timer.minutes = 0;
	timer.hours = 0;
}

/* ISR of Interrupt 1 */
ISR(INT1_vect)
{
	/* Pause */
	TCCR1B = 0;
}

/* ISR of Interrupt 2 */
ISR(INT2_vect)
{
	/* Resume */
	TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10);
}

/* ISR of Timer1 Compare Mode */
ISR(TIMER1_COMPA_vect)
{
	timer.seconds++;
}

/***** Interrupts Definitions ******/

/* Interrupt 0 Definition */
void INT0_init(void)
{
	DDRD &= ~(1<<PD2); /* Configure INT0/PD2 as input pin */
	PORTD |= (1<<PD2); /* Activate internal pull-up resistor */
	MCUCR |= (1<<ISC01); /* Trigger INT0 with the falling edge */
	GICR |= (1<<INT0); /* Enable external interrupt pin INT0 */
}

/* Interrupt 1 Definition */
void INT1_init(void)
{
	DDRD &= ~(1<<PD3); /* Configure INT1/PD3 as input pin */
	MCUCR |= (1<<ISC10) | (1<<ISC11); /* Trigger INT1 with the rising edge */
	GICR |= (1<<INT1); /* Enable external interrupt pin INT1 */
}

/* Interrupt 2 Definition */
void INT2_init(void)
{
	DDRB &= ~(1<<PB2); /* Configure INT2/PB2 as input pin */
	PORTB |= (1<<PB2); /* Activate internal pull-up resistor */
	MCUCSR &= ~(1<<ISC2); /* Trigger INT2 with the falling edge */
	GICR |= (1<<INT2); /* Enable external interrupt pin INT2 */
}

/*
 * Description:
 * For System Clock=1Mhz and timer prescaler is F_CPU/64.
 * Timer frequency will be around 15.625Khz, Ttimer = 64us
 * For compare value equals to 15625 the timer will generate compare match interrupt every 1s.
 */
void Timer1_start(void)
{
	/* Initial value */
	TCNT1 = 0;
	/* Store compare value in OCR1A register */
	OCR1A = 15625;
	/* Enable Timer1 Compare A Interrupt */
	TIMSK |= (1<<OCIE1A);
	/* Set FOC1A for non-PWM mode */
	TCCR1A |= (1<<FOC1A);
	/* Select prescaler of N = 64 and select compare mode */
	TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10);
}

/*********** Main Code *************/
int main(void)
{
	DDRC |= 0x0F; /* Configure First 4 Pins in PORTC as output Pins (for the decoder) */
	DDRA |= 0x3F; /* Configure First 6 Pins in PORTA as output Pins (for the 6 enables of the 6 7-segments) */
	SREG |= (1<<7); /* Enable the Global Interrupt (I_bit) */

    PORTC &= 0xF0;
    PORTA &= 0xC0;

	INT0_init();
	INT1_init();
	INT2_init();
	Timer1_start();

	while(1)
	{
		Check_Time();
		Display_Time();
	}
}

/****** Functions Definitions ******/

/*
 * Description:
 * Display the time on the 7-segments.
 */
void Display_Time(void)
{
	PORTA = (PORTA & 0xC0) | ((1<<5) & 0x3F);
	PORTC = (PORTC & 0xF0) | ((timer.seconds % 10) & 0x0F);
	_delay_us(300);
	PORTA = (PORTA & 0xC0) | ((1<<4) & 0x3F);
	PORTC = (PORTC & 0xF0) | ((timer.seconds / 10) & 0x0F);
	_delay_us(300);
	PORTA = (PORTA & 0xC0) | ((1<<3) & 0x3F);
	PORTC = (PORTC & 0xF0) | ((timer.minutes % 10) & 0x0F);
	_delay_us(300);
	PORTA = (PORTA & 0xC0) | ((1<<2) & 0x3F);
	PORTC = (PORTC & 0xF0) | ((timer.minutes / 10) & 0x0F);
	_delay_us(300);
	PORTA = (PORTA & 0xC0) | ((1<<1) & 0x3F);
	PORTC = (PORTC & 0xF0) | ((timer.hours % 10) & 0x0F);
	_delay_us(300);
	PORTA = (PORTA & 0xC0) | ((1<<0) & 0x3F);
	PORTC = (PORTC & 0xF0) | ((timer.hours / 10) & 0x0F);
	_delay_us(300);
}

/*
 * Description:
 * Check the seconds and minutes are correct (from 0 to 59).
 */
void Check_Time(void)
{
	if(timer.seconds <= 59)
		return;
	else
	{
		timer.seconds = 0;
		timer.minutes++;
	}
	if(timer.minutes <= 59)
		return;
	else
	{
		timer.minutes = 0;
		timer.hours++;
	}
}
