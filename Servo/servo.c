
#define F_CPU 16000000 //Clock Speed
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "uart.h"


#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define MAX_CYLCE		  40000
#define PULSE_MIN         2000         /* experiment with these values */
#define PULSE_MAX         4000              /* to match your own servo */
#define PULSE_MID         3000

volatile int state = 1;
volatile int time = 2000;
volatile int rx = 0;
volatile char rx_buffer[8];
volatile int totalCount = 0;
volatile int data_count = 0;
volatile int angle = 0;
volatile int keepWDAlive = 1;

// prescaler 8 ==> ICR1 = 40,000
void initTimer1Servo(void) {
                   /* Set up Timer1 (16bit) to give a pulse every 20ms */
                             /* Use Fast PWM mode, counter max in ICR1 */
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  TCCR1B |= (1 << CS11);  /* /1 prescaling -- counting in microseconds */
  ICR1 = MAX_CYLCE;                                    /* TOP value = 20ms */
  TIMSK1 |= (1 << TOIE1);                 /* overflow interrupt enable */
  DDRA |= (1 << PA7);                            /* set pin for output */
}

void WDT_Init(void)
{
	//disable interrupts
	cli();
	//reset watchdog
	wdt_reset();
	//set up WDT interrupt
	WDTCSR = (1<<WDCE)|(1<<WDE);
	//Start watchdog timer with 4s prescaller
	WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP3);
	//Enable global interrupts
	sei();
}

// ~20 ms timer
ISR(TIMER1_OVF_vect ) {
//	// 56 us == 1 graad
	if(state == 1){
		ICR1 = time; // kort
		PORTA |= (1 << PA7);
	}
	if(state == 0){
		ICR1 = MAX_CYLCE - time; //lang
		PORTA &= ~(1 << PA7);
	}
	if(keepWDAlive){
		wdt_reset();
	}
	state = !state;
}


//Watchdog timeout ISR
ISR(WDT_vect)
{
    time = PULSE_MID;
    //wdt_disable();
}

ISR(USART0_RX_vect)
{
   rx_buffer[data_count] = UDR0;
   if(rx_buffer[data_count] == 10){
	   rx_buffer[data_count] = '\0';
	   angle = atoi((void*)rx_buffer);
	   if(angle < 0){
		   keepWDAlive = 0;
	   } else {
		   time = angle * (2000/180)+2000;
	   }
	   data_count = 0;
   } else {
	   data_count++;
   }
}


int main(void)
{
	DDRA |= (1 << PA7);
	initTimer1Servo();
	initUsart(BAUD_PRESCALE);
	WDT_Init();
	sei();
	while(1)
	{
	  _delay_ms(100);
	  writeString("Angle & pwm value: ");
	  writeInt(angle);
	  writeString("\n");
	  writeString("rx: ");
	  writeString((char*) rx_buffer);
	  writeString("\n");
	  writeString("WDAlive: ");
	  writeInt(keepWDAlive);
	  writeString("\n");
	}
}
