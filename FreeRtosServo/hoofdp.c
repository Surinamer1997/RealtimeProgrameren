/*
 * hoofdp.c
 *
 *  Created on: Apr 1, 2017
 *      Author: john
 */
#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "serial.h"
#include "uart.h"

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)


#define MAX_CYLCE		  40000
#define PULSE_MIN         2000         /* experiment with these values */
#define PULSE_MAX         4000              /* to match your own servo */
#define PULSE_MID         3000

volatile int state = 1;
volatile int pwmTime = 2000;

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


// ~20 ms timer
ISR(TIMER1_OVF_vect ) {
//	// 56 us == 1 graad
	if(state == 1){
		ICR1 = pwmTime; // kort
		PORTA |= (1 << PA7);
	}
	if(state == 0){
		ICR1 = MAX_CYLCE - pwmTime; //lang
		PORTA &= ~(1 << PA7);
	}
	state = !state;
}


int sendAngle= 0;
QueueHandle_t xQueue;


static void TaskSoundControl(void *pvParameters);
static void TaskBlinkRedLED(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED
static void TaskServoControl(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED


int main() {

	DDRA |= (1 << PA7);
	initTimer1Servo();
	initUsart(BAUD_PRESCALE);

	xQueue = xQueueCreate( 10, sizeof( struct AMessage * ) );

    xTaskCreate(
		TaskBlinkRedLED
		,  (const portCHAR *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */

    xTaskCreate(
		TaskServoControl
		,  (const portCHAR *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */

    sei();
	vTaskStartScheduler();


	return 0;
}

static void TaskSoundControl(void *pvParameters) {
	(void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	unsigned int ad_value;
	// Enable ADC
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);

	// Select ADC channel
	ADMUX = (1 << ADLAR) | (1 << REFS0);
	// Analog Pin A0 is default.

	while(1) {
		//Start single conversion
		ADCSRA |= (1 << ADSC);

		// wait for the conversion to complete
		while(!(ADCSRA & (1 << ADIF)));
		adc_value = ADC;
		if (adc_value < 512) {
			// pass
		} else {
			//pass
		}

		//clear ADIF
		ADCSRA |= (1 << ADIF);
	}


	// Analog pin 15: ADC15 - PCINT23


}

static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	DDRB |= _BV(DDB7);

    while(1)
    {
    	PORTB |=  _BV(PORTB7);       // main (red IO_B7) LED on. EtherMega LED on
    	sendAngle = 1;
    	xQueueSend( xQueue, ( void * ) &sendAngle, ( TickType_t ) 0 );
		vTaskDelayUntil( &xLastWakeTime, ( 2000 / portTICK_PERIOD_MS ) );

		PORTB &= ~_BV(PORTB7);       // main (red IO_B7) LED off. EtherMega LED off
		sendAngle = 180;
		xQueueSend( xQueue, ( void * ) &sendAngle, ( TickType_t ) 0 );
		vTaskDelayUntil( &xLastWakeTime, ( 2000 / portTICK_PERIOD_MS ) );



		xSerialxPrintf_P( &xSerialPort, PSTR("RedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }
}



static void TaskServoControl(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();


	int angle = 180;
    while(1)
    {

    	pwmTime = angle * (2000/180)+2000;
		vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) );

		//Read message queue
		if( xQueue != 0 )
		{
			// Receive a message on the created queue.  Block for 10 ticks if a
			// message is not immediately available.
			if( xQueueReceive( xQueue, &( angle ), ( TickType_t ) 10 ) )
			{
				writeString("Queue: receive message!!\n");
			}
		}

		xSerialxPrintf_P( &xSerialPort, PSTR("Servo angle:  @ %u\r\n"), angle);
    }
}



