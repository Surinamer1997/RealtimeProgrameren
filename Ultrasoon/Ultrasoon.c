#define F_CPU 16000000 //Clock Speed

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/wdt.h>

#include "uart.h"

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

unsigned char working;
unsigned char rising_edge;
double timer_value;
double distance_cm = 0.0;

void Initialize_external_interrupt()
{
  EICRA |= (1 << ISC10); //Any logical change on INT1
  EIMSK |= (1 << INT1); //Enable INT1
}

void Initialize_timer3()
{
  TCCR3B |= (1 << CS30); //No prescaling
  TCNT3 = 0;      //Reset timer
  TIMSK3 |= (1 << TOIE3); //Timer overflow interrupt enable
}

ISR (TIMER3_OVF_vect)
{
  if(rising_edge==1) //Check if there was echo
  {
    timer_value++;
    /*Check if isnt out of range*/
    if(timer_value > 91)
    {
      working = 0;
      distance_cm = -1;
      rising_edge = 0;
    }
  }
}

ISR (INT1_vect)
{
  if(working==1)
  {
    if(rising_edge==0)
    {
      rising_edge=1;
      TCNT3 = 0;
      timer_value = 0;
    }
    else
    {
      rising_edge = 0;
      distance_cm = (int)(((timer_value*256 + TCNT3) /160000) * 340) / 2;//(timer_value*256 + TCNT1)/58;
      working = 0;
    }
  }
}

static void TaskUltrasoon(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED

int main() {
    DDRD &=~ (1 << PD1);
    DDRB |= (1 << PB0);
    Initialize_external_interrupt();
    Initialize_timer3();
    initUsart(BAUD_PRESCALE);

    xTaskCreate(
                TaskUltrasoon
                ,  (const portCHAR *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
                ,  256				// Tested 9 free @ 208
                ,  NULL
                ,  3
                ,  NULL ); // */


        vTaskStartScheduler();


        return 0;
}

static void TaskUltrasoon(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
        /* The xLastWakeTime variable needs to be initialised with the current tick
        count.  Note that this is the only time we access this variable.  From this
        point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
        API function. */
        xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        writeString("Distance: ");
        writeInt(distance_cm);
        writeString("cm\n");

        _delay_ms(50);    //Restart HC-SR04
        PORTB  &=~ (1 << PINB0);
        _delay_us(1);
        PORTB |= (1 << PINB0); //Send 10us second pulse
        _delay_us(10);
        PORTB &=~ (1 << PINB0);
        working = 1;

        _delay_ms(1000);
    }
}
