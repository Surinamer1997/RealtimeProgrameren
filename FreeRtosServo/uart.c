#include <avr/io.h>
#include <stdlib.h>

#include "uart.h"


void initUsart(unsigned int prescaler)
{
   /* Set baud rate */
   UBRR0H = (unsigned char)(prescaler>>8);
   UBRR0L = (unsigned char)prescaler;

   /* Enable receiver and transmitter */
   UCSR0B = (1<<RXEN0)|(1<<TXEN0) | (1<<RXCIE0);

   /* Set frame format: 8data, 1stop bit */
    UCSR0C = (!(1<<USBS0))|(1<<UCSZ01)|(1<<UCSZ00);
}

void writeChar(unsigned char data)
{
   /* Wait for empty transmit buffer */
   while ( !( UCSR0A & (1<<UDRE0)) );
   /* Put data into buffer, sends the data */
   UDR0 = data;
}

void writeInt(int16_t nr)
{
   char buffer[8];
   itoa(nr,buffer,10);
   writeString(buffer);
}

void writeString(char* s)
{
     while(*s)
         writeChar(*s++);
}
