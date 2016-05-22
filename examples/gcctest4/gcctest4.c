/*
    Title:    AVR-GCC test program #4 for the STK200 eva board
    Author:   Volker Oth
    Date:     4/1999
    Purpose:  Uses the UART to communicate with a terminal program on the pc.
              The "tranceive complete" interrupt is used to send the string 
              "Serial Data from AVR receiced###" continuously to the pc.
              When a byte was received from the pc, the "receive complete"
              interrupt is called, which outputs the byte to PortB where the
              LEDs visualize the 8 bits it consists of.
              UART format: 9600 baud, 8bit, 1 stopbit, no parity
    needed
    Software: AVR-GCC to compile
    needed
    Hardware: ATS90S8515/8535/2313/mega(?) on STK200/300 board
    Note:     To contact me, mail to
                  volkeroth@gmx.de
              You might find more AVR related stuff at my homepage:
                  http://members.xoom.com/volkeroth
*/

#include <avr/io.h>
#include <avr/interrupt.h>


#ifndef F_CPU
#define F_CPU 16000000UL            /* Crystal 16.000 Mhz */
#endif
#define UART_BAUD_RATE 9600         /* 9600 baud */


#define UART_BAUD_SELECT (F_CPU/(UART_BAUD_RATE*16l)-1)


typedef unsigned char  u08;
typedef          char  s08;
typedef unsigned short u16;
typedef          short s16;


/* uart globals */
static volatile u08 *uart_data_ptr;
static volatile u08 uart_counter;


ISR(USART_TXC_vect)    
/* signal handler for uart txd ready interrupt */
{
   uart_data_ptr++;

   if (--uart_counter)
      UDR = *uart_data_ptr;         /* write byte to data buffer */
}


ISR(USART_RXC_vect)    
/* signal handler for receive complete interrupt */
{
   register char led;

   led = UDR;                       /* read byte for UART data buffer */
   PORTB = ~led;                    /* output received byte to PortB (LEDs) */
}


void uart_send(u08 *buf, u08 size)
/* send buffer <buf> to uart */
{
   if (!uart_counter) {
      uart_data_ptr = buf;          /* write first byte to data buffer */
      uart_counter = size;
      UDR = *buf;
   }
}


void uart_init(void)
/* initialize uart */
{
   /* configure asynchronous operation, no parity, 1 stop bit, 8 data bits, Tx on rising edge */
   UCSRC = ((1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0)|(0<<UCPOL));       
   /* enable RxD/TxD and ints */
   UCSRB = ((1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN)|(0<<UCSZ2));       
   /* set baud rate */
   UBRRH = (u08)(UART_BAUD_SELECT >> 8);          
   UBRRL = (u08)(UART_BAUD_SELECT & 0x00FF);          
}


int main(void)
{
   DDRB = 0xff;                     /* PortB output */
   PORTB = 0x00;                    /* switch LEDs on */

   uart_init();
   sei();                           /* enable interrupts */

   for (;;) {                       /* loop forever */
      uart_send("Serial Data from AVR received###", 32);
   }            
}
