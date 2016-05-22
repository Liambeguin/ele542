/*
    Title:    AVR-GCC test program #7 for the STK200 eva board
    Authors:  Pieter Conradie, Volker Oth
    Date:     9/1999
    Purpose:  Demonstrates simple printing to and reading from the UART
    needed
    Software: AVR-GCC to compile
    needed
    Hardware: ATs90s08515/8535/Mega603 on STK200/300 board
    Note:     To contact me, mail to
                  volkeroth@gmx.de
              You might find more AVR related stuff at my homepage:
                  http://members.xoom.com/volkeroth
*/

#include "uart.h"
#include <avr/pgmspace.h>

int main(void)
{
    u08 data;
    /* Initialise UART */
    UART_Init();

    PRINT("Hello World !");
    EOL();

    for (;;) {  /* loop forever */
        PRINT("Press any key...");
        EOL();
    
        data = UART_ReceiveByte();
    
        PRINT("You pressed '");
        UART_SendByte(data);
        PRINT("' which is 0x");
        UART_Printfu08(data);
        PRINT(" in hexadecimal.");
        EOL();
    }
}
