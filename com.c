#include <avr/io.h>
#include <util/delay.h>

#define BAUDRATE 9600
#define BAUD_PRESCALE ((F_CPU / (BAUDRATE * 16UL)) - 1)

void USART_Init()
{
    UBRRH = (BAUD_PRESCALE >> 8);
    UBRRL = BAUD_PRESCALE;
    UCSRB = (1 << TXEN) | (1 << RXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

void USART_Transmit(unsigned char data)
{
    while (!(UCSRA & (1 << UDRE)))
        ;
    UDR = data;
}

int main(void)
{
    USART_Init();

    while (1)
    {
        USART_Transmit('H');
        USART_Transmit('e');
        USART_Transmit('l');
        USART_Transmit('l');
        USART_Transmit('o');
        USART_Transmit('\n');
        _delay_ms(1000);
    }

    return 0;
}