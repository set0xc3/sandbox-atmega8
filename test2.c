#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

#include "base.h"

const u8 digitPatterns[10] = {
  // a  b  c  d  e  f  g  dp
  0b11111100, // 0
  0b01100000, // 1
  0b11011010, // 2
  0b11110010, // 3
  0b01100110, // 4
  0b10110110, // 5
  0b10111110, // 6
  0b11100000, // 7
  0b11111110, // 8
  0b11110110  // 9
};

int main(void) 
{
    OUTPUT_MODE(DDRD, PD0);
    OUTPUT_MODE(DDRD, PD1);

    DDRB = 0xFF;

    // HIGH(PORTD, PD1);
    // PORTB = ~digitPatterns[8];

    for (;;) {
      // for (u8 i = 0; i < 10; i += 1) {
      //   HIGH(PORTD, PD0);
      //   PORTB = ~digitPatterns[i];
      //   _delay_ms(400);
      //   LOW(PORTD, PD0);

      //   HIGH(PORTD, PD1);
      //   PORTB = ~digitPatterns[i];
      //   _delay_ms(400);
      //   LOW(PORTD, PD1);

      //   _delay_ms(400);
      // }
      

      HIGH(PORTD, PD0);
      PORTB = ~digitPatterns[0];
      _delay_ms(1);
      LOW(PORTD, PD0);
      _delay_ms(1);

      HIGH(PORTD, PD1);
      PORTB = ~digitPatterns[8];
      _delay_ms(1);
      LOW(PORTD, PD1);
      _delay_ms(1);
    }

    return 0;
}