#include <avr/io.h>
#include <util/delay.h>

#define LED_PIN PB0

int
main()
{
  DDRB  = 0b00000001; // set LED pin as OUTPUT
  PORTB = 0b00000000; // set all pins to LOW

  while (1) {
    PORTB ^= 1 << LED_PIN; // toggle LED pin
    _delay_ms(500);
  }

  return 0;
}
