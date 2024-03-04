#include <avr/io.h>
#include <util/delay.h>

// Define segment patterns for numbers 0 to 9
const uint8_t digitPatterns[10] = {
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

int main(void) {
    // Set the 7-segment display pins as outputs
    DDRB = 0xFF;
    
    while (1) {
        for (uint8_t i = 0; i < 10; i += 1) {
          PORTB = ~digitPatterns[i];
          _delay_ms(50);
        }
    }
    
    return 0;
}