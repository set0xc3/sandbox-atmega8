#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

#include "base.h"
#include "one_wire.h"
#include "one_wire.c"

// Define segment patterns for numbers 0 to 9
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

    // Buffer length must be at least 12bytes long! ["+XXX.XXXX C"]
    u8 temperature[2] = {};
    int8_t digit = 0;
    u16 decimal = 0;

    // Set PORTC as output for segments
    DDRB = 0xFF;

    while (1) {
        ow_reset();
        _delay_us(420);
        ow_write_byte(OW_CMD_SKIP_ROM);
        ow_write_byte(OW_CMD_CONVERT_TEMP);

        _delay_ms(750);
        
        ow_reset();
        _delay_us(420);
        ow_write_byte(OW_CMD_SKIP_ROM);
        ow_write_byte(OW_CMD_READ_SCRATCHPAD);

        temperature[0] = ow_read_byte();
        temperature[1] = ow_read_byte();

        //Store temperature integer digits and decimal digits
        digit = temperature[0] >> 4;
        digit |= (temperature[1] & 0x7) << 4;

        //Store decimal digits
        decimal = temperature[0] & 0xf;
        decimal *= OW_THERM_DECIMAL_STEPS_12BIT;

        PORTB = ~digitPatterns[digit / 10];
        HIGH(PORTB, PB0);
        _delay_ms(500);
        PORTB = ~digitPatterns[digit % 10];
        LOW(PORTB, PB0);
    }

    return 0;
}