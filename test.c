#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

#include "base.h"
#include "one_wire.h"
#include "one_wire.c"

// Define segment patterns for numbers 0 to 9
<<<<<<< HEAD
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
=======
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

// Макрос для установки пина как вход
#define INPUT_MODE(port, pin) ((port) &= ~(1 << (pin)))

// Макрос для установки пина как выход
#define OUTPUT_MODE(port, pin) ((port) |= (1 << (pin)))

// Макрос для установки пина в состояние "низкий уровень"
#define LOW(port, pin) ((port) &= ~(1 << (pin)))

// Макрос для установки пина в состояние "высокий уровень"
#define HIGH(port, pin) ((port) |= (1 << (pin)))

// Макрос для чтения данных с пина
#define READ(port, pin) ((port & (1 << pin)))

// 1-wire

#define CMD_CONVERT_TEMP 0x44
#define CMD_READ_SCRATCHPAD 0xbe
#define CMD_WRITE_SCRATCHPAD 0x4e
#define CMD_COPY_SCRATCHPAD 0x48
#define CMD_RECALL_EEPROM 0xb8
#define CMD_READ_POWER_SUPPLY 0xb4
#define CMD_SEARCH_ROM 0xf0
#define CMD_READ_ROM 0x33
#define CMD_MATCH_ROM 0x55
#define CMD_SKIP_ROM 0xcc
#define CMD_ALARM_SEARCH 0xec

#define THERM_DECIMAL_STEPS_12BIT 625 //.0625

#define LED_PIN PD0
#define LED_PORT PORTD
#define LED_DDR DDRD

#define DS_PORT PORTC
#define DS_DDR DDRC
#define DS_PIN PINC
#define DS_DQ PC0

uint8_t DS18B20_init(void) {

  uint8_t result_ok = 0;

  // Reset
  {
    LOW(DS_PORT, DS_DQ);
    OUTPUT_MODE(DS_DDR, DS_DQ);
    _delay_us(480);
  }

  // Presence
  {
    INPUT_MODE(DS_DDR, DS_DQ);
    _delay_us(60);
  }

  // Read
  {
    result_ok = !READ(DS_PIN, DS_DQ); // 0 - OK, 1 - not OK
    _delay_us(420);
  }

  return result_ok;
}

void DS18B20_write_bit(uint8_t bit) {
  // Pull line low
  LOW(DS_PORT, DS_DQ);
  OUTPUT_MODE(DS_DDR, DS_DQ);

  _delay_us(1);

  if (bit) {
    INPUT_MODE(DS_DDR, DS_DQ);
  }

  _delay_us(60);

  INPUT_MODE(DS_DDR, DS_DQ);
}

void DS18B20_write_byte(uint8_t byte) {
  for (uint8_t i = 0; i < 8; i += 1) {
    DS18B20_write_bit(byte & 1);
    byte >>= 1;
  }
}

uint8_t DS18B20_read_bit(void) {
  uint8_t result_bit = 0;

  // Pull line low for 1uS
  LOW(DS_PORT, DS_DQ);
  OUTPUT_MODE(DS_DDR, DS_DQ);

  _delay_us(1);

  // Release line and wait for 14uS
  INPUT_MODE(DS_DDR, DS_DQ);

  _delay_us(14);

  // Read line value
  if (READ(DS_PIN, DS_DQ)) {
    result_bit = 1;
  }

  _delay_us(45);

  return result_bit;
}

uint8_t DS18B20_read_byte(void) {
  uint8_t result_n = 0;

  for (uint8_t i = 0; i < 8; i += 1) {
    result_n >>= 1;
    result_n |= (DS18B20_read_bit() << 7);
  }

  return result_n;
}

int main(void) {
  OUTPUT_MODE(LED_DDR, LED_PIN);

  // Buffer length must be at least 12bytes long! ["+XXX.XXXX C"]
  uint8_t temperature[2] = {};
  int8_t digit = 0;
  uint16_t decimal = 0;
>>>>>>> a61472f91c68db2af953208e44d30691717ccb56

  // Set PORTC as output for segments
  DDRB = 0xFF;

<<<<<<< HEAD
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
=======
  while (1) {
    DS18B20_init();
    DS18B20_write_byte(CMD_SKIP_ROM);
    DS18B20_write_byte(CMD_CONVERT_TEMP);

    // Wait until conversion is complete
    while (!DS18B20_read_bit())
      ;

    DS18B20_init();
    DS18B20_write_byte(CMD_SKIP_ROM);
    DS18B20_write_byte(CMD_READ_SCRATCHPAD);
>>>>>>> a61472f91c68db2af953208e44d30691717ccb56

    temperature[0] = DS18B20_read_byte();
    temperature[1] = DS18B20_read_byte();

<<<<<<< HEAD
        //Store decimal digits
        decimal = temperature[0] & 0xf;
        decimal *= OW_THERM_DECIMAL_STEPS_12BIT;

        PORTB = ~digitPatterns[digit / 10];
        HIGH(PORTB, PB0);
        _delay_ms(500);
        PORTB = ~digitPatterns[digit % 10];
        LOW(PORTB, PB0);
=======
    // Store temperature integer digits and decimal digits
    digit = temperature[0] >> 4;
    digit |= (temperature[1] & 0x7) << 4;

    // Store decimal digits
    decimal = temperature[0] & 0xf;
    decimal *= THERM_DECIMAL_STEPS_12BIT;

    PORTB = ~digitPatterns[digit / 10];
    HIGH(PORTB, PB0);
    _delay_ms(500);
    PORTB = ~digitPatterns[digit % 10];
    LOW(PORTB, PB0);

    if (digit > 25) {
      HIGH(LED_PORT, LED_PIN);
    } else {
      LOW(LED_PORT, LED_PIN);
>>>>>>> a61472f91c68db2af953208e44d30691717ccb56
    }

    // _delay_ms(750);
  }

  return 0;
}
