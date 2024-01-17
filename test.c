#define F_CPU 1000000UL

#include <avr/interrupt.h>
#include <avr/io.h>

#include <util/delay.h>

#include "base.h"

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

#define LED_PIN PD0
#define LED_PORT PORTD
#define LED_DDR DDRD

#define OW_PORT PORTC
#define OW_DDR DDRC
#define OW_PIN PINC
#define OW_BIT PC0

typedef enum {
  OW_DeviceNoFound,
  OW_ReadFailed,
} OW_Error;

u8 ow_reset(void) {
  u8 res = 0;

  // Reset
  {
    LOW(OW_PORT, OW_BIT);
    OUTPUT_MODE(OW_DDR, OW_BIT);
    _delay_us(480);
  }

  // Presence
  {
    INPUT_MODE(OW_DDR, OW_BIT);
    _delay_us(60);
  }

  // Read
  {
    res = READ(OW_PIN, OW_BIT); // 0 - OK, 1 - not OK
    _delay_us(420);
  }

  return res;
}

void ow_write_bit(u8 bit) {
  LOW(OW_PORT, OW_BIT);
  OUTPUT_MODE(OW_DDR, OW_BIT);

  _delay_us(1);

  if (bit) {
    INPUT_MODE(OW_DDR, OW_BIT);
  }

  _delay_us(60);

  INPUT_MODE(OW_DDR, OW_BIT);
}

void ow_write_byte(u8 byte) {
  for (u8 i = 0; i < 8; i += 1) {
    ow_write_bit(byte & 1);
    byte >>= 1;
  }
}

u8 ow_read_bit(void) {
  u8 res = 0;

  LOW(OW_PORT, OW_BIT);
  OUTPUT_MODE(OW_DDR, OW_BIT);
  _delay_us(1);
  INPUT_MODE(OW_DDR, OW_BIT);
  _delay_us(14);
  res = READ(OW_PIN, OW_BIT); // 0 - OK, 1 - not OK
  _delay_us(45);

  return res;
}

u8 ow_read_byte(void) {
  u8 res = 0;

  for (u8 i = 0; i < 8; i += 1) {
    res >>= 1;
    res |= (ow_read_bit() << 7);
  }

  return res;
}

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

#define THERM_DECIMAL_STEPS_12BIT 625 //.0625

volatile uint16_t timer_ticks = 0;

// Глобальные переменные для хранения данных на дисплеях
volatile u8 display1_data = 0;
volatile u8 display2_data = 0;

int main(void) {
  {
    // Настройка Timer1
    TCCR1B |=
        (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, prescaler = 64
    OCR1A = 77; // Прерывание каждые 5 мс при F_CPU = 1 MHz и prescaler = 64
    TIMSK |= (1 << OCIE1A); // Разрешение прерывания от совпадения

    // Разрешение глобальных прерываний
    sei();
  }

  OUTPUT_MODE(DDRC, PC6);
  HIGH(PORTC, PC6);

  OUTPUT_MODE(DDRD, PD0);
  OUTPUT_MODE(DDRD, PD1);

  // Buffer length must be at least 12bytes long! ["+XXX.XXXX C"]
  u8 temperature[2] = {};
  int8_t digit = 0;
  u16 decimal = 0;

  // Set PORTC as output for segments
  DDRB = 0xFF;

  while (1) {
    while (ow_reset());

    ow_write_byte(CMD_SKIP_ROM);
    ow_write_byte(CMD_CONVERT_TEMP);

    // Wait until conversion is complete
    // while (ow_read_bit());

    ow_reset();
    ow_write_byte(CMD_SKIP_ROM);
    ow_write_byte(CMD_READ_SCRATCHPAD);

    temperature[0] = ow_read_byte();
    temperature[1] = ow_read_byte();

    // Store temperature integer digits and decimal digits
    digit = temperature[0] >> 4;
    digit |= (temperature[1] & 0x7) << 4;

    // Store decimal digits
    decimal = temperature[0] & 0xf;
    decimal *= THERM_DECIMAL_STEPS_12BIT;

    display1_data = digit % 10;
    display2_data = digit / 10;

    _delay_ms(720);
  }
}

ISR(TIMER1_COMPA_vect) {
  static u8 display_selector = 0;

  // Выбор активного дисплея
  if (display_selector == 0) {
    LOW(PORTD, PD1);
    HIGH(PORTD, PD0);
    PORTB = ~digitPatterns[display1_data];
  } else {
    LOW(PORTD, PD0);
    HIGH(PORTD, PD1);
    PORTB = ~digitPatterns[display2_data];
  }

  // Переключение на следующий дисплей
  display_selector ^= 1;
}