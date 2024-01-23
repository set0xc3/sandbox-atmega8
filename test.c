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

#define OW_A 5
#define OW_B 64
#define OW_C 60
#define OW_D 10
#define OW_E 9
#define OW_F 55
#define OW_G 0
#define OW_H 480
#define OW_I 70
#define OW_J 410

typedef enum {
  OW_DeviceNoFound,
  OW_ReadFailed,
} OW_Error;

u8 ow_reset(void);
void ow_write(u8 byte);
void ow_write_bit(u8 bit);
void ow_write_bytes(u8 *buf, u8 byte);
u8 ow_read(void);
u8 ow_read_bit(void);
void ow_read_bytes(u8 *buf);

u8 ow_reset(void) {
  u8 res = 0;
  u8 retries = 125;

  INPUT_MODE(OW_DDR, OW_BIT);
  do {
    if (--retries == 0) {
      return 0;
    }
    _delay_us(1);
  } while (!READ(OW_PIN, OW_BIT));

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
    _delay_us(410);
  }

  return res;
}

void ow_write(u8 byte) {
  for (u8 bit_mask = 0x01; bit_mask; bit_mask <<= 1) {
    ow_write_bit(bit_mask & byte ? 1 : 0);
  }
}

void ow_write_bit(u8 bit) {
  LOW(OW_PORT, OW_BIT);
  OUTPUT_MODE(OW_DDR, OW_BIT);

  _delay_us(10);

  if (bit & 1) {
    LOW(OW_PORT, OW_BIT);
    OUTPUT_MODE(OW_DDR, OW_BIT);
    _delay_us(10);
    HIGH(OW_PORT, OW_BIT);
    _delay_us(60);
  } else {
    LOW(OW_PORT, OW_BIT);
    OUTPUT_MODE(OW_DDR, OW_BIT);
    _delay_us(65);
    HIGH(OW_PORT, OW_BIT);
    _delay_us(5);
  }
}

void ow_write_bytes(u8 *buf, u8 byte) {
  for (u8 i = 0; i < 8; i += 1) {
    ow_write(buf[i]);
  }
}

u8 ow_read(void) {
  u8 res = 0;

  for (u8 bit_mask = 0x01; bit_mask; bit_mask <<= 1) {
    if (ow_read_bit()) {
      res |= bit_mask;
    }
  }

  return res;
}

u8 ow_read_bit(void) {
  u8 res = 0;

  LOW(OW_PORT, OW_BIT);
  OUTPUT_MODE(OW_DDR, OW_BIT);
  _delay_us(2);

  INPUT_MODE(OW_DDR, OW_BIT);
  _delay_us(10);

  res = READ(OW_PIN, OW_BIT); // 0 - OK, 1 - not OK
  _delay_us(55);

  return res;
}

void ow_read_bytes(u8 *buf) {
  for (u8 i = 0; i < 8; i += 1) {
    buf[i] = ow_read();
  }
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
  u8 temp_data[9] = {0};
  f32 celsius, fahrenheit = 0.0;

  // Set PORTC as output for segments
  DDRB = 0xFF;

  while (1) {
    ow_reset();
    ow_write(CMD_SKIP_ROM);
    ow_write(CMD_CONVERT_TEMP);

    _delay_ms(750);

    ow_reset();
    ow_write(CMD_SKIP_ROM);
    ow_write(CMD_READ_SCRATCHPAD);

    for (u8 i = 0; i < 9; i += 1) {
      temp_data[i] = ow_read();
    }

    i16 raw = (temp_data[1] << 8) | temp_data[0];
    u8 cfg = (temp_data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time

    celsius = (f32)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;

    if (celsius >= 22) {
      // display1_data = 1;
    }

    display1_data = (u8)celsius % 10;
    display2_data = (u8)celsius / 10;
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