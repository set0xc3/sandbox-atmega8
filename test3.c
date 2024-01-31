#include <stdio.h>
#define F_CPU 1000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "base.h"

#define ENABLE_INTERRUPTS sei()
#define DISABLE_INTERRUPTS cli()

// Массив значениий для семисегментного индикатора
char display_segment_numbers[11] = {
    0b11111100, // 0
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11110110, // 9
    0b00000010, // -
};

enum Work_Mode {
  WorkMode_Off = 0,
  WorkMode_Control,
  WorkMode_Alarm,
  WorkMode_COUNT,
};

enum Menu_Kind {
  MenuKind_CP = 0,
  MenuKind_PP,
  MenuKind_OB,
  MenuKind_OP,
  MenuKind_TP,
  MenuKind_HI,
  MenuKind_TO,
  MenuKind_TU,
  MenuKind_BU,
  MenuKind_UF,
  MenuKind_COUNT,
};

char display_segment_menu[10][2] = {
    // CP
    {0b10011101, 0b11001111},

    // PP
    {0b11001111, 0b11001111},

    // Ob
    {0b11111101, 0b00111111},

    // OP
    {0b11111101, 0b11001111},

    // TP
    {0b00011111, 0b11001111},

    // HI
    {0b01101111, 0b00001101},

    // TO
    {0b00011111, 0b11111101},

    // TU
    {0b00011111, 0b01111101},

    // bU
    {0b00111111, 0b01111101},

    // UF
    {0b01111101, 0b10001111},
};

u8 menu_data[10] = {
    10, 3, 99, 90, 40, 3, 5, 30, 1, 0,
};

u8 segcounter = 0;
volatile u8 display_1, display_2 = 0;

// Прерывание по переполнению T2, динамическая индикация
ISR(TIMER2_OVF_vect) {
  PORTB = 0xFF;
  PORTD = (1 << segcounter);

  switch (segcounter) {
  case 0:
    PORTB = ~(display_segment_numbers[display_1]);
    break;
  case 1:
    PORTB = ~(display_segment_numbers[display_2]);
    break;
  }
  if ((segcounter++) > 2) {
    segcounter = 0;
  }
}

u8 Temp_MSB, Temp_LSB, OK_Flag, temp_flag = 0;

// Инициализация DS18B20
u8 ds18b20_reset(void) {
  DISABLE_INTERRUPTS;

  PORTC &= ~(1 << PC0); // Устанавливаем низкий уровень
  DDRC |= (1 << PC0);   // PC0 - выход

  ENABLE_INTERRUPTS;

  _delay_us(480);

  DISABLE_INTERRUPTS;

  DDRC &= ~(1 << PC0); // PC0 - вход
  _delay_us(60);

  OK_Flag = !(PINC & (1 << PC0)); // Ловим импульс присутствия датчика

  ENABLE_INTERRUPTS;

  // если OK_Flag = 0 датчик подключен, OK_Flag = 1 датчик не подключен
  _delay_us(410);

  return OK_Flag;
}

// Функция чтения байта из DS18B20
u8 ds18b20_read(void) {
  u8 res = 0;
  for (u8 i = 0; i < 8; i++) {

    DISABLE_INTERRUPTS;

    DDRC |= (1 << PC0); // PC0 - выход
    _delay_us(2);
    DDRC &= ~(1 << PC0); // PC0 - вход
    _delay_us(8);
    res = res >> 1; // Следующий бит
    if (PINC & (1 << PC0)) {
      res |= 0x80;
    }

    ENABLE_INTERRUPTS;

    _delay_us(60);
  }
  return res;
}

// Функция записи байта в DS18B20
void ds18b20_write(u8 data) {
  for (u8 i = 0; i < 8; i++) {

    DISABLE_INTERRUPTS;

    DDRC |= (1 << PC0); // PC0 - выход
    _delay_us(2);

    if (data & 0x01) {
      DDRC &= ~(1 << PC0); // PC0 - вход
    } else {
      DDRC |= (1 << PC0); // PC0 - выход
    }

    data = data >> 1; // Следующий бит
    _delay_us(60);

    DDRC &= ~(1 << PC0); // PC0 - вход

    ENABLE_INTERRUPTS;

    _delay_us(2);
  }
}

static u8 retries = 255;

// Главная функция
int main(void) {
  // Настройка портов ввода/вывода
  DDRD |= (1 << PB0) | (1 << PB1); // Разряды
  PORTD = 0x00;
  DDRB = 0xFF; // Сегменты
  PORTB = 0x00;

  // Настройка Т2
  TIMSK |= (1 << TOIE2); // Разрешение прерывания по Т2
  TCCR2 |= (1 << CS21);  // Предделитель на 8

  u16 buffer = 0;
  u16 temp_int_1,
      temp_int_2 = 0; // Переменные для целого значения температуры
  // u32 temp_point = 0; // Переменная для дробного значения температуры

  ENABLE_INTERRUPTS; // Глобально разрешаем прерывания

  for (;;) {
    for (; !ds18b20_reset();) {
      if (retries == 0) {
        // Тревога!
        OUTPUT_MODE(DDRD, PD2);
        HIGH(PORTD, PD2);
        continue;
      }

      retries -= 1;
      _delay_us(10000);
      _delay_us(10000);
    }

    ds18b20_write(0xCC); // Проверка кода датчика
    ds18b20_write(0x44); // Запуск температурного преобразования

    _delay_ms(750); // Задержка на опрос датчика

    for (; !ds18b20_reset();) {
      if (retries == 0) {
        // Тревога!
        OUTPUT_MODE(DDRD, PD2);
        HIGH(PORTD, PD2);
        continue;
      }

      retries -= 1;
      _delay_us(10000);
      _delay_us(10000);
    }

    ds18b20_write(0xCC); // Проверка кода датчика
    ds18b20_write(0xBE); // Считываем содержимое ОЗУ

    Temp_LSB = ds18b20_read(); // Читаем первые 2 байта блокнота
    Temp_MSB = ds18b20_read();

    // Вычисляем целое значение температуры
    buffer = ((Temp_MSB << 4) & 0x70) | (Temp_LSB >> 4);
    temp_int_1 = buffer % 100 / 10;
    temp_int_2 = buffer % 10;

    buffer = (Temp_LSB & 0x0F);
    // temp_point = buffer * 625 / 1000; // Точность
    // темпер.преобразования(0.0625)

    if (temp_int_1 >= 9 && temp_int_2 >= 9) {
      display_1 = 9;
      display_2 = 9;
    } else if (temp_int_1 == 0 && temp_int_2 == 0) {
    } else {
      // Выводим значения на дисплей
      display_1 = temp_int_1;
      display_2 = temp_int_2;
    }

    INPUT_MODE(DDRD, PD2);
    LOW(PORTD, PD2);
    retries = 255;
  }
}