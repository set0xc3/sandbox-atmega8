#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "base.h"

// Определения пинов для дисплеев
#define SEGMENT_PORT    PORTB
#define SEGMENT_DDR     DDRB

#define DISPLAY1_PIN    PD0
#define DISPLAY2_PIN    PD1

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

// Глобальные переменные для хранения данных на дисплеях
volatile u8 display1_data = 0;
volatile u8 display2_data = 0;

int main(void)
{
    {
        // Настройка Timer1
        TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, prescaler = 64
        OCR1A = 77; // Прерывание каждые 5 мс при F_CPU = 1 MHz и prescaler = 64
        TIMSK |= (1 << OCIE1A); // Разрешение прерывания от совпадения

        // Разрешение глобальных прерываний
        sei();
    }

    OUTPUT_MODE(DDRD, DISPLAY1_PIN);
    OUTPUT_MODE(DDRD, DISPLAY2_PIN);

    SEGMENT_DDR = 0xFF;

    // HIGH(PORTD, DISPLAY1_PIN);
    // PORTB = ~digitPatterns[0];

    while (1)
    {
        // we have a working timer

        if (display1_data < 9) {
            display1_data += 1;
        } else {
            display1_data = 0;
        }

        if (display2_data > 0) {
            display2_data -= 1;
        } else {
            display2_data = 9;
        }

        _delay_ms(500);
    }
}

ISR (TIMER1_COMPA_vect)
{
    static u8 display_selector = 0;

    // Выбор активного дисплея
    if (display_selector == 0) {
        LOW(PORTD, DISPLAY2_PIN);
        HIGH(PORTD, DISPLAY1_PIN);
        PORTB = ~digitPatterns[display1_data];
    } else {
        LOW(PORTD, DISPLAY1_PIN);
        HIGH(PORTD, DISPLAY2_PIN);
        PORTB = ~digitPatterns[display2_data];
    }

    // Переключение на следующий дисплей
    display_selector ^= 1;
}