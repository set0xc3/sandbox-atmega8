#include <avr/io.h>
#include <util/delay.h>

#define PWM_PIN PB1 // Пин, к которому подключен светодиод
#define PWM_MAX 100 // Максимальное значение для яркости светодиода

void
PWM_Init()
{
  // Настройка режима Fast PWM для пина OC1A (PB1)
  TCCR1A |= (1 << COM1A1) | (1 << WGM10) | (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << CS11); // Предделитель настройки счетчика на 8
  DDRB |= (1 << PWM_PIN); // Установка пина OC1A на вывод
}

void
fadeOut()
{
  for (int brightness = PWM_MAX; brightness >= 0; brightness--) {
    OCR1A = brightness; // Установка яркости светодиода
    _delay_ms(1); // Задержка для плавного изменения яркости
  }
}

void
fadeIn()
{
  for (int brightness = 0; brightness <= PWM_MAX; brightness++) {
    OCR1A = brightness; // Установка яркости светодиода
    _delay_ms(1); // Задержка для плавного изменения яркости
  }
}

void
PWM_SetDutyCycle(uint8_t dutyCycle)
{
  OCR1A = dutyCycle; // Установка значения скважности ШИМ
}

int
main()
{
  PWM_Init(); // Инициализация ШИМ

  while (1) {
    fadeIn();  // Плавное включение
    fadeOut(); // Плавное выключение
    // _delay_ms(1000); // Задержка перед включением
  }

  return 0;
}
