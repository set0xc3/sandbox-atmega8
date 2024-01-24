#define F_CPU 1000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// Массив значениий для семисегментного индикатора
char SEGMENTE[] = {
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
};
unsigned char segcounter = 0;
volatile unsigned char display_1, display_2;

// Прерывание по переполнению T2, динамическая индикация
ISR(TIMER2_OVF_vect) {
  PORTB = 0xFF;
  PORTD = (1 << segcounter);

  switch (segcounter) {
  case 0:
    PORTB = ~(SEGMENTE[display_1]);
    break;
  case 1:
    PORTB = ~(SEGMENTE[display_2]);
    break;
  }
  if ((segcounter++) > 2)
    segcounter = 0;
}

unsigned char Temp_MSB, Temp_LSB, OK_Flag, temp_flag;

// Инициализация DS18B20
unsigned char DS18B20_init(void) {
  DDRC |= (1 << PC0);   // PC0 - выход
  PORTC &= ~(1 << PC0); // Устанавливаем низкий уровень
  _delay_us(490);
  DDRC &= ~(1 << PC0); // PC0 - вход
  _delay_us(68);
  OK_Flag = (PINC & (1 << PC0)); // Ловим импульс присутствия датчика
  // если OK_Flag = 0 датчик подключен, OK_Flag = 1 датчик не подключен
  _delay_us(422);
  return OK_Flag;
}

// Функция чтения байта из DS18B20
unsigned char read_18b20(void) {
  unsigned char i, data = 0;
  for (i = 0; i < 8; i++) {
    DDRC |= (1 << PC0); // PC0 - выход
    _delay_us(2);
    DDRC &= ~(1 << PC0); // PC0 - вход
    _delay_us(4);
    data = data >> 1; // Следующий бит
    if (PINC & (1 << PC0))
      data |= 0x80;
    _delay_us(62);
  }
  return data;
}

// Функция записи байта в DS18B20
void write_18b20(unsigned char data) {
  unsigned char i;
  for (i = 0; i < 8; i++) {
    DDRC |= (1 << PC0); // PC0 - выход
    _delay_us(2);
    if (data & 0x01)
      DDRC &= ~(1 << PC0);
    else
      DDRC |= (1 << PC0);
    data = data >> 1; // Следующий бит
    _delay_us(62);
    DDRC &= ~(1 << PC0); // PC0 - вход
    _delay_us(2);
  }
}

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

  unsigned int buffer;
  unsigned int temp_int_1,
      temp_int_2; // Переменные для целого значения температуры
  unsigned int temp_point; // Переменная для дробного значения температуры

  sei(); // Глобально разрешаем прерывания

  while (1) {
    // Если датчик не ответил выводим "минус" во всех разрядах
    if (OK_Flag)
      display_1 = display_2 = 0;

    DS18B20_init();    // Инициализация DS18B20
    write_18b20(0xCC); // Проверка кода датчика
    write_18b20(0x44); // Запуск температурного преобразования
    _delay_ms(1000);   // Задержка на опрос датчика
    DS18B20_init();    // Инициализация DS18B20
    write_18b20(0xCC); // Проверка кода датчика
    write_18b20(0xBE); // Считываем содержимое ОЗУ
    Temp_LSB = read_18b20(); // Читаем первые 2 байта блокнота
    Temp_MSB = read_18b20();
    temp_flag = 1; // Флаг знака температуры равен 1(плюс)

    // Вычисляем отрицательную температуру
    if (Temp_MSB &
        (1 << 3)) // Проверяем бит знака температуры на равенство единице
    {
      signed int temp;
      temp_flag = 0; // Флаг знака равен 0(минус)
      temp = (Temp_MSB << 8) | Temp_LSB;
      temp = -temp; // Переводим дополнительный код в прямой
      Temp_LSB = temp;
      Temp_MSB = temp >> 8;
    }

    // Вычисляем целое значение температуры
    buffer = ((Temp_MSB << 4) & 0x70) | (Temp_LSB >> 4);
    temp_int_1 = buffer % 100 / 10;
    temp_int_2 = buffer % 10;

    // Вычисляем дробное значение температуры
    if (temp_flag == 0)
      buffer = (Temp_LSB & 0x0F) + 1;
    else
      buffer = (Temp_LSB & 0x0F);
    temp_point = buffer * 625 / 1000; // Точность темпер.преобразования(0.0625)

    // Если флаг знака температуры равен 0, в первом разряде ставим минус
    if (temp_flag == 0)
      temp_int_1 = 0; // "минус"
    // Если первая цифра значения температуры меньше 1, то гасим 1-й разряд
    // индикатора
    if (temp_int_1 < 1)
      temp_int_1 = 0; // "пусто"
    // Если первая цифра погашена и вторая цифра значения температуры меньше 1,
    // то гасим 2-й разряд индикатора
    if (temp_int_1 == 10 && temp_int_2 < 1)
      temp_int_2 = 0; // "пусто"
    // Если вторая цифра значения температуры меньше 1 и знак равен "минус", то
    // гасим 2-й разряд индикатора
    if (temp_int_2 < 1 && temp_flag == 0)
      temp_int_2 = 0; // "пусто"

    // Выводим значения на дисплей
    display_1 = temp_int_1;
    display_2 = temp_int_2;
  }
}