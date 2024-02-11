// BASE

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

typedef signed char i8;
typedef signed short i16;
typedef signed int i32;
typedef signed long long i64;

typedef float f32;
typedef double f64;

typedef i8 b8;

#define false 0
#define true 1

#define PIN_MODE_INPUT(port, pin)                                              \
  ((port) &= ~(1 << (pin))) // Макрос для установки пина как вход
#define PIN_MODE_OUTPUT(port, pin)                                             \
  ((port) |= (1 << (pin))) // Макрос для установки пина как выход
#define PIN_STATE_LOW(port, pin)                                               \
  ((port) &=                                                                   \
   ~(1 << (pin))) // Макрос для установки пина в состояние "низкий уровень"
#define PIN_STATE_HIGH(port, pin)                                              \
  ((port) |=                                                                   \
   (1 << (pin))) // Макрос для установки пина в состояние "высокий уровень"
#define PIN_READ(port, pin)                                                    \
  ((port & (1 << pin))) // Макрос для чтения данных с пина

// BASE

#define F_CPU 1000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

#define ENABLE_INTERRUPTS() sei()
#define DISABLE_INTERRUPTS() cli()

#define ARRAY_COUNT(a) (sizeof((a)) / sizeof(*(a)))

#define CLAMP(value, min, max)                                                 \
  ((value <= min) ? min : (value >= max) ? max : value)
#define CLAMP_TOP(value, max) ((value >= max) ? max : value)

// Пины для кнопок
#define PIN_BUTTON_UP PD5
#define PIN_BUTTON_MENU PD6
#define PIN_BUTTON_DOWN PD7
#define PIN_BUTTON_READ PIND
#define PIN_BUTTON_DDR DDRD
#define PIN_BUTTON_PORT PORTD

// Пины для индикации
#define PIN_LED_STOP PC0
#define PIN_LED_RASTOPKA PC1
#define PIN_LED_CONTROL PC2
#define PIN_LED_ALARM PC3
#define PIN_LED_PUMP PC4
#define PIN_LED_FAN PC5
#define PIN_LED_DDR DDRC
#define PIN_LED_PORT PORTC

// Пины для термодатчика
#define PIN_OW PD2
#define PIN_OW_READ PIND
#define PIN_OW_DDR DDRD
#define PIN_OW_PORT PORTD

// Массив значениий для семисегментного индикатора
char display_segment_numbers[12] = {
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
    0b00000000, // пусто
};

char display_segment_menu[10][2] = {
    {0b10011100, 0b11001111}, // CP
    {0b11001110, 0b11001111}, // PP
    {0b11111100, 0b00111111}, // Ob
    {0b11111100, 0b11001111}, // OP
    {0b00011110, 0b11001111}, // TP
    {0b01101110, 0b00001101}, // HI
    {0b00011110, 0b11111101}, // TO
    {0b00011110, 0b01111101}, // TU
    {0b00111110, 0b01111101}, // bU
    {0b01111100, 0b10001111}, // UF
};

// Перечисление для определения состояний системы
typedef enum State {
  STATE_MAIN = 0,
  STATE_MENU,
  STATE_MENU_TEMP_CHANGE,
  STATE_MENU_PARAMETERS,
  STATE_HEATING,
  STATE_VENTILATION,
  STATE_ALARM,
  STATE_COUNT,
} State;

typedef enum Button {
  BUTTON_UP = 0,
  BUTTON_MENU,
  BUTTON_DOWN,
  BUTTON_COUNT,
} Button;

typedef enum Parameters {
  CP = 0,
  PP,
  OB,
  OP,
  TP,
  HI,
  TO,
  TU,
  BU,
  UF,
} Parameters;

// Структура для хранения параметров меню
typedef union Settings {
  struct {
    u8 ventilation_work_duration;  // CP - ПРОДУВКА РАБОТА
    u8 ventilation_pause_duration; // PP - ПРОДУВКА ПЕРЕРЫВ
    u8 fan_speed; // Ob - СКОРОСТЬ ОБОРОТОВ ВЕНТИЛЯТОРА
    u8 fan_power_during_ventilation; // OP - ОБОРОТЫ ВЕНТИЛЯТОРА ВО ВРЕМЯ
                                     // ПРОДУВКИ
    u8 pump_connection_temperature; // ТЕМПЕРАТУРА ПОДКЛЮЧЕНИЯ НАСОСА ЦО
    u8 hysteresis;          // HI - ГИСТЕРЕЗИС
    u8 fan_power_reduction; // tO – УМЕНЬШЕНИЕ СИЛЫ ПРОДУВКИ
    u8 controller_shutdown_temperature; // tU – ТЕМПЕРАТУРА ОТКЛЮЧЕНИЯ
                                        // КОНТРОЛЛЕРА
    u8 sound_signal_enabled; // bU – ВКЛЮЧЕНИЕ И ОТКЛЮЧЕНИЕ ЗВУКОВОГО СИГНАЛА
    u8 factory_settings;     // Uf – ЗАВОДСКИЕ НАСТРОЙКИ
  } p;

  u8 e[10];
} Settings; // 10-bytes

// Глобальные переменные
volatile u8 display_idx = 0;
volatile u8 display_enable = 1;

volatile Parameters menu_idx = CP;
volatile u8 menu_seconds = 0; // 2s
volatile u8 menu_timer_enable = 0;

volatile u8 buttons[BUTTON_COUNT] = {0};
volatile u8 last_buttons[BUTTON_COUNT] = {0};

volatile enum State state = STATE_MAIN;

volatile Settings settings = {0};

volatile u32 prev_time, time = 0;

volatile u8 temp = 0;
volatile u8 target_temp = 0;
volatile u8 Temp_MSB, Temp_LSB, OK_Flag, temp_flag = 0;
volatile u8 temp1, temp2 = 0;
// volatile u32 temp_point = 0; // Переменная для дробного значения температуры

// Прототипы функций
void init_IO(void);
void init_timer(void);
void read_temperature(void);
void display_menu(u8 display1, u8 display2);
void handle_buttons(void);

void settings_reset(void);
void settings_change_params(i8 value);

u8 button_pressed(u8 code);
u8 button_released(u8 code);
u8 button_down(u8 code);

u8 ds18b20_reset(void);
u8 ds18b20_read(void);
void ds18b20_write(u8 data);

int main(void) {
  settings_reset();

  init_IO();
  init_timer();

  while (1) {
    read_temperature();
    handle_buttons();
  }

  return 0;
}

void init_IO(void) {
  // Настройка пинов для кнопок
  PIN_BUTTON_DDR &=
      ~(1 << PIN_BUTTON_MENU) | (1 << PIN_BUTTON_UP) | (1 << PIN_BUTTON_DOWN);
  // PORTD |= (1 << PIN_BUTTON_MENU) | (1 << PIN_BUTTON_UP) | (1 <<
  // PIN_BUTTON_DOWN);

  // Настройка пинов для индикации
  PIN_LED_DDR |= (1 << PIN_LED_STOP) | (1 << PIN_LED_RASTOPKA) |
                 (1 << PIN_LED_CONTROL) | (1 << PIN_LED_ALARM) |
                 (1 << PIN_LED_PUMP) | (1 << PIN_LED_FAN);
}

void init_timer(void) {
  // Настройка Timer1
  TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler 8
  TIMSK |= (1 << OCIE1A);
  OCR1A = 125 - 1; // 1ms for 1MHz clock

  // Настройка Timer2
  // Настройка предделителя и запуск таймера
  TCCR2 =
      (1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
  TIMSK |= (1 << OCIE2);
  OCR2 = 4; // 5ms for 1MHz clock

  // Разрешение глобальных прерываний
  ENABLE_INTERRUPTS();
}

void read_temperature(void) {
  ds18b20_reset();
  ds18b20_write(0xCC); // Проверка кода датчика
  ds18b20_write(0x44); // Запуск температурного преобразования

  _delay_us(750); // Задержка на опрос датчика

  ds18b20_reset();
  ds18b20_write(0xCC); // Проверка кода датчика
  ds18b20_write(0xBE); // Считываем содержимое ОЗУ

  Temp_LSB = ds18b20_read(); // Читаем первые 2 байта блокнота
  Temp_MSB = ds18b20_read();

  // Вычисляем целое значение температуры
  temp = ((Temp_MSB << 4) & 0x70) | (Temp_LSB >> 4);
  temp1 = temp % 100 / 10;
  temp2 = temp % 10;

  // temp = (Temp_LSB & 0x0F);
  // temp_point = temp * 625 / 1000; // Точность
  // темпер.преобразования(0.0625)
}

void display_menu(u8 display1, u8 display2) {
  if (!display_enable) {
    DDRD = 0;
    PORTD = 0;
    DDRB = 0;
    return;
  }

  DDRD |= (1 << PD0) | (1 << PD1);
  PORTD = (1 << display_idx);
  DDRB = 0xFF;

  switch (display_idx) {
  case 0:
    PORTB = ~(display1);
    break;
  case 1:
    PORTB = ~(display2);
    break;
  default:
    break;
  }

  display_idx = (display_idx + 1) % 2;
}

void handle_buttons(void) {
  static u32 start_time = 0;

  memcpy(last_buttons, buttons, sizeof(last_buttons));

  buttons[BUTTON_UP] = PIN_BUTTON_READ & (1 << PIN_BUTTON_UP);
  buttons[BUTTON_MENU] = PIN_BUTTON_READ & (1 << PIN_BUTTON_MENU);
  buttons[BUTTON_DOWN] = PIN_BUTTON_READ & (1 << PIN_BUTTON_DOWN);

  switch (state) {
  case STATE_MAIN: {
    if (button_pressed(BUTTON_MENU)) {
      menu_timer_enable = 1;
    } else if (button_released(BUTTON_MENU)) {
      if (state != STATE_MENU) {
        menu_timer_enable = 0;
        menu_seconds = 0;
      }
    }

    if (button_pressed(BUTTON_UP)) {
      menu_seconds = 0;
      start_time = time;
      target_temp = target_temp < 80 ? target_temp + 1 : 80;
      state = STATE_MENU_TEMP_CHANGE;
      menu_timer_enable = 1;
    }
    if (button_pressed(BUTTON_DOWN)) {
      menu_seconds = 0;
      start_time = time;
      target_temp = target_temp > 35 ? target_temp - 1 : 35;
      state = STATE_MENU_TEMP_CHANGE;
      menu_timer_enable = 1;
    }
  } break;

  case STATE_MENU_TEMP_CHANGE: {
    if (button_pressed(BUTTON_MENU)) {
      menu_timer_enable = 0;
      menu_seconds = 0;
      state = STATE_MAIN;
    }

    if (button_pressed(BUTTON_UP)) {
      menu_seconds = 0;
      start_time = time;
      target_temp = target_temp < 80 ? target_temp + 1 : 80;
      state = STATE_MENU_TEMP_CHANGE;
      menu_timer_enable = 1;
    }
    if (button_pressed(BUTTON_DOWN)) {
      menu_seconds = 0;
      start_time = time;
      target_temp = target_temp > 35 ? target_temp - 1 : 35;
      state = STATE_MENU_TEMP_CHANGE;
      menu_timer_enable = 1;
    }

    if (button_down(BUTTON_UP)) {
      menu_seconds = 0;
      u32 press_duration = time - start_time;
      if (press_duration >= 800) {
        target_temp = target_temp < 80 ? target_temp + 1 : 80;
        _delay_ms(10);
      }
    }
    if (button_down(BUTTON_DOWN)) {
      menu_seconds = 0;
      u32 press_duration = time - start_time;
      if (press_duration >= 800) {
        target_temp = target_temp > 35 ? target_temp - 1 : 35;
        _delay_ms(10);
      }
    }
  } break;

  case STATE_MENU: {
    if (settings.p.factory_settings) {
      settings_reset();
    }

    if (button_pressed(BUTTON_MENU)) {
      menu_seconds = 0;
      state = STATE_MENU_PARAMETERS;
      break;
    }

    // BUTTON_UP
    {
      if (button_pressed(BUTTON_UP)) {
        menu_seconds = 0;
        start_time = time;
        menu_idx = menu_idx < 9 ? menu_idx + 1 : 9;
      }

      if (button_down(BUTTON_UP)) {
        menu_seconds = 0;
        u32 press_duration = time - start_time;
        if (press_duration >= 800) {
          menu_idx = menu_idx < 9 ? menu_idx + 1 : 9;
          _delay_ms(50);
        }
      }
    }

    // BUTTON_DOWN
    {
      if (button_pressed(BUTTON_DOWN)) {
        menu_seconds = 0;
        start_time = time;
        menu_idx = menu_idx > 0 ? menu_idx - 1 : 0;
      }

      if (button_down(BUTTON_DOWN)) {
        menu_seconds = 0;
        u32 press_duration = time - start_time;
        if (press_duration >= 800) {
          menu_idx = menu_idx > 0 ? menu_idx - 1 : 0;
          _delay_ms(50);
        }
      }
    }
  } break;

  case STATE_MENU_PARAMETERS: {
    if (button_pressed(BUTTON_MENU)) {
      menu_seconds = 0;
      state = STATE_MENU;
      break;
    }

    // BUTTON_UP
    {
      if (button_pressed(BUTTON_UP)) {
        menu_seconds = 0;
        start_time = time;
        settings_change_params(1);
      }
      if (button_down(BUTTON_UP)) {
        menu_seconds = 0;
        u32 press_duration = time - start_time;
        if (press_duration >= 800) {
          settings_change_params(1);
          _delay_ms(10);
        }
      }
    }

    // BUTTON_DOWN
    {
      if (button_pressed(BUTTON_DOWN)) {
        menu_seconds = 0;
        start_time = time;
        settings_change_params(-1);
      }
      if (button_down(BUTTON_DOWN)) {
        menu_seconds = 0;
        u32 press_duration = time - start_time;
        if (press_duration >= 800) {
          settings_change_params(-1);
          _delay_ms(10);
        }
      }
    }
  } break;
  }
}

void settings_reset(void) {
  settings.p.ventilation_work_duration = 10;       // 5-95
  settings.p.ventilation_pause_duration = 3;       // 1-99
  settings.p.fan_speed = 99;                       // 30-99
  settings.p.fan_power_during_ventilation = 90;    // 30-99
  settings.p.pump_connection_temperature = 40;     // 25-70
  settings.p.hysteresis = 3;                       // 0-5
  settings.p.fan_power_reduction = 5;              // 0-10
  settings.p.controller_shutdown_temperature = 30; // 25-50
  settings.p.sound_signal_enabled = 1;             // 0-1
  settings.p.factory_settings = 0;                 // 0-1
  target_temp = 59;
}

void settings_change_params(i8 value) {
  switch (menu_idx) {
  case CP:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 5, 95);
    break;
  case PP:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 1, 99);
    break;
  case OB:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 30, 99);
    break;
  case OP:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 30, 99);
    break;
  case TP:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 25, 70);
    break;
  case HI:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 0, 5);
    break;
  case TO:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 0, 10);
    break;
  case TU:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 25, 50);
    break;
  case BU:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 0, 1);
    break;
  case UF:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 0, 1);
    break;
  default:
    settings.e[menu_idx] = CLAMP(settings.e[menu_idx] + value, 0, 99);
    break;
  }
}

u8 button_pressed(u8 code) { return !last_buttons[code] && buttons[code]; }

u8 button_released(u8 code) { return last_buttons[code] && !buttons[code]; }

u8 button_down(u8 code) { return last_buttons[code] && buttons[code]; }

ISR(TIMER1_COMPA_vect) {
  static u32 start_time = 0;

  prev_time = time;

  time += 1;
  start_time += 1;

  if (state == STATE_MENU_TEMP_CHANGE) {
    if (start_time >= 250) {
      display_enable ^= 1;
      start_time = 0;
    }
  } else {
    display_enable = 1;
  }

  if (menu_timer_enable) {
    if (time / 1000 > prev_time / 1000) {
      menu_seconds += 1;
    }

    if (menu_seconds >= 2 && state == STATE_MAIN) {
      state = STATE_MENU;
    }

    if (menu_seconds >= 5 && state != STATE_MAIN) {
      state = STATE_MAIN;
      menu_timer_enable = 0;
      menu_seconds = 0;
    }
  } else {
    menu_seconds = 0;
  }
}

ISR(TIMER2_COMP_vect) {
  switch (state) {
  case STATE_MAIN:
    display_menu(display_segment_numbers[temp % 100 / 10],
                 display_segment_numbers[temp % 10]);
    break;
  case STATE_MENU_TEMP_CHANGE:
    display_menu(display_segment_numbers[target_temp % 100 / 10],
                 display_segment_numbers[target_temp % 10]);
    break;
  case STATE_MENU:
    display_menu(display_segment_menu[menu_idx][0],
                 display_segment_menu[menu_idx][1]);
    break;
  case STATE_MENU_PARAMETERS:
    display_menu(display_segment_numbers[settings.e[menu_idx] % 100 / 10],
                 display_segment_numbers[settings.e[menu_idx] % 10]);
    break;
  }
}

// Инициализация DS18B20
u8 ds18b20_reset(void) {
  DISABLE_INTERRUPTS();

  PIN_OW_PORT &= ~(1 << PIN_OW); // Устанавливаем низкий уровень
  PIN_OW_DDR |= (1 << PIN_OW); // выход

  ENABLE_INTERRUPTS();

  _delay_us(480);

  DISABLE_INTERRUPTS();

  PIN_OW_DDR &= ~(1 << PIN_OW); // вход
  _delay_us(60);

  OK_Flag = !(PIN_OW_READ & (1 << PIN_OW)); // Ловим импульс присутствия датчика

  ENABLE_INTERRUPTS();

  // если OK_Flag = 0 датчик подключен, OK_Flag = 1 датчик не подключен
  _delay_us(410);

  return OK_Flag;
}

// Функция чтения байта из DS18B20
u8 ds18b20_read(void) {
  u8 res = 0;
  for (u8 i = 0; i < 8; i++) {

    DISABLE_INTERRUPTS();

    PIN_OW_DDR |= (1 << PIN_OW); // выход
    _delay_us(2);
    PIN_OW_DDR &= ~(1 << PIN_OW); // вход
    _delay_us(8);
    res = res >> 1; // Следующий бит
    if (PIN_OW_READ & (1 << PIN_OW)) {
      res |= 0x80;
    }

    ENABLE_INTERRUPTS();

    _delay_us(60);
  }
  return res;
}

// Функция записи байта в DS18B20
void ds18b20_write(u8 data) {
  for (u8 i = 0; i < 8; i++) {

    DISABLE_INTERRUPTS();

    PIN_OW_DDR |= (1 << PIN_OW); // выход
    _delay_us(2);

    if (data & 0x01) {
      PIN_OW_DDR &= ~(1 << PIN_OW); // вход
    } else {
      PIN_OW_DDR |= (1 << PIN_OW); // выход
    }

    data = data >> 1; // Следующий бит
    _delay_us(60);

    PIN_OW_DDR &= ~(1 << PIN_OW); // вход

    ENABLE_INTERRUPTS();

    _delay_us(2);
  }
}