// BASE

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t  i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef float  f32;
typedef double f64;

// BASE

#define F_CPU 1000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

static inline void
interrupts_enable(void)
{
  sei();
}

static inline void
interrupts_disable(void)
{
  cli();
}

static inline void
delay_us(uint32_t value)
{
  _delay_us(value);
}

static inline void
delay_ms(uint32_t value)
{
  _delay_ms(value);
}

// GPIO

static inline void
gpio_set_mode_input(volatile uint8_t *port, uint8_t pin)
{
  *port &= ~(1 << pin);
}

static inline void
gpio_set_mode_output(volatile uint8_t *port, uint8_t pin)
{
  *port |= (1 << pin);
}

static inline void
gpio_write_low(volatile uint8_t *port, uint8_t pin)
{
  *port &= ~(1 << pin);
}

static inline void
gpio_write_height(volatile uint8_t *port, uint8_t pin)
{
  *port |= (1 << pin);
}

static inline uint8_t
gpio_read(volatile uint8_t *port, uint8_t pin)
{
  return *port & (1 << pin);
}

// Time

#define SECONDS(value) ((uint32_t)value * 1000UL)
#define MINUTES(value) ((uint32_t)value * 60UL * 1000UL)

typedef struct Timer32 {
  bool     wait_done, sleep_pending;
  uint32_t ticks;
} Timer32;

static volatile uint32_t s_ticks;

static inline uint32_t
get_ticks(void)
{
  return s_ticks;
}

static inline void
timer_reset(Timer32 *timer)
{
  memset(timer, 0, sizeof(Timer32));
}

static inline bool
timer_expired(uint32_t *ticks, uint32_t period, uint32_t now_ticks)
{
  if (ticks == 0) {
    return false;
  }

  // Если первый опрос, установить время завершения
  if (*ticks == 0) {
    *ticks = now_ticks + period;
  }

  if (now_ticks < *ticks) {
    return false;
  }

  *ticks = now_ticks + period;

  return true;
}

static inline bool
timer_expired_ext(Timer32 *self, uint32_t wait, uint32_t period,
                  uint32_t sleep_duration, uint32_t now_ticks)
{
  if (self == 0) {
    return false;
  }

  if (!self->wait_done) {
    // Если первый опрос, установить время завершения
    if (self->ticks == 0) {
      self->ticks = now_ticks + wait;
      return false;
    }

    if (now_ticks < self->ticks) {
      return false;
    }

    self->wait_done = true;
    self->ticks     = now_ticks + period;
  }

  // Засыпание
  if (self->sleep_pending && now_ticks <= self->ticks) {
    return false;
  } else if (self->sleep_pending) {
    self->sleep_pending = false;
    self->ticks         = now_ticks + period;
  }

  // Работа
  if (self->ticks >= now_ticks) {
    return true;
  }

  self->sleep_pending = true;
  self->ticks         = now_ticks + sleep_duration;

  return false;
}

#define ARRAY_COUNT(a) (sizeof((a)) / sizeof(*(a)))

#define CLAMP(value, min, max)                                                \
  ((value <= min) ? min : (value >= max) ? max : value)
#define CLAMP_TOP(value, max) ((value >= max) ? max : value)

// Пины для кнопок
#define PIN_BUTTON_UP   PD5
#define PIN_BUTTON_MENU PD6
#define PIN_BUTTON_DOWN PD7
#define PIN_BUTTON_READ PIND
#define PIN_BUTTON_DDR  DDRD
#define PIN_BUTTON_PORT PORTD

// Пины для индикации
#define PIN_LED_STOP     PC0
#define PIN_LED_RASTOPKA PC1
#define PIN_LED_CONTROL  PC2
#define PIN_LED_ALARM    PC3
#define PIN_LED_PUMP     PC4
#define PIN_LED_FAN      PC5
#define PIN_LED_DDR      DDRC
#define PIN_LED_PORT     PORTC

// Пины для термодатчика
#define PIN_OW      PD2
#define PIN_OW_READ PIND
#define PIN_OW_DDR  DDRD
#define PIN_OW_PORT PORTD

// Пины для вентилятора
#define PIN_FAN      PD3
#define PIN_FAN_DDR  DDRD
#define PIN_FAN_PORT PORTD

// Массив значениий для семисегментного индикатора
static char display_segment_numbers[12] = {
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

static char display_segment_menu[10][2] = {
  { 0b10011100, 0b11001111 }, // CP
  { 0b11001110, 0b11001111 }, // PP
  { 0b11111100, 0b00111111 }, // Ob
  { 0b11111100, 0b11001111 }, // OP
  { 0b00011110, 0b11001111 }, // TP
  { 0b01101110, 0b00001101 }, // HI
  { 0b00011110, 0b11111101 }, // TO
  { 0b00011110, 0b01111101 }, // TU
  { 0b00111110, 0b01111101 }, // bU
  { 0b01111100, 0b10001111 }, // UF
};

typedef enum Error {
  Error_None             = 0,
  Error_Temp_Sensor      = 1 << 0, // 00000001
  Error_Low_Temperature  = 1 << 1, // 00000010
  Error_High_Temperature = 1 << 2, // 00000100
} Error;

typedef enum Mode {
  MODE_STOP,
  MODE_RASTOPKA,
  MODE_CONTROL,
} Mode;

typedef enum State {
  STATE_HOME = 0,
  STATE_MENU,
  STATE_MENU_TEMP_CHANGE,
  STATE_MENU_PARAMETERS,
  STATE_ALARM,
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

typedef enum Temp_Step {
  Temp_Step_Convert,
  Temp_Step_Read,
  Temp_Step_Done,
} Temp_Step;

typedef struct Temp_Ctx {
  // u32 temp_point; // Переменная для дробного значения температуры
  u32       last_temp, temp;
  Temp_Step step;
  Timer32   timer;
} Temp_Ctx;

typedef struct Option {
  u8 value, min, max;
} Option;

#define OPTIONS_MAX 10

// Структура для хранения параметров меню
typedef union Options {
  struct {
    Option fan_work_duration;  // CP - ПРОДУВКА РАБОТА (Сек.)
    Option fan_pause_duration; // PP - ПРОДУВКА ПЕРЕРЫВ (Мин.)
    Option fan_speed; // Ob - СКОРОСТЬ ОБОРОТОВ ВЕНТИЛЯТОРА
    Option fan_power_during_ventilation; // OP - ОБОРОТЫ ВЕНТИЛЯТОРА ВО ВРЕМЯ
                                         // ПРОДУВКИ
    Option pump_connection_temperature; // ТЕМПЕРАТУРА ПОДКЛЮЧЕНИЯ НАСОСА ЦО
    Option hysteresis;          // HI - ГИСТЕРЕЗИС
    Option fan_power_reduction; // tO – УМЕНЬШЕНИЕ СИЛЫ ПРОДУВКИ
    Option controller_shutdown_temperature; // tU – ТЕМПЕРАТУРА ОТКЛЮЧЕНИЯ
                                            // КОНТРОЛЛЕРА
    Option
        sound_signal_enabled; // bU – ВКЛЮЧЕНИЕ И ОТКЛЮЧЕНИЕ ЗВУКОВОГО СИГНАЛА
    Option factory_settings;  // Uf – ЗАВОДСКИЕ НАСТРОЙКИ
  };

  Option e[OPTIONS_MAX];
} Options; // 10-bytes

// Глобальные переменные
static u8 display_enable = 1;

static u8 menu_idx          = CP;
static u8 menu_seconds      = 0; // 2 сек. до входа в меню
static u8 menu_timer_enable = 0;

static Error   error_flags = Error_None;
static Mode    mode        = MODE_STOP;
static State   last_state, state = STATE_HOME;
static Options options;
static Option  option_temp_target;

// Temp
static Temp_Ctx temp_ctx;

static u8 buttons[BUTTON_COUNT];
static u8 last_buttons[BUTTON_COUNT];

static u32 prev_time = 0;

// Прототипы функций
static void init_io(void);
static bool get_temp(Temp_Ctx *self);
static void display_menu(u8 display1, u8 display2);
static void handle_buttons(void);
static void startup_alarm(void);

static inline void
change_state(u8 new_state)
{
  last_state = state;
  state      = new_state;
}

static inline void
restore_last_state(void)
{
  state = last_state;
}

static void options_reset(void);
static void options_change_params(i8 value);

static u8 button_pressed(u8 code);
static u8 button_released(u8 code);
static u8 button_down(u8 code);

static u8   ow_reset(void);
static u8   ow_read(void);
static u8   ow_read_bit(void);
static void ow_send_bit(u8 bit);
static void ow_send(u8 data);
static bool ow_skip(void);
static u8   ow_crc_update(u8 crc, u8 byte);

#define LEDS_MAX 6

typedef enum Leds {
  Leds_Stop     = PIN_LED_STOP,
  Leds_Rastopka = PIN_LED_RASTOPKA,
  Leds_Control  = PIN_LED_CONTROL,
  Leds_Alarm    = PIN_LED_ALARM,
  Leds_Pump     = PIN_LED_PUMP,
  Leds_Fan      = PIN_LED_FAN
} Leds;

static bool leds_list[LEDS_MAX];

static void leds_init(void);
static void leds_display(Leds led);
static void leds_change(Leds led, bool enable);
static void leds_off(void);

static Timer32 timer_cp;
static Timer32 timer_pp;
static Timer32 timer_controller_shutdown_temperature;
static Timer32 timer_menu;
static Timer32 timer_temp_alarm;
static Timer32 timer_ow_alarm;

static inline void menu_button(u8 code, i8 value);
static inline void menu_parameters_button(u8 code, i8 value);
static inline void menu_change_temp_button(u8 code, i8 value);

// System

static inline void
system_tick_init(void)
{
  // Настройка Timer1
  {
    // устанавливаем режим СТС (сброс по совпадению)
    TCCR1B |= (1 << WGM12);

    // Установка предделителя 8
    TCCR1B |= (1 << CS10);

    // Разрешение прерывания по сравнению
    TIMSK |= (1 << OCIE1A);

    // Установка значение для сравнения (1 мс для тактовой частоты 1 МГц и
    // предделителя 1)
    OCR1A = 999;
  }

  // Настройка Timer2
  {
    // Настройка предделителя и запуск таймера
    TCCR2 = (1 << WGM21) | (1 << CS22) | (1 << CS21)
            | (1 << CS20); // Prescaler 1024
    TIMSK |= (1 << OCIE2);
    OCR2 = 4; // 5ms for 1MHz clock
  }

  // Разрешение глобальных прерываний
  interrupts_enable();
}

static inline void
fan_enable(void)
{
  leds_change(Leds_Control, false);
  leds_change(Leds_Rastopka, true);
  leds_change(Leds_Fan, true);
  gpio_set_mode_output(&PIN_FAN_DDR, PIN_FAN);
  gpio_write_height(&PIN_FAN_PORT, PIN_FAN);
}

static inline void
fan_disable(void)
{
  gpio_set_mode_output(&PIN_FAN_DDR, PIN_FAN);
  gpio_write_low(&PIN_FAN_PORT, PIN_FAN);
  leds_change(Leds_Fan, false);
}

int
main(void)
{
  options_reset();

  init_io();
  system_tick_init();
  leds_init();

  do {
    if (!ow_reset()) {
      error_flags = Error_Temp_Sensor;
      startup_alarm();
    }
  } while (timer_expired_ext(&timer_ow_alarm, 0, SECONDS(1), 0, s_ticks));

  while (true) {
    _delay_ms(1);

    handle_buttons();

    if (ow_reset()) {
      timer_reset(&timer_ow_alarm);
      get_temp(&temp_ctx);
    } else {
      if (timer_expired_ext(&timer_ow_alarm, SECONDS(10), 0, 0, s_ticks)) {
        error_flags = Error_Temp_Sensor;
        startup_alarm();
      }
    }

    if (state == STATE_ALARM) {
      if (options.sound_signal_enabled.value) {
        // ...
      }
    }

    if (state != STATE_ALARM) {
      if (state == STATE_MENU_TEMP_CHANGE) {
        if (timer_expired_ext(&timer_menu, 0, 0, 250, s_ticks)) {
          display_enable ^= 1;
        }
      } else {
        display_enable = 1;
      }

      if (state == STATE_HOME) {
        if (options.factory_settings.value == 1) {
          options_reset();
        }
      }

      if (mode != MODE_STOP) {
        if (temp_ctx.temp > 90) {
          if (timer_expired_ext(&timer_temp_alarm, SECONDS(5), 0, 0,
                                s_ticks)) {
            error_flags = Error_High_Temperature;
            startup_alarm();
          }
        } else if (temp_ctx.temp < 90) {
          timer_reset(&timer_temp_alarm);
        }

        if (options.controller_shutdown_temperature.value > temp_ctx.temp) {
          if (timer_expired_ext(&timer_controller_shutdown_temperature,
                                MINUTES(5), 0, 0, s_ticks)) {
            error_flags = Error_Low_Temperature;
            startup_alarm();
          }
        }

        if (options.controller_shutdown_temperature.value < temp_ctx.temp) {
          timer_reset(&timer_controller_shutdown_temperature);
        }

        // Алгоритм работы
        if (temp_ctx.temp < 35) {
          // Вентилятор начнет работу в ручном режиме.
          mode = MODE_RASTOPKA;

          fan_enable();

          timer_reset(&timer_cp);
          timer_reset(&timer_pp);
        } else {
          // Вентилятор начнет работу в автоматическом режиме.
          if (temp_ctx.temp
              >= option_temp_target.value + options.hysteresis.value) {
            mode = MODE_CONTROL;
            leds_change(Leds_Control, true);
            leds_change(Leds_Rastopka, false);

            if (!timer_pp.wait_done && timer_cp.wait_done) {
              timer_reset(&timer_cp);
            }

            if (timer_expired_ext(
                    &timer_pp, MINUTES(options.fan_pause_duration.value),
                    MINUTES(options.fan_pause_duration.value),
                    MINUTES(options.fan_pause_duration.value), s_ticks)) {
              if (timer_expired_ext(
                      &timer_cp, 0, SECONDS(options.fan_work_duration.value),
                      SECONDS(options.fan_work_duration.value), s_ticks)) {
                fan_enable();
              } else {
                fan_disable();
              }
            } else {
              fan_disable();
            }
          } else if (temp_ctx.temp
                     <= option_temp_target.value - options.hysteresis.value) {
            mode = MODE_RASTOPKA;
            leds_change(Leds_Rastopka, true);
            leds_change(Leds_Control, false);

            if (timer_pp.wait_done) {
              timer_reset(&timer_cp);
              timer_reset(&timer_pp);
            }

            if (timer_expired_ext(
                    &timer_cp, 0, SECONDS(options.fan_work_duration.value),
                    SECONDS(options.fan_work_duration.value), s_ticks)) {
              fan_enable();
            } else {
              fan_disable();
            }
          }
        }

        if (temp_ctx.temp >= options.pump_connection_temperature.value) {
          leds_change(Leds_Pump, true);
        } else {
          leds_change(Leds_Pump, false);
        }
      }
    }

    leds_display(Leds_Stop);
    leds_display(Leds_Rastopka);
    leds_display(Leds_Control);
    leds_display(Leds_Alarm);
    leds_display(Leds_Pump);
    leds_display(Leds_Fan);
  }

  return 0;
}

void
init_io(void)
{
  // Настройка пинов для кнопок
  PIN_BUTTON_DDR &= ~(1 << PIN_BUTTON_MENU) | (1 << PIN_BUTTON_UP)
                    | (1 << PIN_BUTTON_DOWN);
  // PORTD |= (1 << PIN_BUTTON_MENU) | (1 << PIN_BUTTON_UP) | (1 <<
  // PIN_BUTTON_DOWN);

  // Настройка пинов для индикации
  PIN_LED_DDR |= (1 << PIN_LED_STOP) | (1 << PIN_LED_RASTOPKA)
                 | (1 << PIN_LED_CONTROL) | (1 << PIN_LED_ALARM)
                 | (1 << PIN_LED_PUMP) | (1 << PIN_LED_FAN);
}

bool
get_temp(Temp_Ctx *self)
{
  bool res = false;

  self->last_temp = self->temp;

  self->step = self->step == Temp_Step_Done ? Temp_Step_Convert : self->step;

  switch (self->step) {
  case Temp_Step_Convert: {
    if (ow_reset()) {
      ow_send(0xCC); // Проверка кода датчика
      ow_send(0x44); // Запуск температурного преобразования

      timer_reset(&self->timer);
      self->step = Temp_Step_Read;
    }
  } break;

  case Temp_Step_Read: {
    if (timer_expired_ext(&self->timer, 1000, 0, 0, s_ticks)) {
      if (ow_reset()) {
        u8 crc = 0;
        u8 scratchpad[8];

        ow_send(0xCC); // Проверка кода датчика
        ow_send(0xBE); // Считываем содержимое ОЗУ

        u8 i = 0;
        for (i = 0; i < 8; i++) {
          u8 b          = ow_read();
          scratchpad[i] = b;
          crc           = ow_crc_update(crc, b);
        }
        if (ow_read() == crc) {
          self->temp = ((scratchpad[1] << 4) & 0x70) | (scratchpad[0] >> 4);
          res        = true;
        }
#if 0
          temp = (Temp_LSB & 0x0F);
          temp_point = temp * 625 / 1000; // Точность
          темпер.преобразования(0.0625)
#endif
      }

      self->step = Temp_Step_Done;
    }
  } break;

  default:
    break;
  }

  return res;
}

void
display_menu(u8 display1, u8 display2)
{
  static u8 display_idx = 0;

  if (!display_enable) {
    gpio_write_low(&PORTD, PD0);
    gpio_write_low(&PORTD, PD1);
    return;
  }

  gpio_set_mode_output(&DDRD, PD0);
  gpio_set_mode_output(&DDRD, PD1);
  DDRB = 0xFF;

  switch (display_idx) {
  case 0:
    gpio_write_height(&PORTD, PD0);
    gpio_write_low(&PORTD, PD1);
    PORTB = ~(display1);
    break;
  case 1:
    gpio_write_low(&PORTD, PD0);
    gpio_write_height(&PORTD, PD1);
    PORTB = ~(display2);
    break;
  default:
    break;
  }

  display_idx = (display_idx + 1) % 2;
}

void
menu_button(u8 code, i8 value)
{
  static Timer32 timer;

  if (button_pressed(code)) {
    menu_seconds = 0;
    menu_idx
        = CLAMP(menu_idx + value == UINT8_MAX ? 0 : menu_idx + value, 0, 9);
  }

  if (button_released(code)) {
    timer_reset(&timer);
  }

  if (button_down(code)) {
    menu_seconds = 0;

    if (timer_expired_ext(&timer, 500, 0, 50, s_ticks)) {
      menu_idx
          = CLAMP(menu_idx + value == UINT8_MAX ? 0 : menu_idx + value, 0, 9);
    }
  }
}

void
menu_parameters_button(u8 code, i8 value)
{
  static Timer32 timer;

  if (button_pressed(code)) {
    menu_seconds = 0;
    options_change_params(value);
  }

  if (button_released(code)) {
    timer_reset(&timer);
  }

  if (button_down(code)) {
    menu_seconds = 0;
    if (timer_expired_ext(&timer, 500, 0, 10, s_ticks)) {
      options_change_params(value);
    }
  }
}

void
menu_change_temp_button(u8 code, i8 value)
{
  static Timer32 timer;

  if (button_pressed(code)) {
    menu_seconds = 0;
    option_temp_target.value
        = CLAMP(option_temp_target.value + value == UINT8_MAX
                    ? 0
                    : option_temp_target.value + value,
                option_temp_target.min, option_temp_target.max);
    change_state(STATE_MENU_TEMP_CHANGE);
    menu_timer_enable = 1;
  }

  if (button_released(code)) {
    timer_reset(&timer);
  }

  if (button_down(code)) {
    menu_seconds = 0;
    if (timer_expired_ext(&timer, 500, 0, 10, s_ticks)) {
      option_temp_target.value
          = CLAMP(option_temp_target.value + value == UINT8_MAX
                      ? 0
                      : option_temp_target.value + value,
                  option_temp_target.min, option_temp_target.max);
    }
  }
}

void
handle_buttons(void)
{
  memcpy(&last_buttons, &buttons, sizeof(last_buttons));

  buttons[BUTTON_UP]   = PIN_BUTTON_READ & (1 << PIN_BUTTON_UP);
  buttons[BUTTON_MENU] = PIN_BUTTON_READ & (1 << PIN_BUTTON_MENU);
  buttons[BUTTON_DOWN] = PIN_BUTTON_READ & (1 << PIN_BUTTON_DOWN);

  switch (state) {
  case STATE_HOME: {
    if (button_pressed(BUTTON_MENU)) {
      menu_timer_enable = 1;
      leds_off();
    } else if (button_released(BUTTON_MENU)) {
      menu_timer_enable = 0;

      if (state != STATE_MENU) {
        menu_seconds = 0;
      }

      if (last_state == STATE_HOME || last_state == STATE_ALARM) {
        if (mode == MODE_STOP) {
          mode = MODE_RASTOPKA;
          leds_off();
          leds_change(Leds_Rastopka, true);
        } else if (mode != MODE_STOP) {
          mode = MODE_STOP;
          leds_off();
          leds_change(Leds_Stop, true);
        }
        break;
      } else if (last_state == STATE_MENU_TEMP_CHANGE
                 || last_state == STATE_ALARM
                 || last_state == STATE_MENU_PARAMETERS
                 || last_state == STATE_MENU) {
        last_state = STATE_HOME;
      }
    }

    menu_change_temp_button(BUTTON_UP, 1);
    menu_change_temp_button(BUTTON_DOWN, -1);
  } break;

  case STATE_MENU: {
    if (button_pressed(BUTTON_MENU)) {
      menu_seconds = 0;
      change_state(STATE_MENU_PARAMETERS);
      break;
    }

    menu_button(BUTTON_UP, 1);
    menu_button(BUTTON_DOWN, -1);
  } break;

  case STATE_MENU_PARAMETERS: {
    if (button_pressed(BUTTON_MENU)) {
      menu_seconds = 0;
      change_state(STATE_MENU);
      break;
    }

    menu_parameters_button(BUTTON_UP, 1);
    menu_parameters_button(BUTTON_DOWN, -1);
  } break;

  case STATE_MENU_TEMP_CHANGE: {
    if (button_pressed(BUTTON_MENU)) {
      menu_timer_enable = 0;
      menu_seconds      = 0;
      change_state(STATE_HOME);
      break;
    }

    menu_change_temp_button(BUTTON_UP, 1);
    menu_change_temp_button(BUTTON_DOWN, -1);
  } break;

  case STATE_ALARM: {
    if (button_released(BUTTON_MENU)) {
      error_flags = Error_None;
      change_state(STATE_HOME);

      // disable sound alarm
      {
        // ...
      }
    }
    break;
  }
  }
}

void
startup_alarm(void)
{
  mode = MODE_STOP;
  change_state(STATE_ALARM);
  leds_off();
  leds_change(Leds_Stop, true);
  leds_change(Leds_Alarm, true);
  timer_reset(&timer_cp);
  timer_reset(&timer_pp);
  timer_reset(&timer_controller_shutdown_temperature);
  timer_reset(&timer_menu);
  timer_reset(&timer_temp_alarm);
  timer_reset(&timer_ow_alarm);
  menu_timer_enable = 0;
  menu_seconds      = 0;
  display_enable    = 1;
}

void
options_reset(void)
{
  options.fan_work_duration            = (Option){ 10, 5, 95 }; // 5-95 seconds
  options.fan_pause_duration           = (Option){ 3, 1, 99 };  // 1-99 minutes
  options.fan_speed                    = (Option){ 99, 30, 99 };    // 30-99
  options.fan_power_during_ventilation = (Option){ 90, 30, 99 };    // 30-99
  options.pump_connection_temperature  = (Option){ 40, 25, 70 };    // 25-70
  options.hysteresis                   = (Option){ 3, 1, 9 };       // 1-9
  options.fan_power_reduction          = (Option){ 5, 0, 10 };      // 0-10
  options.controller_shutdown_temperature = (Option){ 30, 25, 50 }; // 25-50
  options.sound_signal_enabled            = (Option){ 1, 0, 1 };    // 0-1
  options.factory_settings                = (Option){ 0, 0, 1 };    // 0-1
  option_temp_target                      = (Option){ 60, 35, 80 };
}

void
options_change_params(i8 value)
{
  options.e[menu_idx].value
      = CLAMP(options.e[menu_idx].value + value == UINT8_MAX
                  ? 0
                  : options.e[menu_idx].value + value,
              options.e[menu_idx].min, options.e[menu_idx].max);
}

u8
button_pressed(u8 code)
{
  return !last_buttons[code] && buttons[code];
}

u8
button_released(u8 code)
{
  return last_buttons[code] && !buttons[code];
}

u8
button_down(u8 code)
{
  return last_buttons[code] && buttons[code];
}

// Инициализация DS18B20
u8
ow_reset(void)
{
  bool res = false;

  res = ow_skip();
  if (res) {
    interrupts_disable();
    gpio_write_low(&PIN_OW_PORT, PIN_OW);
    gpio_set_mode_output(&PIN_OW_DDR, PIN_OW);
    _delay_us(640);
    gpio_set_mode_input(&PIN_OW_DDR, PIN_OW);
    _delay_us(80);
    interrupts_enable();
    res = !gpio_read(&PIN_OW_READ, PIN_OW);
    _delay_us(410);
  }

  return res;
}

u8
ow_read_bit(void)
{
  u8 res = 0;

  interrupts_disable();
  gpio_set_mode_output(&PIN_OW_DDR, PIN_OW);
  _delay_us(2);
  gpio_set_mode_input(&PIN_OW_DDR, PIN_OW);
  _delay_us(8);
  res = gpio_read(&PIN_OW_READ, PIN_OW);
  interrupts_enable();
  _delay_us(80);

  return res;
}

u8
ow_read(void)
{
  u8 r = 0;
  u8 p = 0;

  for (p = 8; p; p--) {
    r >>= 1;
    if (ow_read_bit()) {
      r |= 0x80;
    }
  }

  return r;
}

void
ow_send_bit(u8 bit)
{
  interrupts_disable();
  gpio_set_mode_output(&PIN_OW_DDR, PIN_OW);

  if (bit) {
    _delay_us(5);
    gpio_set_mode_input(&PIN_OW_DDR, PIN_OW);
    interrupts_enable();
    _delay_us(90);
  } else {
    _delay_us(90);
    gpio_set_mode_input(&PIN_OW_DDR, PIN_OW);
    interrupts_enable();
    _delay_us(5);
  }
}

void
ow_send(u8 byte)
{
  u8 p = 0;

  for (p = 8; p; p--) {
    ow_send_bit(byte & 1);
    byte >>= 1;
  }
}

bool
ow_skip(void)
{
  u8 retries = 80;

  interrupts_disable();
  gpio_set_mode_input(&PIN_OW_DDR, PIN_OW);
  interrupts_enable();

  do {
    if (--retries == 0) {
      return false;
    }
    _delay_us(1);
  } while (!gpio_read(&PIN_OW_READ, PIN_OW));

  return true;
}

// Обновляет значение контольной суммы crc применением всех бит байта b.
// Возвращает обновлённое значение контрольной суммы
u8
ow_crc_update(u8 crc, u8 byte)
{
  u8 p = 0;

  for (p = 8; p; p--) {
    crc = ((crc ^ byte) & 1) ? (crc >> 1) ^ 0b10001100 : (crc >> 1);
    byte >>= 1;
  }
  return crc;
}

void
leds_init(void)
{
  leds_off();
  leds_change(Leds_Stop, true);
}

void
leds_display(Leds led)
{
  if (leds_list[led]) {
    gpio_write_height(&PIN_LED_PORT, led);
  } else {
    gpio_write_low(&PIN_LED_PORT, led);
  }
}

void
leds_off(void)
{
  leds_list[Leds_Stop]     = false;
  leds_list[Leds_Rastopka] = false;
  leds_list[Leds_Control]  = false;
  leds_list[Leds_Alarm]    = false;
  leds_list[Leds_Pump]     = false;
  leds_list[Leds_Fan]      = false;
}

void
leds_change(Leds led, bool enable)
{
  leds_list[led] = enable;
}

ISR(TIMER1_COMPA_vect)
{
  prev_time = s_ticks;
  s_ticks += 1;

  if (menu_timer_enable) {
    if (s_ticks / 1000 > prev_time / 1000) {
      menu_seconds += 1;
    }

    if (menu_seconds >= 2 && state == STATE_HOME) {
      change_state(STATE_MENU);
    }

    if (menu_seconds >= 5 && state != STATE_HOME) {
      change_state(STATE_HOME);
      if (last_state == STATE_MENU_TEMP_CHANGE
          || last_state == STATE_MENU_PARAMETERS || last_state == STATE_MENU) {
        last_state = STATE_HOME;
      }
      menu_timer_enable = 0;
      menu_seconds      = 0;
    }
  } else {
    menu_seconds = 0;
  }
}

ISR(TIMER2_COMP_vect)
{
  switch (state) {
  case STATE_HOME:
    display_menu(display_segment_numbers[temp_ctx.temp % 100 / 10],
                 display_segment_numbers[temp_ctx.temp % 10]);
    break;
  case STATE_ALARM:
    if (error_flags == Error_Temp_Sensor) {
      display_menu(display_segment_numbers[10], display_segment_numbers[10]);
    } else {
      display_menu(display_segment_numbers[temp_ctx.temp % 100 / 10],
                   display_segment_numbers[temp_ctx.temp % 10]);
    }
    break;
  case STATE_MENU_TEMP_CHANGE:
    display_menu(display_segment_numbers[option_temp_target.value % 100 / 10],
                 display_segment_numbers[option_temp_target.value % 10]);
    break;
  case STATE_MENU:
    display_menu(display_segment_menu[menu_idx][0],
                 display_segment_menu[menu_idx][1]);
    break;
  case STATE_MENU_PARAMETERS:
    display_menu(display_segment_numbers[options.e[menu_idx].value % 100 / 10],
                 display_segment_numbers[options.e[menu_idx].value % 10]);
    break;
  default:
    break;
  }
}