#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

static inline void interrupts_enable(void) { sei(); }

static inline void interrupts_disable(void) { cli(); }

static inline void delay_us(uint32_t value) { _delay_us(value); }

static inline void delay_ms(uint32_t value) { _delay_ms(value); }

// GPIO

static inline void gpio_set_mode_input(volatile uint8_t *port, uint8_t pin) {
  *port &= ~(1 << pin);
}

static inline void gpio_set_mode_output(volatile uint8_t *port, uint8_t pin) {
  *port |= (1 << pin);
}

static inline void gpio_write_low(volatile uint8_t *port, uint8_t pin) {
  *port &= ~(1 << pin);
}

static inline void gpio_write_height(volatile uint8_t *port, uint8_t pin) {
  *port |= (1 << pin);
}

static inline uint8_t gpio_read(volatile uint8_t *port, uint8_t pin) {
  return *port & (1 << pin);
}

// Time

#define SECONDS(value) ((uint32_t)value * 1000UL)
#define MINUTES(value) ((uint32_t)value * 60UL * 1000UL)

typedef struct Timer32 {
  bool wait_done, sleep_pending;
  uint32_t ticks;
} Timer32;

static volatile uint32_t s_ticks;

static inline uint32_t get_ticks(void) { return s_ticks; }

static inline void timer_reset(Timer32 *timer) {
  memset(timer, 0, sizeof(Timer32));
}

static inline bool timer_expired(uint32_t *ticks, uint32_t period,
                                 uint32_t now_ticks) {
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

static inline bool timer_expired_ext(Timer32 *self, uint32_t wait,
                                     uint32_t period, uint32_t sleep_duration,
                                     uint32_t now_ticks) {
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
    self->ticks = now_ticks + period;
  }

  // Засыпание
  if (self->sleep_pending && now_ticks <= self->ticks) {
    return false;
  } else if (self->sleep_pending) {
    self->sleep_pending = false;
    self->ticks = now_ticks + period;
  }

  // Работа
  if (self->ticks >= now_ticks) {
    return true;
  }

  self->sleep_pending = true;
  self->ticks = now_ticks + sleep_duration;

  return false;
}

// System

static inline void system_tick_init(void) {
  // устанавливаем режим СТС (сброс по совпадению)
  TCCR1B |= (1 << WGM12);

  // Установка предделителя 8
  TCCR1B |= (1 << CS10);

  // Разрешение прерывания по сравнению
  TIMSK |= (1 << OCIE1A);

  // Установка значение для сравнения (1 мс для тактовой частоты 1 МГц и
  // предделителя 1)
  OCR1A = 999;

  interrupts_enable();
}

// Segments Display

ISR(TIMER1_COMPA_vect) { s_ticks += 1; }

int main(void) {
  system_tick_init();

  gpio_set_mode_output(&DDRC, PC0);
  gpio_set_mode_output(&DDRC, PC1);
  gpio_set_mode_output(&DDRC, PC2);
  gpio_set_mode_output(&DDRC, PC3);
  gpio_set_mode_output(&DDRC, PC4);
  gpio_set_mode_output(&DDRC, PC5);

  for (;;) {
    uint32_t now_ticks = get_ticks();

    {
      static Timer32 timer;

      if (timer_expired_ext(&timer, SECONDS(1), SECONDS(1), SECONDS(1),
                            now_ticks)) {
        gpio_write_height(&DDRC, PC0);
      } else {
        gpio_write_low(&DDRC, PC0);
      }
    }

    {
      static uint32_t ticks = 0;
      if (timer_expired(&ticks, MINUTES(1), now_ticks)) {
        gpio_write_height(&DDRC, PC1);
      }
    }

    {
      static uint32_t ticks = 0;
      if (timer_expired(&ticks, MINUTES(5), now_ticks)) {
        gpio_write_height(&DDRC, PC2);
      }
    }

    {
      static uint32_t ticks = 0;
      if (timer_expired(&ticks, MINUTES(99), now_ticks)) {
        gpio_write_height(&DDRC, PC2);
      }
    }

    delay_ms(1);
  }

  return 0;
}
