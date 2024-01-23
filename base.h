#ifndef BASE_H
#define BASE_H

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

typedef signed char  i8;
typedef signed short i16;
typedef signed int i32;
typedef signed long long i64;

typedef float f32;
typedef double f64;

#define INPUT_MODE(port, pin)   ((port) &= ~(1 << (pin)))   // Макрос для установки пина как вход
#define OUTPUT_MODE(port, pin)  ((port) |= (1 << (pin)))    // Макрос для установки пина как выход
#define LOW(port, pin)          ((port) &= ~(1 << (pin)))   // Макрос для установки пина в состояние "низкий уровень"
#define HIGH(port, pin)         ((port) |= (1 << (pin)))    // Макрос для установки пина в состояние "высокий уровень"
#define READ(port, pin)         ((port & (1 << pin)))       // Макрос для чтения данных с пина

#endif