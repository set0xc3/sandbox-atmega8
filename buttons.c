#define F_CPU 1000000UL

#include <avr/io.h>

#define BUTTON1_PIN PD5
#define BUTTON2_PIN PD6
#define BUTTON3_PIN PD7
#define LED_PIN PD2

void button1Task() {
    PORTD |= (1 << LED_PIN);  // Включение диода при нажатии кнопки 1
}

void button2Task() {
    // Ваш код для выполнения задачи при нажатии кнопки 2
}

void button3Task() {
    // Ваш код для выполнения задачи при нажатии кнопки 3
}

int main(void) {
    DDRD &= ~(1 << BUTTON1_PIN);  // Установка пина кнопки 1 на вход
    PORTD |= (1 << BUTTON1_PIN);  // Включение подтягивающего резистора на пине кнопки 1

    DDRD |= (1 << LED_PIN);  // Установка пина диода на выход

    while (1) {
        if (!(PIND & (1 << BUTTON1_PIN))) {
            button1Task();  // Выполнение задачи при нажатии кнопки 1
            while (!(PIND & (1 << BUTTON1_PIN)));  // Ждем, пока кнопка не будет отпущена
            PORTD &= ~(1 << LED_PIN);  // Выключение диода после отпускания кнопки 1
        }

        if (!(PIND & (1 << BUTTON2_PIN))) {
            button2Task();  // Выполнение задачи при нажатии кнопки 2
            while (!(PIND & (1 << BUTTON2_PIN)));  // Ждем, пока кнопка не будет отпущена
        }

        if (!(PIND & (1 << BUTTON3_PIN))) {
            button3Task();  // Выполнение задачи при нажатии кнопки 3
            while (!(PIND & (1 << BUTTON3_PIN)));  // Ждем, пока кнопка не будет отпущена
        }
    }

    return 0;
}