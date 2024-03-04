@echo off

avr-gcc led_s.c -o led_s.elf -mmcu=atmega8 -DF_CPU=16000000UL -Wall -g -Os -std=c99
avr-objcopy led_s.elf -O ihex led_s.hex
avr-size led_s.hex
