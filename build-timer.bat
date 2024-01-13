@echo off

avr-gcc timer.c -o timer.elf -mmcu=atmega8 -Wall -O3 -std=c99
avr-objcopy timer.elf -O ihex timer.hex
avr-size timer.hex
