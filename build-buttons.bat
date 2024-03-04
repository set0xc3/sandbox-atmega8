@echo off

avr-gcc buttons.c -o buttons.elf -mmcu=atmega8 -Wall -O3 -std=c99
avr-objcopy buttons.elf -O ihex buttons.hex
avr-size buttons.hex