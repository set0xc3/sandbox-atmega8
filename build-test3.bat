@echo off

avr-gcc test3.c -o test3.elf -mmcu=atmega8 -Wall -O3 -std=c99
avr-objcopy test3.elf -O ihex test3.hex
avr-size test3.hex
