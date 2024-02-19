@echo off

avr-gcc test.c -o test.elf -mmcu=atmega8 -Wall -O3 -std=c99
avr-objcopy test.elf -O ihex test.hex
avr-size test.hex
