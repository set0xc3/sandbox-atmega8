@echo off

avr-gcc test2.c -o test2.elf -mmcu=atmega8 -Wall -O3 -std=c99
avr-objcopy test2.elf -O ihex test2.hex
avr-size test2.hex
