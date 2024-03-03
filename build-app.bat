@echo off

avr-gcc app.c -o app.elf -mmcu=atmega8 -Wall -Os -std=c99
avr-objcopy app.elf -O ihex app.hex
avr-size app.hex
