@echo off

avr-gcc dog.c -o dog.elf -mmcu=atmega8 -Wall -Os
avr-objcopy dog.elf -O ihex dog.hex
avr-size dog.hex
