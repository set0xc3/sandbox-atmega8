@echo off

avr-gcc dog.c -o dog.elf -mmcu=atmega8 -Wall -Os -std=c99
avr-objcopy dog.elf -O ihex dog.hex
avr-size dog.hex
