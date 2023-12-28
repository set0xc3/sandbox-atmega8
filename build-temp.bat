@echo off

avr-gcc temp.c -o temp.elf -mmcu=atmega8 -DF_CPU=16000000UL -Wall -g -Os -std=c99
avr-objcopy temp.elf -O ihex temp.hex
avr-size temp.hex
