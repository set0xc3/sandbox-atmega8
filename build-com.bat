@echo off

avr-gcc com.c -o com.elf -mmcu=atmega8 -DF_CPU=16000000UL -Wall -g -Os -std=c99
avr-objcopy com.elf -O ihex com.hex
avr-size com.hex
