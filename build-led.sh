avr-gcc led.c -o led.elf -mmcu=atmega8 -DF_CPU=16000000UL -Wall -g -Os
avr-objcopy led.elf -O ihex led.hex
avr-size led.hex
