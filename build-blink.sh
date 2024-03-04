avr-gcc blink.c -o blink.elf -mmcu=atmega8 -DF_CPU=16000000UL -Wall -g -Os
avr-objcopy blink.elf -O ihex blink.hex
avr-size blink.hex
