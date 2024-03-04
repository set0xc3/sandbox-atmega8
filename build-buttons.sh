avr-gcc buttons.c -o buttons.elf -mmcu=atmega8 -DF_CPU=16000000UL -Wall -g -Os
avr-objcopy buttons.elf -O ihex buttons.hex
avr-size buttons.hex
