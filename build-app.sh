#/bin/sh

# --param=min-pagesize=0 | https://gcc.gnu.org/bugzilla/show_bug.cgi?id=105523

avr-gcc app.c -o app.elf -mmcu=atmega8 -Wall -Os -std=gnu99 --param=min-pagesize=0
avr-objcopy app.elf -O ihex app.hex
avr-size app.hex
