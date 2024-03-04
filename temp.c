#include <avr/io.h>
#include <util/delay.h>

#define OW_PIN PD2

// Define segment patterns for numbers 0 to 9
const uint8_t digitPatterns[10] = {
  // a  b  c  d  e  f  g  dp
  0b11111100, // 0
  0b01100000, // 1
  0b11011010, // 2
  0b11110010, // 3
  0b01100110, // 4
  0b10110110, // 5
  0b10111110, // 6
  0b11100000, // 7
  0b11111110, // 8
  0b11110110  // 9
};

struct Tstruct {
    uint8_t no_answer;
    int8_t value;
    uint8_t pos_sign;
};

uint8_t DS18B20_init(uint8_t pin_number);
uint8_t read_18b20(uint8_t pin_number);
void write_18b20(uint8_t dat, uint8_t pin_number);
struct Tstruct read_temperature_ds18b20(uint8_t pin_number);



int main() {
    // Настройка пина как входа
    DDRD &= ~(1 << OW_PIN);
    // Включение подтягивающего резистора
    PORTD |= (1 << OW_PIN);
    
   DDRB = 0xFF;
    
   static struct Tstruct tempS = {(uint8_t)0, (int8_t)0, (uint8_t)0};
   
   while (1) {
      tempS = read_temperature_ds18b20(3);

      if (tempS.no_answer != 0) {
        PORTB = ~digitPatterns[9];
      } else {
        PORTB = ~digitPatterns[(uint8_t)tempS.value];
      }
   }

    return 0;
}

uint8_t DS18B20_init(uint8_t pin_number) { 
    
    uint8_t no_answer = 0;	 		
    
    PORTD &= ~(1 << pin_number); // LOW
    DDRD |= (1 << pin_number); // OUTPUT
    _delay_us(480);

    DDRD &= ~(1 << pin_number); // HIGH
    _delay_us(70);

    no_answer = (PIND & (1 << pin_number)); //0 - OK, 1 - not OK
    _delay_us(410);
    
    return no_answer;	
}

uint8_t read_18b20(uint8_t pin_number) {

    uint8_t dat = 0;
    for(uint8_t i = 0; i < 8; i++) {		
        DDRD |= (1 << pin_number);
	_delay_us(2);        
	DDRD &= ~(1 << pin_number);
	_delay_us(4);        
	dat = dat >> 1;	    
	if(PIND & (1 << pin_number)) {			
            dat |= 0x80; 
        }
	_delay_us(62);
    }	
    return dat;
}

void write_18b20(uint8_t dat, uint8_t pin_number) {

    for(uint8_t j = 0;j < 8;j++) {
        DDRD |= (1 << pin_number);
        _delay_us(2);      		
	if(dat & 0x01) {
            DDRD &= ~(1 << pin_number);	
        } else {
        DDRD |= (1 << pin_number);
        }
        dat = dat >> 1; 
        _delay_us(62); 
        DDRD &= ~(1 << pin_number);
        _delay_us(2); 
    }
}

struct Tstruct read_temperature_ds18b20(uint8_t pin_number) {
    
    struct Tstruct tempS = {(uint8_t)0, (int8_t)0, (uint8_t)0};
    uint8_t Temp_L;
    uint8_t Temp_H;
    uint8_t NotOK_flag = 0;
    uint8_t temp_sign;
    int8_t temp_int;
    
    NotOK_flag = DS18B20_init(pin_number);
    write_18b20(0xCC, pin_number);                     // check sensor code
    write_18b20(0x44, pin_number);     		// start temperature conversion
    
    _delay_ms(1000);
  
    NotOK_flag = DS18B20_init(pin_number);        	// initialization DS18B20
    write_18b20(0xCC, pin_number);     		// check sensor code
    write_18b20(0xBE, pin_number);     		// prepare reading
    Temp_L = read_18b20(pin_number);			// read the first 2 bytes of scratchpad
    Temp_H = read_18b20(pin_number); 
	
    temp_sign = 1;                                    // temperature sign flag is 1 (plus)	
    if(Temp_H &(1 << 3)) {    	                       // check the sign bit			
        signed int tmp = 0;
	temp_sign = 0;      			// sign flag is 0 (minus)
	tmp = (Temp_H << 8) | Temp_L;
	tmp = -tmp;
	Temp_L = tmp;
	Temp_H = tmp >> 8; 
    }		
    temp_int = (((Temp_H << 4) & 0x70)|(Temp_L >> 4)); // compute the integer value of the temperature
    
    tempS.no_answer = NotOK_flag;
    tempS.value = temp_int;
    tempS.pos_sign = temp_sign;
    
    return tempS;  
}