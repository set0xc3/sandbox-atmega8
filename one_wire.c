#include "one_wire.h"

#define OW_SET_BIT(reg, bit)    reg |= (1 << bit)
#define OW_CLEAR_BIT(reg, bit)  reg &= ~(1 << bit)
#define OW_INVERT_BIT(reg, bit) reg ^= (1 << bit)

void ow_set_mode(u8 mode)
{
    if (mode) {
        OW_CLEAR_BIT(OW_PORT, OW_BIT);
        OW_SET_BIT(OW_DDR, OW_BIT);
    } else {
        OW_CLEAR_BIT(OW_PORT, OW_BIT);
        OW_CLEAR_BIT(OW_DDR, OW_BIT);
    }
}

u8 ow_reset(void) 
{ 
    ow_set_mode(OW_MODE_WRITE);
    _delay_us(480);
    ow_set_mode(OW_MODE_READ);
    _delay_us(60);

    return (OW_PORT & (1 << OW_BIT)); // 0 - OK, 1 - not OK   
}

void ow_write_bit(u8 bit) 
{
    ow_set_mode(OW_MODE_WRITE);
    _delay_us(1);

    if (bit) {
        ow_set_mode(OW_MODE_READ);
    } else {
        _delay_ms(60);
        ow_set_mode(OW_MODE_READ);
    }
}

void ow_write_byte(u8 byte) 
{
    for (u8 i = 0; i < 8; i += 1) {
        ow_write_bit(byte & 1);
        byte >>= 1;
    }
}

u8 ow_read_bit(void)
{
    u8 res = 0;

    ow_set_mode(OW_MODE_WRITE);
    _delay_ms(1);
    ow_set_mode(OW_MODE_READ);
    _delay_us(14);

    if (OW_PORT & (1 << OW_BIT)) {
        res = 1;
    }

    _delay_us(45);

    return res;
}

u8 ow_read_byte(void) 
{
    u8 res = 0;

    for (u8 i = 0; i < 8; i += 1) {
        res >>= 1;
        res |= (ow_read_bit() << 7);
    }

    return res;
}