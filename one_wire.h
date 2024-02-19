#ifndef ONE_WIRE_H
#define ONE_WIRE_H

#define OW_CMD_CONVERT_TEMP         0x44
#define OW_CMD_READ_SCRATCHPAD      0xbe
#define OW_CMD_WRITE_SCRATCHPAD     0x4e
#define OW_CMD_COPY_SCRATCHPAD      0x48
#define OW_CMD_RECALL_EEPROM        0xb8
#define OW_CMD_READ_POWER_SUPPLY    0xb4
#define OW_CMD_SEARCH_ROM           0xf0
#define OW_CMD_READ_ROM             0x33
#define OW_CMD_MATCH_ROM            0x55
#define OW_CMD_SKIP_ROM             0xcc
#define OW_CMD_ALARM_SEARCH         0xec

#define OW_SEARCH_FIRST             0xff
#define OW_PRESENCE_ERR             0xff
#define OW_DATA_ERR                 0xfe
#define OW_LAST_DEVICE              0x00

#define OW_MODE_READ    0
#define OW_MODE_WRITE   1

#define OW_PORT PORTC
#define OW_DDR DDRC
#define OW_PIN PINC
#define OW_BIT PC0

#define OW_MAX_DEVICES 1

#define OW_THERM_DECIMAL_STEPS_12BIT 625 //.0625

#include "base.h"

typedef struct {
    u8 addr[OW_MAX_DEVICES][8];
    u8 count;
} OW_Devices;

void ow_set_mode(u8 mode);
u8   ow_reset(void);
void ow_write_bit(u8 bit);
void ow_write_byte(u8 byte);
u8   ow_read_bit(void);
u8   ow_read_byte(void);

#endif