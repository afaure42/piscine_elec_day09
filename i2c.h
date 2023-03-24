#ifndef I2C_H
# define I2C_H

#include <avr/io.h>
#include <util/twi.h>

#define SCL_FREQUENCY 100000
#define TWI_BITRATE (((F_CPU / SCL_FREQUENCY) - 16) / 2)

#define ACK 1
#define NACK 0
#define TW_WRITE 0
#define TW_READ 1


void i2c_init(void);
void i2c_stop();
void i2c_wait(void);
void i2c_start(uint8_t address, uint8_t write);
uint8_t i2c_send_byte(uint8_t * buffer, uint8_t size);
uint8_t i2c_read_byte(uint8_t * buffer, uint8_t size);

#endif
