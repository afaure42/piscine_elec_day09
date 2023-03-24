#include "i2c.h"
#include "uart.h"

void i2c_init(void)
{
	//setting SCL frequency in the bitrate register
	TWBR = TWI_BITRATE;

	//setting no prescaler 
	TWSR = 0;
}

void i2c_stop()
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_wait(void)
{
	while((TWCR & (1 << TWINT)) == 0);
}

void i2c_start(uint8_t address, uint8_t write)
{
	TWCR = 1 << TWINT | 1 << TWSTA | 1 << TWEN; //sending start

	i2c_wait();

	TWDR = (address << 1) | (write & 1);

	TWCR = (1 << TWINT) | (1 << TWEN);
	i2c_wait();
}

uint8_t i2c_send_byte(uint8_t * buffer, uint8_t size)
{
	for(uint8_t i = 0; i < size && (TWSR & TW_STATUS)!= TW_MR_DATA_NACK; i++)
	{
		TWDR = buffer[i];

		TWCR = (1 << TWINT) | (1 << TWEN);
		i2c_wait();
	}
	return 0;
}

uint8_t i2c_read_byte(uint8_t * buffer, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
		TWCR = (1 << TWINT) | (1 << TWEN) | (i == size - 1 ? NACK : ACK);
		i2c_wait();

		buffer[i] = TWDR;
	}
	return 0;
}
