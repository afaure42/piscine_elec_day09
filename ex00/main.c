#include <avr/io.h>
#include <util/delay.h>
#include "../i2c.h"

#define EXPANDER_ADDR 0b00100000

#define LED_D9  3
#define LED_D10 2
#define LED_D11 1
#define SW3 0

#define COMMAND_INPUT_PORT0 0
#define COMMAND_INPUT_PORT1 1
#define COMMAND_OUTPUT_PORT0 2
#define COMMAND_OUTPUT_PORT1 3
#define COMMAND_CONFIG_PORT0 6
#define COMMAND_CONFIG_PORT1 7

#define EXPANDER_REGISTER_0 0
#define EXPANDER_REGISTER_1 1

void expander_init()
{
	i2c_start(EXPANDER_ADDR, TW_WRITE);

	uint8_t register_select;
	uint8_t register_status = 0;


	//set sw3 as input (1) and everything else as output (0)
	register_status = (1 << SW3);
	register_select = COMMAND_CONFIG_PORT0;
	i2c_send_byte(&register_select, 1);
	i2c_send_byte(&register_status, 1);
	
	//set 7segment as output
	register_status = 0;
	i2c_send_byte(&register_status, 1); //config port 1 now

	//clear output registers
	i2c_start(EXPANDER_ADDR, TW_WRITE);
	register_status = 0xFF;
	register_select = COMMAND_OUTPUT_PORT0;
	i2c_send_byte(&register_select, 1);
	i2c_send_byte(&register_status, 1); //first one will put 0 in register 0
	i2c_send_byte(&register_status, 1); //second one will automatically be put in register 1
	i2c_stop();
}

//warning you need to stop after calling this function
uint8_t expander_get_register(uint8_t register_id)
{
	uint8_t command;
	uint8_t ret = 0;

	command = COMMAND_OUTPUT_PORT0 + register_id;
	i2c_start(EXPANDER_ADDR, TW_WRITE);
	i2c_send_byte(&command, 1);
	i2c_start(EXPANDER_ADDR, TW_READ);
	i2c_read_byte(&ret, 1);
	return ret;
}

void exp_set_pin(uint8_t led_id, uint8_t status)
{
	uint8_t register_status;
	uint8_t temp;
	register_status = expander_get_register(EXPANDER_REGISTER_0);

	if (status == 0)
		register_status |= (1 << led_id);
	else
		register_status &= ~(1 << led_id);
	
	i2c_start(EXPANDER_ADDR, TW_WRITE);
	temp = COMMAND_OUTPUT_PORT0;
	i2c_send_byte(&temp, 1);
	i2c_send_byte(&register_status, 1);
	i2c_stop();
}

int main()
{
	i2c_init();
	expander_init();

	for(;;)
	{
		exp_set_pin(LED_D9, 1);
		_delay_ms(500);
		exp_set_pin(LED_D9, 0);
		_delay_ms(500);
	}
}
