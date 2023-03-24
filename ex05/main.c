#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../i2c.h"
#include "../uart.h"

#define EXPANDER_ADDR 0b00100000

#define SW3 0
#define LED_D9  3
#define LED_D10 2
#define LED_D11 1
#define SEGMENT_DIGIT_0 4
#define SEGMENT_DIGIT 1 5
#define SEGMENT_DIGIT_2 6
#define SEGMENT_DIGIT_3 7

#define SELECT_INPUT_PORT0 0
#define SELECT_INPUT_PORT1 1
#define SELECT_OUTPUT_PORT0 2
#define SELECT_OUTPUT_PORT1 3
#define SELECT_CONFIG_PORT0 6
#define SELECT_CONFIG_PORT1 7

#define EXPANDER_REGISTER_0 0
#define EXPANDER_REGISTER_1 1


#define SEGMENT_DOT (0b10000000)

volatile uint8_t last_button_value = 0;
volatile uint8_t counter = 0;

volatile uint8_t segment_display_buffer[4];
volatile uint8_t segment_display_index = 0;
volatile const uint8_t segment_digit_pins[] =
{
	7,
	6,
	5,
	4,
};

volatile uint8_t segment_digits[] =
{
	0b00111111, //0
	0b00000110, //1
	0b01011011, //2
	0b01001111, //3
	0b01100110, //4
	0b01101101, //5
	0b01111101, //6
	0b00000111, //7
	0b01111111, //8
	0b01100111, //9
};


void exp_set_pin(uint8_t led_id, uint8_t status);
void expander_init();

void expander_segment_select_digit(uint8_t select_digit);

uint8_t expander_get_register(uint8_t select_register);
void expander_set_register(uint8_t select_register, uint8_t register_status);
void segment_putnbr(uint16_t nbr);
void clear_segment();

ISR(TIMER0_OVF_vect)
{
	// expander_segment_select_digit(segment_display_index);
	expander_set_register(SELECT_OUTPUT_PORT0, expander_get_register(SELECT_OUTPUT_PORT0) | 0b11110000);
	expander_set_register(SELECT_OUTPUT_PORT1, segment_display_buffer[segment_display_index]);
	expander_segment_select_digit(segment_display_index);
	segment_display_index++;
	if (segment_display_index == 4)
		segment_display_index = 0;
}

ISR(TIMER1_COMPA_vect)
{
	segment_putnbr(counter);
	counter++;
	if (counter > 9999)
		counter = 0;
}

void expander_init()
{
	i2c_start(EXPANDER_ADDR, TW_WRITE);

	uint8_t register_select;
	uint8_t register_status = 0;


	//set sw3 as input (1) and everything else as output (0)
	register_status = (1 << SW3);
	register_select = SELECT_CONFIG_PORT0;
	i2c_send_byte(&register_select, 1);
	i2c_send_byte(&register_status, 1);
	
	//set 7segment as output
	register_status = 0;
	i2c_send_byte(&register_status, 1); //config port 1 now

	//clear output registers
	i2c_start(EXPANDER_ADDR, TW_WRITE);
	register_status = 0xFF;
	register_select = SELECT_OUTPUT_PORT0;
	i2c_send_byte(&register_select, 1);
	i2c_send_byte(&register_status, 1); //first one will put 0 in register 0
	i2c_send_byte(&register_status, 1); //second one will automatically be put in register 1
	i2c_stop();

	segment_display_buffer[0] = 0x0;
	segment_display_buffer[1] = 0x0;
	segment_display_buffer[2] = 0x0;
	segment_display_buffer[3] = 0x0;

}

void expander_segment_select_digit(uint8_t select_digit)
{
	uint8_t status = expander_get_register(SELECT_OUTPUT_PORT0);

	status |= 0b11110000; //clear digits ( clearing is setting bits)

	status &= ~(1 << segment_digit_pins[select_digit]);

	expander_set_register(SELECT_OUTPUT_PORT0, status);
}

uint8_t expander_get_register(uint8_t select_register)
{
	uint8_t command;
	uint8_t ret = 0;
	command = select_register;
	i2c_start(EXPANDER_ADDR, TW_WRITE);
	i2c_send_byte(&command, 1);
	i2c_start(EXPANDER_ADDR, TW_READ);
	i2c_read_byte(&ret, 1);
	i2c_stop();
	return ret;
}

void expander_set_register(uint8_t select_register, uint8_t register_status)
{
	i2c_start(EXPANDER_ADDR, TW_WRITE);
	i2c_send_byte(&select_register, 1);
	i2c_send_byte(&register_status, 1);
	i2c_stop();
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
	temp = SELECT_OUTPUT_PORT0;
	i2c_send_byte(&temp, 1);
	i2c_send_byte(&register_status, 1);
	i2c_stop();
}

void timer_init(void)
{
	//just normla mode with 1024 prescaler
	TCCR0B = (1<< CS02);

	TIMSK0 = (1 << TOIE0);

	//setting timer 1 on a 1 second loop
	TCCR1B = (1 << CS12);
	TCCR1B |= (1 << WGM12);

	OCR1A = 62500;

	TIMSK1 = (1 << OCIE1A);
}

void segment_putnbr(uint16_t nbr)
{
	clear_segment();
	if (nbr > 999)
		segment_display_buffer[3] = segment_digits[nbr / 1000];
	if (nbr > 99)
		segment_display_buffer[2] = segment_digits[(nbr % 1000) / 100];
	if (nbr > 9)
		segment_display_buffer[1] = segment_digits[(nbr % 100) / 10];
	segment_display_buffer[0] = segment_digits[nbr % 10];
}

void segment_putnbr_fill_zero(uint16_t nbr)
{
	clear_segment();

	segment_display_buffer[3] = segment_digits[nbr / 1000];
	segment_display_buffer[2] = segment_digits[(nbr % 1000) / 100];
	segment_display_buffer[1] = segment_digits[(nbr % 100) / 10];
	segment_display_buffer[0] = segment_digits[nbr % 10];
}

void clear_segment()
{
	segment_display_buffer[0] = 0;
	segment_display_buffer[1] = 0;
	segment_display_buffer[2] = 0;
	segment_display_buffer[3] = 0;
}

int main()
{
	i2c_init();
	expander_init();
	timer_init();

	sei();
	for(;;)
	{
	}
}
