// lab5_48_code.c 
// Andrew McCullough
// 11.13.2017

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions_m48.h"


//local temp variables
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
char lcd_string_local_temp[16];
uint16_t lm73_temp;


//*****************************************************************************
//	local_temp
void local_temp(){

	//read temperature data from LM73 (2 bytes)
	twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); 
	_delay_ms(2);

	//now assemble the two bytes read back into one 16-bit value
	lm73_temp = lm73_rd_buf[0];  //save high temperature byte into lm73_temp
	lm73_temp = lm73_temp << 8;  //shift it into upper byte 
	lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp
	lm73_temp = lm73_temp >> 7;

	itoa(lm73_temp, lcd_string_local_temp, 10);    	
}
//*****************************************************************************


int main()
{

	init_twi();
	uart_init();

	//local temp settings
	lm73_wr_buf[0] = 0x00;  
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);

	sei();

	while(1){

		if(uart_getc() != '\0'){

			local_temp();

			uart_puts(lcd_string_local_temp);
			uart_putc('\0'); 
		}

	}//while
}//main                    
