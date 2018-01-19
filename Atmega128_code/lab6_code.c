// lab6_code.c 
// Andrew McCullough
// 11.29.2017

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

//define selects for digits
#define d4 0x00
#define d3 0x10
#define dc 0x20
#define d2 0x30
#define d1 0x40

//define segments
#define n0 0b11000000
#define n1 0b11111001
#define n2 0b10100100
#define n3 0b10110000
#define n4 0b10011001
#define n5 0b10010010
#define n6 0b10000010
#define n7 0b11111000
#define n8 0b10000000
#define n9 0b10010000
#define nn 0b11111111
#define nd 0b01111111

#define nc 0b11111100
#define na 0b11111011
#define nb 0b11111000


//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]; 

//time to display on LEDs
uint8_t time_hr = 0;
uint8_t time_min = 0;
uint8_t time_sec = 0;
uint8_t alarm_hr = 0;
uint8_t alarm_min = 0;
uint8_t alarm_sec = 0;
uint8_t snooze_count = 0;

//indicators on LED
//bit 0 colon, bit 1 alarm armed
uint8_t indicators = 0b00000000;

//array for button debounce memory
uint8_t button[8];

//ADC value
uint16_t adc_val;

//mode
//bit 7: 1 clock
//bit 6: 1 radio 
//bit 5: 1 alarm
//bit 4: 1 alarm on, 0 alarm off
//bit 3: 1 snoozing, 0 not snoozing
//bit 2: 1 alarming, 0 not alarming
//bit 1: 1 radio on, 0 radio off
uint8_t mode = 0b10000000; 

//local temp variables
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
char lcd_string_local_temp[16];
uint16_t lm73_temp;

//remote temp variables
char lcd_string_remote_temp[16];
volatile uint8_t rcv_rdy;
char rx_char;

//current and last encoder value
uint8_t enc_val;
uint8_t enc_last;

//radio variables
extern enum radio_band{FM, AM, SW};
extern volatile uint8_t STC_interrupt;
extern uint8_t  si4734_wr_buf[9];
extern uint8_t  si4734_rd_buf[9];
extern uint8_t  si4734_tune_status_buf[8];
volatile enum radio_band current_radio_band = FM;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

uint16_t current_fm_freq = 10630;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;

//Used in debug mode for UART1
char uart1_tx_buf[40];      //holds string to send to crt
char uart1_rx_buf[40];      //holds string that recieves data from uart   


//**************************************************************************
//	chk_buttons                                      
//debounce code extended for all buttons, stores state in array state[8]
uint8_t chk_buttons() {

	static uint16_t state[8] = {0,0,0,0,0,0,0,0};
	uint8_t i = 0;

	//Using debounce code from lab1, created loop to look through all buttons
	while (i < 8){
		state[i] = (state[i] << 1) | (! bit_is_clear(PINA, i)) | 0xE000;

		if (state[i] == 0xF000){
			button[i] = 1;
			return 1;
		}

		else{
			button[i] = 0;
		}

		i++;
	}
	return 0;
}
//**************************************************************************

//**************************************************************************
//	store_seg_data
//Gets digit and puts in segment data
void store_seg_data(uint8_t time_hr, uint8_t time_min, uint16_t current_fm_freq){

	segment_data[3] = time_min%10;
	segment_data[2] = (time_min%100 - segment_data[3])/10;
	segment_data[1] = time_hr%10;
	segment_data[0] = (time_hr%100 - segment_data[1])/10;

	segment_data[7] = alarm_min%10;
	segment_data[6] = (alarm_min%100 - segment_data[7])/10;
	segment_data[5] = alarm_hr%10;
	segment_data[4] = (alarm_hr%100 - segment_data[5])/10;

	uint16_t freq = (current_fm_freq/10);

	segment_data[11] = freq%10;
	segment_data[10] = (freq%100 - segment_data[11])/10;
	segment_data[9] = (freq%1000 - segment_data[10]*10 - segment_data[11])/100;
	segment_data[8] = (freq%10000 - segment_data[9]*100 - segment_data[10]*10 - segment_data[11])/1000;
}
//**************************************************************************

//**************************************************************************
//	display_led
//gets data for each segment and displays on led board, taking away leading 0s
//sends segments to PORTA and what digit to PORTB
void display_led(){

	switch (mode & 0b11100000){

		//display hrs and mins for time
		case(0b10000000):
			PORTA =  dec_to_7seg[segment_data[3]];
			PORTB =  d4;
			_delay_ms(.2);

			PORTA =  dec_to_7seg[segment_data[2]];
			PORTB =  d3;
			_delay_ms(.2);

			//decides what to light up on indicator led's
			switch (indicators){

				case(0b00000000): 
					PORTA = nn;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000001): 
					PORTA = nc;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000010): 
					PORTA = na;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000011): 
					PORTA = nb;
					PORTB = dc;
					_delay_ms(.2);
					break;
			}

			PORTA =  dec_to_7seg[segment_data[1]];
			PORTB =  d2;
			_delay_ms(.2);

			PORTA =  dec_to_7seg[segment_data[0]];
			PORTB =  d1;
			_delay_ms(.2);
			break;


			//display frequency
		case(0b01000000):
			PORTA =  dec_to_7seg[segment_data[11]];
			PORTB =  d4;
			_delay_ms(.2);

			PORTA =  dec_to_7seg[segment_data[10]]; 
			PORTA &= 0b01111111;
			PORTB =  d3;
			_delay_ms(.2);

			//decides what to light up on indicator led's
			switch (indicators){

				case(0b00000000): 
					PORTA = nn;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000001): 
					PORTA = nn;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000010): 
					PORTA = na;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000011): 
					PORTA = na;
					PORTB = dc;
					_delay_ms(.2);
					break;
			}

			PORTA =  dec_to_7seg[segment_data[9]];
			PORTB =  d2;
			_delay_ms(.2);

			if (segment_data[8] == 0){PORTA = nn;}
			else PORTA =  dec_to_7seg[segment_data[8]];
			PORTB =  d1;
			_delay_ms(.2);
			break;  


			//display hrs and mins for alarm
		case(0b00100000):
			PORTA =  dec_to_7seg[segment_data[7]];
			PORTB =  d4;
			_delay_ms(.2);

			PORTA =  dec_to_7seg[segment_data[6]];
			PORTB =  d3;
			_delay_ms(.2);

			//decides what to light up on indicator led's
			switch (indicators){

				case(0b00000000): 
					PORTA = nc;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000001): 
					PORTA = nc;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000010): 
					PORTA = nb;
					PORTB = dc;
					_delay_ms(.2);
					break;

				case(0b00000011): 
					PORTA = nb;
					PORTB = dc;
					_delay_ms(.2);
					break;
			}

			PORTA =  dec_to_7seg[segment_data[5]];
			PORTB =  d2;
			_delay_ms(.2);

			PORTA =  dec_to_7seg[segment_data[4]];
			PORTB =  d1;
			_delay_ms(.2);
			break;

	}    
	PORTB = 0x60;
}
//**************************************************************************

//**************************************************************************
//	spi_init
//initializes the spi
void spi_init(){

	DDRB |= 0x07;
	PORTB |= 0x01;
	DDRD |= 0x04;
	DDRE |= 0b11000000;

	DDRF |= 0x08;
	PORTF &= 0xF7;
	PORTB |= _BV(PB1);

	//master mode, clk low on idle, leading edge sample
	SPCR |= (1<<SPE) | (1<<MSTR); 
	SPSR |= (1<<SPI2X); 

}
//**************************************************************************

//**************************************************************************
//  spi_get
//get the spi data
uint8_t spi_get(){

	PORTE |= 0x80;
	PORTE &= ~0x40;
	PORTE |= 0x40;
	PORTE &= ~0x80;
	SPDR = 0x00;

	while (!(SPSR &(1<<SPIF))){}

	return ~SPDR;
}
//**************************************************************************

//**************************************************************************
//hardware reset of Si4734
//	radio_reset
void radio_reset(){
	PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	PORTE |=  (1<<PE2); //hardware reset Si4734 
	_delay_us(200);     //hold for 200us, 100us by spec         
	PORTE &= ~(1<<PE2); //release reset 
	_delay_us(30);      //5us required because of my slow I2C translators I suspect
	//Si code in "low" has 30us delay...no explaination
	DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

}
//**************************************************************************

//**************************************************************************
//  enc_get
//checks the previous encoder value against the new encoder value
//adds or subtracts amount based off the mode
void enc_get(){     

	enc_val = spi_get();

	if (enc_val != enc_last){

		//left knob
		switch (enc_val & 0b00000011){

			case (0b00000010):
				//left turn
				if ((enc_last & 0x03) == 0b00000000){
					current_fm_freq -= 20;
					if (current_fm_freq < 8810){current_fm_freq = 10790;}
					if ((mode & 0b00000010) == 0b00000010){fm_tune_freq();}
				} 
				//right turn
				else if ((enc_last & 0x03) == 0b00000011){
					current_fm_freq += 20;
					if (current_fm_freq > 10790){current_fm_freq = 8810;}
					if ((mode & 0b00000010) == 0b00000010){fm_tune_freq();}
				}
				break;
		}

		//right knob
		switch (enc_val & 0b00001100){

			case (0b00001000):
				//left turn
				if ((enc_last & 0x0C) == 0b00000000){
					if (OCR3A <= 0xE000){OCR3A += 0x1000;} 
				}
				//right turn
				else if ((enc_last & 0x0C) == 0b00001100){
					if (OCR3A >= 0x6000){OCR3A -= 0x1000;} 
				} 
				break;
		}
	}

	enc_last = enc_val;
}
//**************************************************************************

//**************************************************************************
//	button_cmd
//check the buttons and toggle the mode value based on what button
void button_cmd(){

	if (chk_buttons()){

		//8th button: time mode
		if (button[7] == 1){
			mode &= ~0b01100000;
			mode |=  0b10000000;
		}

		//7th button: radio mode
		if (button[6] == 1){
			mode &= ~0b10100000;
			mode |=  0b01000000;
		}

		//6th button: alarm mode
		if (button[5] == 1){
			mode &= ~0b11000000;
			mode |=  0b00100000;
		}

		//5th button: toggles alarm on and off
		if (button[4] == 1){
			mode ^= 1 << 4;
			indicators ^= 1 << 1;

			switch(mode & 0b00010000){  
				case (0b00010000): 
					set_cursor(1,0); 
					string2lcd("ALARM"); 
					break;
				case (0b00000000):
					set_cursor(1,0); 
					string2lcd("     "); 
					break;
			}		

		}

		//4th button: snooze mode, if alarm sounding
		if (button[3] == 1){
			if ((mode & 0b00000100) == 0b00000100){
				mode |= 0b00001000;
			}
		}

		//4th button: radio on and tune/off
		if (button[2] == 1){
			mode ^= 1 << 1;

			if ((mode & 0b00000010) == 0b00000010){
				radio_reset();
				fm_pwr_up();
				fm_tune_freq();
			}
			else if ((mode & 0b00000010) == 0b00000000){
				radio_pwr_dwn();
			}
		}

		//1st button: minutes
		//2nd button: hours
		//switch for hr and min setting
		switch (mode & 0b11100000){

			//time mode buttons
			case (0b10000000):
				if (button[0] == 1){
					time_min++;
					if (time_min == 60){time_min = 0;}
				}

				if (button[1] == 1){
					time_hr++;
					if (time_hr == 24){time_hr = 0;}
				}
				break;

				//radio mode buttons
			case (0b01000000):
				if (button[0] == 1){
					current_fm_freq = 10630;
					fm_tune_freq();
				}

				if (button[1] == 1){
					current_fm_freq = 10470;
					fm_tune_freq();
				}
				break; 

				//alarm mode buttons
			case (0b00100000):
				if (button[0] == 1){
					alarm_min++;
					if (alarm_min == 60){alarm_min = 0;}
				}

				if (button[1] == 1){
					alarm_hr++;
					if (alarm_hr == 24){alarm_hr = 0;}
				}
				break;
		}
	}
}
//**************************************************************************

//**************************************************************************
//	adc_get
//gets adc value
void adc_get(){

	ADCSRA |= (1<<ADSC);
	while (bit_is_clear(ADCSRA,ADIF)){}
	ADCSRA |= (1<<ADIF);
	adc_val = (ADC/50);
}
//**************************************************************************

//**************************************************************************
//	set_brightness
//sets the brightness of the led display
void set_brightness(){

	switch(adc_val){

		case(20): OCR2 = 0xFF; break;
		case(19): OCR2 = 0xFF; break;
		case(18): OCR2 = 0xFF; break;
		case(17): OCR2 = 0xFF; break;
		case(16): OCR2 = 0xDF; break;
		case(15): OCR2 = 0xBF; break;
		case(14): OCR2 = 0x9F; break;
		case(13): OCR2 = 0x7F; break;
		case(12): OCR2 = 0x5F; break;
		case(11): OCR2 = 0x3F; break;
		case(10): OCR2 = 0x1F; break;
		case(9): OCR2 = 0x0F; break;
		case(8): OCR2 = 0x0F; break;
		case(7): OCR2 = 0x06; break;
		case(6): OCR2 = 0x06; break;
		case(5): OCR2 = 0x00; break;
		case(4): OCR2 = 0x00; break;
		case(3): OCR2 = 0x00; break;
		case(2): OCR2 = 0x00; break;
		case(1): OCR2 = 0x00; break;
		case(0): OCR2 = 0x00; break;
	}
}
//**************************************************************************

//**************************************************************************
//	local_temp
void local_temp(){

	set_cursor(2,2);

	//read temperature data from LM73 (2 bytes)
	twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); 
	display_led();
	display_led();

	//now assemble the two bytes read back into one 16-bit value
	lm73_temp = lm73_rd_buf[0];  //save high temperature byte into lm73_temp
	lm73_temp = lm73_temp << 8;  //shift it into upper byte 
	lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp
	lm73_temp = lm73_temp >> 7;

	itoa(lm73_temp, lcd_string_local_temp, 10);    		
	string2lcd(lcd_string_local_temp);
}
//**************************************************************************

//**************************************************************************
//	remote_temp
//sends character to m48 to initiate temperature read
void remote_temp(){

	uart_putc('r');
} 
//**************************************************************************

//**************************************************************************
//	ISR USART0
//USART receive interrupt
ISR(USART0_RX_vect){


	static  uint8_t  i;
	rx_char = UDR0;              //get character
	lcd_string_remote_temp[i++]=rx_char;  //store in array 
	//if entire string has arrived, set flag, reset index
	if(rx_char == '\0'){
		rcv_rdy=1; 
		lcd_string_remote_temp[--i]  = (' ');     //clear the count field
		lcd_string_remote_temp[i+1]  = (' ');
		lcd_string_remote_temp[i+2]  = (' ');
		i=0;  
	}
}
//**************************************************************************

//**************************************************************************
//	tcnt0_init
//initialize tcnt0
void tcnt0_init(){

	TIMSK |= (1<<TOIE0);
	TCCR0 |= (1<<CS00);
	ASSR |= (1<<AS0);
}
//**************************************************************************

//**************************************************************************
//	tcnt1_init
//initialize tcnt1
//enable speaker output
void tcnt1_init(){

	DDRD |= 0b00001000;
	TCCR1A |= 0x00;
	TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS11);
	TCCR1C = 0x00;
	OCR1A = 0x0050;
	TIMSK &= ~(1<<OCIE1A); 	
}
//**************************************************************************

//**************************************************************************
//	tcnt1_on
//turn on tcnt1
//enable speaker output
void tcnt1_on(){

	TIMSK |= (1<<OCIE1A);
}
//**************************************************************************

//**************************************************************************
//	tcnt1_off
//turn off tcnt1
//disable speaker output
void tcnt1_off(){

	TIMSK &= ~(1<<OCIE1A); 	
}
//**************************************************************************

//**************************************************************************
//	tcnt2_init
//initialize tcnt2
void tcnt2_init(){

	TCCR2 |= (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS20);
	OCR2 = 0xFF;
}
//**************************************************************************

//**************************************************************************
//	tcnt3_init
//initialize tcnt3
void tcnt3_init(){

	DDRE |= (1<<PE3);
	TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31);
	TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS30);
	TCCR3C |= 0x00;
	OCR3A = 0xA000;
	ICR3 = 0xF000;
}
//**************************************************************************

//**************************************************************************
//	ISR 1
ISR(TIMER1_COMPA_vect){

	TCCR1A = 0;

	static uint8_t count = 0;
	count++;

	if (count % 1 == 0){PORTD = (1<<PD3);}	
	if (count % 2 == 0){PORTD = (0<<PD3);}	

	if(count % 100 == 0){OCR1A = 0x0080;}   
	if(count % 200 == 0){OCR1A = 0x0050;}   

}
//**************************************************************************

//**************************************************************************
//	radio_init
void radio_init(){

	//Rogers radio code
	DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
	DDRE  |= 0x08; //Port E bit 3 is TCNT3 PWM output for volume
	PORTE |= 0x04; //radio reset is on at powerup (active high)

	EICRB |= (1 << ISC70) | (1 <<ISC71);
	EIMSK |= (1<< INT7);   
}
//**************************************************************************

//**************************************************************************
//external interrupt for radio
//	ISR 7
ISR(INT7_vect){STC_interrupt = TRUE;}
//**************************************************************************

//**************************************************************************
//	ISR 0
ISR(TIMER0_OVF_vect){

	static uint8_t count = 0;
	count++;

	adc_get();
	set_brightness();

	if ((count % 64) == 0){

		//blink colon
		indicators ^= 1 << 0;
	}

	if ((count % 128) == 0){

		count = 0;
		time_sec++;

		local_temp();
		remote_temp();

		//bound minutes and hours
		if (time_sec == 60){time_sec = 0, time_min++;}
		if (time_min == 60){time_min = 0, time_hr++;}
		if (time_hr == 24){time_hr = 0;}

		//if alarm on, and times match, sound alarm
		if (((mode & 0b00010000) == 0b00010000)
				&& (time_hr == alarm_hr)
				&& (time_min == alarm_min)
				&& (time_sec == alarm_sec)){
			mode |= 0b00000100;
			tcnt1_on();
		}    

		//if alarm off, stop sounding alarm and stop snooze
		if ((mode & 0b00010000) == 0b00000000){
			mode &= ~0b00001100;
			tcnt1_off();
		}		    


		//if snooze pressed, increment count and stop sounding alarm
		if ((mode & 0b00001000) == 0b00001000){
			snooze_count++;
			mode &=  ~0b00000100;
			tcnt1_off();
		}

		//if snooze done, turn off snooze and sound alarm again
		if (snooze_count == 10){
			mode &= ~0b00001000;
			mode |= 0b00000100;
			snooze_count = 0;
			tcnt1_on();
		} 
	}
}
//**************************************************************************


int main()
{

	//initialize decimal to 7 segment array
	dec_to_7seg[0] = n0;
	dec_to_7seg[1] = n1;
	dec_to_7seg[2] = n2;
	dec_to_7seg[3] = n3;
	dec_to_7seg[4] = n4;
	dec_to_7seg[5] = n5;
	dec_to_7seg[6] = n6;
	dec_to_7seg[7] = n7;
	dec_to_7seg[8] = n8;
	dec_to_7seg[9] = n9;

	//set port bits 4-7 B as outputs
	DDRB = 0b11110000;

	//initialize tcnt0, spi, and set interrupts
	tcnt0_init();
	tcnt1_init();
	tcnt2_init();
	tcnt3_init();
	spi_init();
	lcd_init();
	init_twi();
	uart_init();
	radio_init();
	sei();

	clear_display();
	cursor_home();	

	//Initialize the ADC
	DDRF  &= ~(_BV(DDF7));  
	PORTF &= ~(_BV(PF7));	
	ADMUX = 0b01000111;
	ADCSRA = 0b10000111;

	//local temp settings
	lm73_wr_buf[0] = 0x00;  
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);

	set_cursor(2,0);
	string2lcd("L:");

	set_cursor(2,5);
	string2lcd("R:");


	while(1){

		if(rcv_rdy==1){

			set_cursor(2,7);
			string2lcd(lcd_string_remote_temp);  //write out string if its ready
			rcv_rdy=0;
		}

		display_led();

		//make PORTA an input port with pullups
		DDRA = 0x00;
		PORTA = 0xFF;

		//enable tristate buffer for pushbutton switches
		PORTB |= 0b01110000;

		//delay for tristate buffer to fully activate
		_delay_ms(.1);

		button_cmd();    

		//disable tristate buffer for pushbutton switches
		PORTB &= ~0b01110000;

		//make PORTA an output
		DDRA = 0xFF;

		//Get each digit of the decimal number and store in segment data array
		store_seg_data(time_hr, time_min, current_fm_freq);

		//show number on LED	
		display_led();

		enc_get();

	}//while
}//main                    
