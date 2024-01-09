#define F_CPU 8000000UL
//#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "led_reg.h"

#define SWSPI_PORT PORTD
#define SWSPI_REG DDRD
#define SWSPI_PIN PIND
#define SWSPI_CLK 5
#define SWSPI_MOSI 6
#define SWSPI_CS 7

#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))

#define FSM_IDLE 0x00
#define FSM_SIZE_AND_NUMBER_BYTE 0x01
#define FSM_ADDR_PAYLOAD_BYTE 0x02
#define FSM_SET_ADDRESS_BYTE 0x03

#define SERVICE_ADDRES 0xFE

//#define UART_BAUD 9600L
#define UART_BAUD 38400L
#define prescaler ((F_CPU/(16*UART_BAUD))-1)
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)

volatile uint8_t in_data = 0x0;
volatile uint8_t reg_address = 0x10;
volatile uint8_t line_count = 0x8; //static
volatile uint8_t line_width = 0x8;
volatile uint8_t led_data[8] = {0x01, 0x02, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80};
volatile uint8_t fsm_spi_slave_state = FSM_IDLE;

volatile uint8_t preset_width = 0;
volatile uint8_t preset_data_pointer = 0;

volatile uint8_t show_flag = 0;
volatile uint8_t spi_dbg_buf[50] = {0x0};
volatile uint8_t spi_dbg_buf_ptr = 0;

void spi_init_slave (void)
{
	DDRB = (1<<PINB4);               //MISO as OUTPUT
 	SPCR |= (1<<SPE)|(1<<SPIE)|(0<<CPOL)|(0<<CPHA);       //Enable SPI && interrupt enable bit
 	SPDR=0;
}

void spi_init_sw_master(void){
	sbi(SWSPI_REG, SWSPI_CLK);
	sbi(SWSPI_REG, SWSPI_MOSI);
	sbi(SWSPI_REG, SWSPI_CS);

	cbi(SWSPI_PORT, SWSPI_CLK);
	cbi(SWSPI_PORT, SWSPI_MOSI);
	sbi(SWSPI_PORT, SWSPI_CS);
}

void timer_init(void){
	timer_disable();
	TIMSK|=(1<<TOIE0);
}
void timer_disable(void){
	TCCR0 = (0<<CS02) | (0<<CS00);
	TCNT0 = 0;
}
void timer_enable(void){
	TCCR0 = (1<<CS02) | (1<<CS00);
}


void usart_init(void){
	UBRRL = LO(prescaler);
	UBRRH = HI(prescaler);
	UCSRA = 0;
	UCSRB = (0<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);

}
void usart_send_byte(uint8_t b){
	while( !(UCSRA & (1<<UDRE)) );
	UDR = b;
}

void send_led_data(void){
	for(uint8_t i = 0; i<line_count; i++){
		cbi(SWSPI_PORT, SWSPI_CS);
		uint8_t pos = (0xFF & ~(1 << (i)));
		for (uint8_t p = 0; p<8; p++){
			//if ((pos<<p)&0x80){
			if ((pos>>p)&0x01){
				sbi(SWSPI_PORT, SWSPI_MOSI);
			}else {	
				cbi(SWSPI_PORT, SWSPI_MOSI);
			}
			_delay_us(50);
			 sbi(SWSPI_PORT, SWSPI_CLK);
			_delay_us(50);
			 cbi(SWSPI_PORT, SWSPI_CLK);
		}	

		for (uint8_t k = 0; k<line_width; k++){
			//if ((led_data[i]<<k)&0x80){
			if ((led_data[i]>>k)&0x01){
				sbi(SWSPI_PORT, SWSPI_MOSI);
			}else {	
				cbi(SWSPI_PORT, SWSPI_MOSI);
			}
			_delay_us(50);
			 sbi(SWSPI_PORT, SWSPI_CLK);
			_delay_us(50);
			 cbi(SWSPI_PORT, SWSPI_CLK);
		}	
		cbi(SWSPI_PORT, SWSPI_MOSI);
		sbi(SWSPI_PORT, SWSPI_CS);
	}
	_delay_ms(10);
}

ISR(SPI_STC_vect)
{
  	in_data = SPDR;
	
	spi_dbg_buf[spi_dbg_buf_ptr] = in_data;
	spi_dbg_buf_ptr++;

  	SPDR = 0x0;
	sbi(PORTC, PINC0);
	switch(fsm_spi_slave_state){
		case FSM_IDLE:
			if (in_data == SERVICE_ADDRES) {
				fsm_spi_slave_state = FSM_SET_ADDRESS_BYTE; 
				//timer_enable();
			} else if (in_data == reg_address) {
				fsm_spi_slave_state = FSM_SIZE_AND_NUMBER_BYTE; 
				//timer_enable();
			}
			break;
		case FSM_SET_ADDRESS_BYTE:

			reg_address = in_data;
			fsm_spi_slave_state = FSM_IDLE;
			//timer_disable();
			break;
		case FSM_SIZE_AND_NUMBER_BYTE:

			fsm_spi_slave_state = FSM_ADDR_PAYLOAD_BYTE;
			preset_width = (in_data & 0x0f);
			preset_data_pointer = (in_data >> 4);
			break;
		case FSM_ADDR_PAYLOAD_BYTE:

			led_data[preset_data_pointer] = in_data;
			line_width = preset_width;
			fsm_spi_slave_state = FSM_IDLE;
			show_flag = 1;
			//timer_disable();
			break;
		default:
			break;


	}
}
ISR(TIMER0_OVF_vect)
{
	/*if (fsm_spi_slave_state != FSM_IDLE){
		fsm_spi_slave_state = FSM_IDLE;
		preset_width = 0;
		preset_data_pointer = 0;
	}*/
	timer_disable();
}

int main(void)
{

	sbi(DDRC, PINC0);
	cbi(PORTC, PINC0);
	usart_init();
	usart_send_byte('A');
	spi_init_slave();
	spi_init_sw_master();
	sei();
	while(1){
		if (show_flag){
			show_flag = 0;
			for(uint8_t z = 0; z < spi_dbg_buf_ptr; z++){
				usart_send_byte(spi_dbg_buf[z]);
			}
			spi_dbg_buf_ptr = 0;
			
			usart_send_byte(0x11);

			usart_send_byte(preset_data_pointer);
			usart_send_byte(preset_width);
			for(uint8_t z = 0; z < 8; z++){
				usart_send_byte(led_data[z]);
			}
			

		}
		send_led_data();
	};
}
