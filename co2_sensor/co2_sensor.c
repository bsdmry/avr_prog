#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "co2_sensor.h"
#include "cbuf.h"

#define UART_BAUD 9600L
#define prescaler ((F_CPU/(16*UART_BAUD))-1)
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)

#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))

#define SWSPI_PORT PORTC
#define SWSPI_REG DDRC
#define SWSPI_PIN PINC
#define SWSPI_CLK 4
#define SWSPI_CS 5
//#define SWSPI_MOSI 3
#define PORT_MOSI1 PORTD 
#define PIN_MOSI1 5 

#define LED_CHARS_NUM 4

static uint8_t reply[24] = {0x00};
rb_u8 buffer  = {
	.data = reply,
	.size = sizeof(reply),
	.head = 0,
	.tail = 0,
	.length = 0
};

//uint8_t read_co2_request[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
uint8_t read_co2_request[9] = {0x42, 0x4d, 0xe3, 0x00, 0x00, 0x01, 0x72};
uint8_t self_cal_off_request[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
uint8_t self_cal_on_request[9] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xe6};
uint8_t calibration_start_request[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
uint16_t co2_ppm = 0;
uint16_t co2_ppm_comp = 0;
volatile uint8_t read_flag = 0;
static const uint8_t nums[10] = {
	0b00111111,
	0b00000110,
	0b01011011,
	0b01001111,
	0b01100110,
	0b01101101,
	0b01111101,
	0b00000111,
	0b01111111,
	0b01101111
};
uint8_t led_data[LED_CHARS_NUM] = {0x00, 0xFF, 0x00, 0xFF};

void send_led_data(void){
	for(uint8_t i = 0; i<LED_CHARS_NUM; i++){
		cbi(SWSPI_PORT, SWSPI_CS);
		uint8_t pos = (0xFF & ~(1 << (i)));
		for (uint8_t p = 0; p<8; p++){
			if ((pos<<p)&0x80){ //Byte order from left to right
			//if ((pos>>p)&0x01){ Byte order from right to left
				sbi(PORT_MOSI1, PIN_MOSI1);
			}else {	
				cbi(PORT_MOSI1, PIN_MOSI1);
			}
			_delay_us(100);
			 sbi(SWSPI_PORT, SWSPI_CLK);
			_delay_us(100);
			 cbi(SWSPI_PORT, SWSPI_CLK);
		}	

		for (uint8_t k = 0; k<8; k++){
			if ((led_data[i]<<k)&0x80){ //Bit0 is A
			//if ((led_data[i]>>k)&0x01){ //Bit0 is DP
				sbi(PORT_MOSI1, PIN_MOSI1);
			}else {	
				cbi(PORT_MOSI1, PIN_MOSI1);
			}
			_delay_us(100);
			 sbi(SWSPI_PORT, SWSPI_CLK);
			_delay_us(100);
			 cbi(SWSPI_PORT, SWSPI_CLK);
		}	
		cbi(PORT_MOSI1, PIN_MOSI1);
		sbi(SWSPI_PORT, SWSPI_CS);
		_delay_ms(1);
	}
}

void spi_init_sw_master(void){
	sbi(SWSPI_REG, SWSPI_CLK);
	sbi(DDRD, PIN_MOSI1);
	sbi(SWSPI_REG, SWSPI_CS);

	cbi(SWSPI_PORT, SWSPI_CLK);
	cbi(PORT_MOSI1, PIN_MOSI1);
	sbi(SWSPI_PORT, SWSPI_CS);
}

void usart_init(void){
	UBRRL = LO(prescaler);
	UBRRH = HI(prescaler);
	UCSRA = 0;
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
}

void timer_init(void){
        TCCR1B|=(1<<CS12) | (1<<CS10);//Предделитель = 64
        TIMSK|=(1<<TOIE1);//Разрешить прерывание по переполнению таймера 1
        TCNT1=3035;//Начальное значение таймера
}

void usart_send_byte(uint8_t b){
	while( !(UCSRA & (1<<UDRE)) );
	UDR = b;
}
void usart_send_data(uint8_t *data, uint8_t size){
	for (uint8_t i = 0; i<size; i++){
		usart_send_byte(data[i]);
	}
}

ISR(USART_RXC_vect){
	rb_u8_push(&buffer, UDR);
	if (buffer.length >= 16){
		read_flag = 1;
	}
}

ISR(TIMER1_OVF_vect)
{
        TCNT1=3035;//Начальное значение таймера
	//read_flag = 1;
}

void read_co2_values(void){

	//usart_send_data(read_co2_request, 7);
	//_delay_ms(1000);
	co2_ppm = reply[6];
	co2_ppm = (co2_ppm << 8) + reply[7];
	co2_ppm_comp = co2_ppm;

	uint8_t dig;
	for (uint8_t p = 0; p < 4; p++) {
                dig = co2_ppm % 10;
                led_data[3-p] = nums[dig];
                co2_ppm /= 10;
        }

	for (uint8_t i = 0; i < 24; i++){
		reply[i] = 0x00;
	}
	buffer.head = 0;
	buffer.tail = 0;
	buffer.length = 0;
}

int main(void)
{

	usart_init();
	//timer_init();
	sbi(DDRC, PC0);
	sbi(DDRC, PC1);
	sbi(DDRC, PC2);
	sbi(PORTC, PC0);
	sbi(PORTC, PC1);
	sbi(PORTC, PC2);
	spi_init_sw_master();
	_delay_ms(500);

	send_led_data();
	sei();

	while(1){
		if(read_flag){
			read_flag = 0;
			read_co2_values();
		}
		if (co2_ppm_comp <= 1100){
			sbi(PORTC, PC0);
			cbi(PORTC, PC1);
			cbi(PORTC, PC2);
		} else if ((co2_ppm_comp > 1100) && (co2_ppm_comp < 2100)){
			cbi(PORTC, PC0);
			sbi(PORTC, PC1);
			cbi(PORTC, PC2);
		
		} else if (co2_ppm_comp >= 2100){
			cbi(PORTC, PC0);
			cbi(PORTC, PC1);
			sbi(PORTC, PC2);
		} else {
			sbi(PORTC, PC0);
			cbi(PORTC, PC1);
			sbi(PORTC, PC2);
		}
		send_led_data();
	};
}
