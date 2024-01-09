void send_led_data(void);
void spi_init_sw_master(void);
void usart_init(void);
void timer_init(void);
void usart_send_byte(uint8_t b);
void usart_send_data(uint8_t *data, uint8_t size);
//uint8_t get_crc(uint8_t *data);
void read_co2_values(void);
