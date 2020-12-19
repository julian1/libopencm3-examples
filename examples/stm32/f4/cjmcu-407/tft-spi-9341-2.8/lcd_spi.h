

// low-level spi primitives, that hide underlying stm32 spi details.

void lcd_spi_setup( void );



void lcd_spi_assert_rst(void);
void lcd_spi_deassert_rst(void);

void lcd_spi_turn_on_backlight( void );



void lcd_spi_enable(void);

// void lcd_spi_assert_command(void );
// void lcd_spi_assert_data(void );
// void lcd_spi_send8( uint8_t x );

// void lcd_send_command0(uint8_t command);
// void lcd_send_command1(uint8_t command, uint8_t data);


void lcd_send_command(uint8_t command, const uint8_t *dataBytes, uint32_t numDataBytes);


void lcd_send_command_repeat(uint8_t command, uint16_t x, uint32_t n );




