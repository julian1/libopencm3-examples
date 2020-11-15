

// prefix all these with lcd_spi_
// spi primitives, that hide underlying stm32 spi details.

void lcd_spi_setup( void );

void lcd_spi_assert_cs(void);

void lcd_spi_assert_rst(void);
void lcd_spi_deassert_rst(void);

void lcd_spi_turn_on_backlight( void );


// only  the following should really be used by gfx

void lcd_spi_set_command(void );
void lcd_spi_set_data(void );


void send8( uint8_t x );

void sendCommand0(uint8_t command);
void sendCommand1(uint8_t command, uint8_t data);
void sendCommand(uint8_t command, const uint8_t *dataBytes, uint8_t numDataBytes);




