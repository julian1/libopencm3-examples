

// prefix all these with lcd_spi_
// spi primitives, that hide underlying stm32 spi details.

void lcd_spi_setup( void );



// change name - use assert for cs/nss. not rst.
void lcd_spi_assert_rst(void);
void lcd_spi_deassert_rst(void);

void lcd_spi_turn_on_backlight( void );


// only  the following should really be used by gfx



void lcd_spi_enable(void);

void lcd_spi_assert_command(void );
void lcd_spi_assert_data(void );


void lcd_spi_send8( uint8_t x );

void lcd_send_command0(uint8_t command);
void lcd_send_command1(uint8_t command, uint8_t data);
void lcd_send_command(uint8_t command, const uint8_t *dataBytes, uint8_t numDataBytes);




