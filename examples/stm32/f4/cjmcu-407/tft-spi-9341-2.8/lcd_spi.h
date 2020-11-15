

// spi primitives, that hide underlying stm32 spi details.

void lcd_spi_setup( void );

void assert_cs(void);

void deassert_rst(void);
void assert_rst(void);

void turn_on_backlight( void );


// only  the following should really be used by gfx

void set_command(void );
void set_data(void );

void delay( uint16_t x );

void send8( uint8_t x );

void sendCommand(uint8_t command, const uint8_t *dataBytes, uint8_t numDataBytes);
void sendCommand0(uint8_t command);




