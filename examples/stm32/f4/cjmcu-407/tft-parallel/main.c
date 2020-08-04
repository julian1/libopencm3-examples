/*
  8-bit parallel.

  in 8 bit parallel.
    is the command still 16 bit.
    which byte is loaded first.

  - general
    https://controllerstech.com/interface-tft-display-with-stm32/

  https://www.displayfuture.com/Display/datasheet/controller/ILI9341.pdf

  simple,
  https://github.com/adafruit/Adafruit_ILI9341

  primitives
    https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_SPITFT.cpp

  optimised. 70 forks. but uses spi.
    https://github.com/PaulStoffregen/ILI9341_t3

  So we would have to bit-bash.
    we can use the millisec msleep to begin with.
    a nop loop for micro-second. just write the inner loop - execute it a thousand times and blink led -

  Note - rather than setting gpio individually - would have to left/shift values into place for the bus.
    but would likely be pretty damn fast.
  --
 */


#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "clock.h"


// GPIOE
#define LCD_PORT  GPIOE

#define LCD_RST   GPIO2
#define LCD_CS    GPIO3
#define LCD_RS    GPIO4
#define LCD_WR    GPIO5
#define LCD_RD    GPIO6


#define LCD_DATA_PORT  GPIOD

// LCD

static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}


// should put control registers on different port - this would allow 16 bit parallel bit bashing
// without hardly any code changes.

#if 0

void Adafruit_SPITFT::sendCommand16(uint16_t commandWord,
                                    const uint8_t *dataBytes,
                                    uint8_t numDataBytes) {
  SPI_BEGIN_TRANSACTION();
  if (_cs >= 0)
    SPI_CS_LOW();

  if (numDataBytes == 0) {
    SPI_DC_LOW();             // Command mode
    SPI_WRITE16(commandWord); // Send the command word
    SPI_DC_HIGH();            // Data mode
  }
  for (int i = 0; i < numDataBytes; i++) {
    SPI_DC_LOW();             // Command mode
    SPI_WRITE16(commandWord); // Send the command word
    SPI_DC_HIGH();            // Data mode
    commandWord++;
    SPI_WRITE16((uint16_t)pgm_read_byte(dataBytes++));
  }

  if (_cs >= 0)
    SPI_CS_HIGH();
  SPI_END_TRANSACTION();
}

#endif




static inline void delay( uint16_t x )
{
  msleep(x);
}

static void send16( uint16_t x )
{
    gpio_clear(LCD_PORT, LCD_WR);   // clear write strobe
    delay(1);
    gpio_port_write(LCD_DATA_PORT, x);    // low bytes first
    gpio_set(LCD_PORT, LCD_WR);           // write on rising edge
    delay(1);

    gpio_clear(LCD_PORT, LCD_WR);         // clear write strobe
    delay(1);
    gpio_port_write(LCD_DATA_PORT, x >> 8);   // high bytes
    gpio_set(LCD_PORT, LCD_WR);       // write on rising edge
    delay(1);
}


static void sendCommand16(uint16_t commandWord, const uint8_t *dataBytes, uint8_t numDataBytes)
{

    gpio_clear(LCD_PORT, LCD_CS); // assert chip select

    gpio_clear(LCD_PORT, LCD_RS);   // assert command
    send16(commandWord);

    gpio_set(LCD_PORT, LCD_RS);     // assert data

    // OKK. the data is 8 bytes... that's not very clear 
    send16(commandWord);

}



int main(void)
{
  clock_setup();
  led_setup();

  rcc_periph_clock_enable( RCC_GPIOE );
  rcc_periph_clock_enable( RCC_GPIOD );

  // #define GPIO12                              (1 << 12)

  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 0xff ); // JA first 8 bits.

  // gpio_set does not clear.... it does oring operations...
  // set everything to 0
  gpio_clear(GPIOD, 0xff);  // next 8 bits.

  // so to write the register needs a clear first. then set the positive values.
  // gpio_set is not | in the register it is doing something more....
  // gpio_set(GPIOD, GPIO5 );  // turn on, referenced to gnd
  // gpio_set(GPIOD, 0b0000000000000000);  // turn on ????
  // gpio_set(GPIOD, 1 << 12 );  // turn on ????


  // gpio_port_write(GPIOD, 1 << 5);   // not sure that atomic, but more useful...

 	while (1) {

    // gpio_toggle(GPIOD, 1 << 5 );  // blink


    gpio_toggle(GPIOE, GPIO0);  // toggle led
    msleep(300);
	}

  return 0;
}


