/*

spi2
  pb12- pb15  pb9,pb10,   pe14,pd14,pi2,pi3

  pb12 - nss  - cs   blue.  slave select.
  pb13 - sck.  white
  pb14 - miso - grey
  pb15 - mosi - purple

  pe11 - wr - orange 
  pe12 - rs - yellow
  pe13 - rst - green

  we need to map these by name. #define WR PE
  actually not sure. it's messy given the 
 */


#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "clock.h"

// GPIOB
#define LCD_NSS   GPIO12 // not AF
#define LCD_SCK   GPIO13
#define LCD_MISO  GPIO14 // not used
#define LCD_MOSI  GPIO15

// GPIOE
#define LCD_WR    GPIO11
#define LCD_RS    GPIO12
#define LCD_RST   GPIO13

#define LCD_SPI   SPI2

static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}





// https://github.com/ImpulseAdventure/Waveshare_ILI9486/blob/master/src/Waveshare_ILI9486.cpp

void initializeLcd()
 {
#if 0
  //  Trigger hardware reset.
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delayMicroseconds(20);
  digitalWrite(LCD_RST, HIGH);

  //  TO-DO - how long after a reset until the screen can be used?  Doesn't seem to be
  //  specified in the datasheet.
  //  Experimentally, any less than this and the initial screen clear is incomplete.
  delay(65);
#endif
  
  // appears to do the right thing on the pins. but lcd doesn't change.

  gpio_set(GPIOE, LCD_RST);   // high
  msleep(5);  // milli
  gpio_clear(GPIOE, LCD_RST); // low
  msleep(5);  // milli
  gpio_set(GPIOE, LCD_RST);   // high
  msleep(65);  // milli
}


void startWrite()
 {
#if 0
  SPI.beginTransaction(_tftSpiSettingsWrite);
  digitalWrite(LCD_CS, LOW);
#endif

	gpio_clear(GPIOB, LCD_NSS);	/* Select the LCD */  // pull low

 }



 inline void lcdWriteReg(uint8_t reg)
 {
#if 0
  digitalWrite(LCD_DC, LOW);
  SPI.transfer(0);
  SPI.transfer(reg);
#endif

	gpio_clear(GPIOE, LCD_RS);	// make sure in command mode 
	(void) spi_xfer(LCD_SPI, 0 );
	(void) spi_xfer(LCD_SPI, reg );
 }

 inline void lcdWriteData(uint8_t data)
 {
#if 0
  digitalWrite(LCD_DC, HIGH);
  SPI.transfer(0);
  SPI.transfer(data);
#endif
  gpio_set(GPIOE, LCD_RS);	/* Set the D/CX pin */
	(void) spi_xfer(LCD_SPI, 0 );
	(void) spi_xfer(LCD_SPI, data );
 }

inline void lcdWriteDataContinue(uint8_t data)
 {
  // eg. no change of RS
	(void) spi_xfer(LCD_SPI, 0 );
	(void) spi_xfer(LCD_SPI, data );
 }

 inline void lcdWriteCommand(uint8_t reg, uint8_t data)
 {
  lcdWriteReg(reg);
  lcdWriteData(data);
 }

 inline void lcdWriteCommand2(uint8_t reg, uint8_t data, uint8_t data2)
 {
  lcdWriteReg(reg);
  lcdWriteData(data);
  lcdWriteDataContinue(data2);
 }



int main(void)
{

  clock_setup();
  led_setup();

  rcc_periph_clock_enable( RCC_GPIOE );
  rcc_periph_clock_enable( RCC_GPIOB );


  // general registers
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_WR | LCD_RS | LCD_RST ); 

  // 
  // OK. in the example lcd-spi.c it looks like only mosi and clk are set AF. not cs. or miso.
  // spi2 is AF5
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, LCD_SCK | LCD_MOSI );
  gpio_set_af(GPIOB, GPIO_AF5, LCD_SCK | LCD_MOSI );

  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_NSS); 


  rcc_periph_clock_enable(RCC_SPI2);
  spi_init_master(LCD_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_4,
    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
    SPI_CR1_CPHA_CLK_TRANSITION_1,
    SPI_CR1_DFF_8BIT,
    SPI_CR1_MSBFIRST);
  spi_enable_ss_output(LCD_SPI);
  spi_enable(LCD_SPI);


  gpio_clear(GPIOE, LCD_WR); // pull low, select WR for board transceivers


  initializeLcd();

	while (1) {
    gpio_toggle(GPIOE, GPIO0);  // toggle led
    msleep(300);
	}

  return 0;
}


