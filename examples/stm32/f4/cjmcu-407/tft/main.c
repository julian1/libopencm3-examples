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



inline void startWrite( void)
 {
#if 0
  SPI.beginTransaction(_tftSpiSettingsWrite);
  digitalWrite(LCD_CS, LOW);
#endif
	gpio_clear(GPIOB, LCD_NSS);	/* Select the LCD */  // pull low
 }

void endWrite()
 {
#if 0
  digitalWrite(LCD_CS, HIGH);
  SPI.endTransaction();
#endif

	gpio_set(GPIOB, LCD_NSS);	// deslect, pull high
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
  msleep(5);  // milli. should be micro
  gpio_clear(GPIOE, LCD_RST); // low
  msleep(5);  // milli, should be mico
  gpio_set(GPIOE, LCD_RST);   // high
  msleep(65);  // milli


  startWrite();

{
   //  Power control settings
   lcdWriteCommand2(0xC0, 0x19, 0x1a);
   lcdWriteCommand2(0xC1, 0x45, 0x00);
   lcdWriteCommand(0xC2, 0x33);        //  Power/Reset on default

   lcdWriteCommand2(0xC5, 0x00, 0x28);  //  VCOM control

   lcdWriteCommand2(0xB1, 0xA0, 0x11);  //  Frame rate control

   lcdWriteCommand(0xB4, 0x02);        //  Display Z Inversion

   lcdWriteReg(0xB6);                  //  Display Control Function      
   lcdWriteData(0x00);
   lcdWriteDataContinue(0x42);
   lcdWriteDataContinue(0x3B);

   lcdWriteReg(0xE0);                  //  Positive Gamma control
   lcdWriteData(0x1F);
   lcdWriteDataContinue(0x25);
   lcdWriteDataContinue(0x22);
   lcdWriteDataContinue(0x0B);
   lcdWriteDataContinue(0x06);
   lcdWriteDataContinue(0x0A);
   lcdWriteDataContinue(0x4E);
   lcdWriteDataContinue(0xC6);
   lcdWriteDataContinue(0x39);
   lcdWriteDataContinue(0x00);
   lcdWriteDataContinue(0x00);
   lcdWriteDataContinue(0x00);
   lcdWriteDataContinue(0x00);
   lcdWriteDataContinue(0x00);
   lcdWriteDataContinue(0x00);

   lcdWriteReg(0XE1);                  //  Negative Gamma control
   lcdWriteData(0x1F);
   lcdWriteDataContinue(0x3F);
   lcdWriteDataContinue(0x3F);
   lcdWriteDataContinue(0x0F);
   lcdWriteDataContinue(0x1F);
   lcdWriteDataContinue(0x0F);
   lcdWriteDataContinue(0x46);
   lcdWriteDataContinue(0x49);
   lcdWriteDataContinue(0x31);
   lcdWriteDataContinue(0x05);
   lcdWriteDataContinue(0x09);
   lcdWriteDataContinue(0x03);
   lcdWriteDataContinue(0x1C);
   lcdWriteDataContinue(0x1A);
   lcdWriteDataContinue(0x00);

   //  From original driver, but register numbers don't make any sense.
   if (0)
   {
    lcdWriteReg(0XF1);
    lcdWriteData(0x36);
    lcdWriteDataContinue(0x04);
    lcdWriteDataContinue(0x00);
    lcdWriteDataContinue(0x3C);
    lcdWriteDataContinue(0x0F);
    lcdWriteDataContinue(0x0F);
    lcdWriteDataContinue(0xA4);
    lcdWriteDataContinue(0x02);

    lcdWriteReg(0XF2);
    lcdWriteData(0x18);
    lcdWriteDataContinue(0xA3);
    lcdWriteDataContinue(0x12);
    lcdWriteDataContinue(0x02);
    lcdWriteDataContinue(0x32);
    lcdWriteDataContinue(0x12);
    lcdWriteDataContinue(0xFF);
    lcdWriteDataContinue(0x32);
    lcdWriteDataContinue(0x00);

    lcdWriteReg(0XF4);
    lcdWriteData(0x40);
    lcdWriteDataContinue(0x00);
    lcdWriteDataContinue(0x08);
    lcdWriteDataContinue(0x91);
    lcdWriteDataContinue(0x04);

    lcdWriteReg(0XF8);
    lcdWriteData(0x21);
    lcdWriteDataContinue(0x04);
   }

   lcdWriteCommand(0x3A, 0x55);

   //  Set initial rotation to match AFX defaults - tall / narrow
   lcdWriteCommand2(0xB6, 0x00, 0x22);
   lcdWriteCommand(0x36, 0x08);

   lcdWriteReg(0x11); // Sleep out

/*

    JA
   //  Fill screen to black
   writeFillRect2(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x0000);
*/

   lcdWriteReg(0x29);  // Turn on display
  }
  endWrite();
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


