/*
  8-bit parallel.

  8230 that thought was ILI9341.

  trying to work it out,
  https://forum.arduino.cc/index.php?topic=438292.0


 */


#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "clock.h"

// GPIOE
#define LCD_PORT  GPIOE

#define LCD_RST   GPIO2   // reset
#define LCD_CS    GPIO3   // chip select
#define LCD_RS    GPIO4   // register select
#define LCD_WR    GPIO5   // write strobe
#define LCD_RD    GPIO6   // read


#define LCD_DATA_PORT  GPIOD

// LCD

static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}


// should put control registers on different port - this would allow 16 bit parallel bit bashing
// without hardly any code changes.


// OK. really not sure if we have to pad everything to 16 bit.
// sendCommand16 is a specialization for 16 bit parallel path.
// eg. should only require 16 bits, if its unavoidable because the bus is genuinely 16 bit.

static inline void delay( uint16_t x )
{
  msleep(x);
}



static void send8( uint8_t x )
{
  gpio_port_write(LCD_DATA_PORT, x);    // setup port
  gpio_clear(LCD_PORT, LCD_WR);         // clear write strobe. falling edge. host asserts.
  delay(1);
  gpio_set(LCD_PORT, LCD_WR);           // write on rising edge. tft reads.
  delay(1);

  // gpio_clear(LCD_PORT, LCD_WR);         // clear write strobe
}

// OK. screen does same thing - whether we asset chip select or not.
// So, think we want to check...

// is the screen flickering a power supply issue?
/*
  p11.

  DCX
  This pin is used to select “Data or Command” in the parallel interface or
  4-wire 8-bit serial data interface. When DCX = ’1’, data is selected. When DCX
  = ’0’, command is selected. This pin is used serial interface clock in 3-wire
  9-bit / 4-wire 8-bit serial data interface

  RDX I MCU (VDDI/VSS)   8080-    /8080I-II system (RDX): Serves as a read
    signal and MCU read data at the rising edge. Fix to VDDI level when not in use.

  WRX (D/CX) I MCU (VDDI/VSS) - 8080-    /8080I-II system (WRX): Serves as a
  write signal and writes data at the rising edge. - 4-line system (D/CX): Serves
  as command or parameter selec

  RESX I MCU (VDDI/VSS) This signal will reset the device and must be applied
  to properly initialize the chip. Signal is active low.

  CSX I MCU (VDDI/VSS) Chip select input pin (“Low” enable). This pin can be
  permanently fixed “Low” in MPU interface mode only.* note1,2
  -------


  // p28 is the best. at showing the levels of everything
  // note also that reading, involves a write from host first to select what to read.
*/

static void sendCommand(uint8_t command, const uint8_t *dataBytes, uint8_t numDataBytes)
{

  gpio_clear(LCD_PORT, LCD_RS);   // low - to assert register, D/CX  p24
  send8(command);

  gpio_set(LCD_PORT, LCD_RS);     // high - to assert data
  for(unsigned i = 0; i < numDataBytes; ++i) {
    send8(dataBytes[ i ]);
  }

}


static void sendCommand0(uint8_t command)
{

  gpio_clear(LCD_PORT, LCD_RS);   // low - to assert register, D/CX  p24
  send8(command);
}

static void sendData0(uint8_t data)
{
  // advantage is that it can be done in a loop. without allocating stack for buffer.
  gpio_set(LCD_PORT, LCD_RS);     // high - to assert data
  send8(data);
}





/*

  fucking manual doesn't even say,
    https://www.jaycar.com.au/medias/sys_master/images/images/9404693577758/XC4630-dataSheetMain.pdf
    
  but other doc says its UC8230
    https://www.jaycar.com.au/medias/sys_master/images/images/9404693544990/XC4630-manualMain.pdf

  https://forum.arduino.cc/index.php?topic=438292.0

    looks exactly like this, 
      https://www.amazon.it/Arduino-Mega2560-320x240-pollici-lettore/dp/B01C3RDFN6/

    
    The UC8230S register set "looks" very similar to an ILI9320.

  very similar to ILI9320 supposedly...
  
  Yes,  the UC8230 is in the same class as ILI9320.   i.e. no Band Scroll.
  
  I have only managed to find a "Register list" for the UC8230.   Not a full datasheet.
  The Registers and bitfields seem to be in the same places as ILI9320 / SPFD5408.

*/



// 

int main(void)
{
  clock_setup();
  led_setup();

  rcc_periph_clock_enable( RCC_GPIOE );
  rcc_periph_clock_enable( RCC_GPIOD );


  // gpio_mode_setup(LCD_DATA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 0xff ); // JA first 8 bits.
  gpio_mode_setup(LCD_DATA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7  ); // JA first 8 bits.
  gpio_mode_setup(LCD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_RST | LCD_CS | LCD_RS | LCD_WR | LCD_RD);


  gpio_set(LCD_PORT, LCD_RD);   // turn read off. operates both the transceiver and the lcd which reads on rising edge. 
                                // when set to read - then if gpio is output - it will sink all the output voltage. very bad.
                                // screen flashing resulted from drop in power supply


  // assert chip select, with low
  gpio_clear(LCD_PORT, LCD_CS);


  // hardware reset - review
  gpio_set(  LCD_PORT, LCD_RST);    // high
  msleep(150);
  gpio_clear(LCD_PORT, LCD_RST);   // low
  msleep(150);
  gpio_set(  LCD_PORT, LCD_RST);   // high
  msleep(150);



  // not sure that the correct commands and data are being sent...
  // display off, or changing the brightness should have done something.

  // OK. nothing looks like its working...


   // 0x6809:

    // OK this actually does something.
    sendCommand0(0x68  );
    sendData0(0x9 );


  bool on = 0;
 	while (1) {

    // gpio_toggle(GPIOD, 1 << 5 );  // blink

    // freaking weird - on

    if(on) {
      // led on draws more power... how...
      gpio_set(GPIOE, GPIO0);
      // sendCommand0(ILI9341_INVOFF );
      // ILI9341_DrawPixel(50, 50, 0xf777 );
    }
    else {
      gpio_clear(GPIOE, GPIO0);
      // sendCommand0(ILI9341_INVON);
      // ILI9341_DrawPixel(50, 50, 0x7700 );
    }
    on = ! on;

    msleep(500);
	}

  return 0;
}


