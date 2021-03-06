/*
  8-bit parallel. purchsed from jaycar. think code is uc5408 which is very similar to ili9320

item
  jaycar doc/manual says UC8230
    https://www.jaycar.com.au/medias/sys_master/images/images/9404693544990/XC4630-manualMain.pdf

  https://forum.arduino.cc/index.php?topic=438292.0
    looks exactly like this,
      https://www.amazon.it/Arduino-Mega2560-320x240-pollici-lettore/dp/B01C3RDFN6/

  VERY IMPORTANT

  has specific 8230 init list. (distinct from 9320)
    https://github.com/prenticedavid/MCUFRIEND_kbv/blob/master/MCUFRIEND_kbv.cpp

--------

  many variants like 9320

  forum - gets it working with id = 5408.
  https://forum.arduino.cc/index.php?topic=438292.0

    "The Registers and bitfields seem to be in the same places as ILI9320 / SPFD5408."

    case 0x8230:
        _lcd_capable = 0 | REV_SCREEN | INVERT_GS | INVERT_RGB;
        goto common_9320;

    case 0x5408:
      _lcd_capable = 0 | REV_SCREEN | READ_BGR; //Red 2.4" thanks jorgenv, Ardlab_Gent
      // _lcd_capable = 0 | REV_SCREEN | READ_BGR | INVERT_GS; //Blue 2.8" might be different
      goto common_9320;

  So just treats it as 9320...


  OK. think we really need to read the registers... that will
  tell us if our commands are working, and provide some info.
  ---------

  8 bit mode,

  The i80/8-bit system interface is selected by setting the IM[3:0] as “0011”
  and the DB17~DB10 pins are used to transfer the data. When writing the 16-bit
  register, the data is divided into upper byte (8 bits and LSB is not used)
  lower byte and the upper byte is transferred first. The display data is also
  divided in upper byte (8 bits) and lower byte, and the upper byte is
  transferred first. The written data is expanded into 18 bits internally (see
  the figure below) and then written into GRAM. The unused DB[9:0] pins must be
  tied to AGND.

  https://www.rockbox.org/wiki/pub/Main/GSoCSansaView/ILI9320DS_V0.55.pdf

  register meaning command? it has to be 16 bit.
  so register and data are both 16 bit.
  ---------------

  some example code here,
    https://github.com/grossws/stm32-lcd

  another simple example,
    https://github.com/claiff/ili9320/tree/clear/src

  -------------
  ok everything powered on at 3.3V which is good. don't need the transceivers though


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


#define LCD_WIDTH 30
#define LCD_HEIGHT 30

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


static void send16( uint16_t x )
{
  send8(x >> 8);   // Check
  send8(x & 0xFF);

}


static void sendCommand16(uint16_t cmd, uint16_t data)
{
  gpio_clear(LCD_PORT, LCD_RS);   // low - to assert register, D/CX  p24
  send16(cmd);
  gpio_set(LCD_PORT, LCD_RS);     // high - to assert data
  send16(data);
}




// https://github.com/prenticedavid/MCUFRIEND_kbv/blob/master/MCUFRIEND_kbv.cpp

#define TFTLCD_DELAY 0xFFFF

static const uint16_t ILI9320_regValues[] = {
  0x00e5, 0x8000,
  0x0000, 0x0001,
  0x0001, 0x100,
  0x0002, 0x0700,
  0x0003, 0x1030,
  0x0004, 0x0000,
  0x0008, 0x0202,
  0x0009, 0x0000,
  0x000A, 0x0000,
  0x000C, 0x0000,
  0x000D, 0x0000,
  0x000F, 0x0000,
  //-----Power On sequence-----------------------
  0x0010, 0x0000,
  0x0011, 0x0007,
  0x0012, 0x0000,
  0x0013, 0x0000,
  TFTLCD_DELAY, 50,
  0x0010, 0x17B0,  //SAP=1, BT=7, APE=1, AP=3
  0x0011, 0x0007,  //DC1=0, DC0=0, VC=7
  TFTLCD_DELAY, 10,
  0x0012, 0x013A,  //VCMR=1, PON=3, VRH=10
  TFTLCD_DELAY, 10,
  0x0013, 0x1A00,  //VDV=26
  0x0029, 0x000c,  //VCM=12
  TFTLCD_DELAY, 10,
  //-----Gamma control-----------------------
  0x0030, 0x0000,
  0x0031, 0x0505,
  0x0032, 0x0004,
  0x0035, 0x0006,
  0x0036, 0x0707,
  0x0037, 0x0105,
  0x0038, 0x0002,
  0x0039, 0x0707,
  0x003C, 0x0704,
  0x003D, 0x0807,
  //-----Set RAM area-----------------------
  0x0060, 0xA700,     //GS=1
  0x0061, 0x0001,
  0x006A, 0x0000,
  0x0021, 0x0000,
  0x0020, 0x0000,
  //-----Partial Display Control------------
  0x0080, 0x0000,
  0x0081, 0x0000,
  0x0082, 0x0000,
  0x0083, 0x0000,
  0x0084, 0x0000,
  0x0085, 0x0000,
  //-----Panel Control----------------------
  0x0090, 0x0010,
  0x0092, 0x0000,
  0x0093, 0x0003,
  0x0095, 0x0110,
  0x0097, 0x0000,
  0x0098, 0x0000,
  //-----Display on-----------------------
  0x0007, 0x0173,
  TFTLCD_DELAY, 50,
};




static void init( void ) {

  for( unsigned i = 0; i < sizeof( ILI9320_regValues) / sizeof(uint16_t)  ; i += 2 )  {

    uint16_t cmd = ILI9320_regValues[ i ];
    uint16_t data = ILI9320_regValues[ i + 1];

    if(cmd == TFTLCD_DELAY) {
      // printf("delay %d\n", data );
      msleep(data);
    }
    else {
      // printf("cmd %x   data %x\n", cmd, data);
      sendCommand16(cmd, data);
    }
  }
}






static uint16_t min(uint16_t a, uint16_t b) {
  return (a > b) ? b : a;
}

static uint16_t max(uint16_t a, uint16_t b) {
  return (a > b) ? a : b;
}

static uint16_t sat(uint16_t val, uint16_t vmin, uint16_t vmax) {
  return min(vmax, max(vmin, val));
}

static void lcd_set_cursor(uint16_t x, uint16_t y) {
  sendCommand16(0x20, x);
  sendCommand16(0x21, y);
}


static void lcd_set_window(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom) {
  sendCommand16(0x50, sat(left, 0, LCD_WIDTH-1));
  sendCommand16(0x51, sat(right-1, 0, LCD_WIDTH-1));
  sendCommand16(0x52, sat(top, 0, LCD_HEIGHT-1));
  sendCommand16(0x53, sat(bottom-1, 0, LCD_HEIGHT-1));
}

static void lcd_fill(uint32_t color) {
  // uint16_t data = lcd_pixel_from_rgb32(color);
  // uint16_t data = 0xf7f7;
  uint16_t data = 0x2200;

  lcd_set_window(0, 0, 320 , 200);
  lcd_set_cursor(0, 0);


  gpio_clear(LCD_PORT, LCD_RS);   // low - to assert register, D/CX  p24
  send16(0x22);

  gpio_set(LCD_PORT, LCD_RS);   // high data

  for(uint32_t i = 0; i < LCD_WIDTH*LCD_HEIGHT; i++) {
      // _lcd_tx_data(data);
      send16(i + 999 );
  }
}




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
                                // screen flashing resulted from drop in power supply, not initialisation.


  // assert chip select, with low
  gpio_clear(LCD_PORT, LCD_CS);


  // hardware reset - review
  gpio_set(  LCD_PORT, LCD_RST);    // high
  msleep(150);
  gpio_clear(LCD_PORT, LCD_RST);   // low
  msleep(150);
  gpio_set(  LCD_PORT, LCD_RST);   // high
  msleep(150);


  init();


  lcd_fill(0xf7f7 );

 	while (1) {
    gpio_toggle(GPIOE, GPIO0);
    msleep(500);
	}

  return 0;
}


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



/*

  fucking manual doesn't even say,
    https://www.jaycar.com.au/medias/sys_master/images/images/9404693577758/XC4630-dataSheetMain.pdf

  but other doc says its UC8230
    https://www.jaycar.com.au/medias/sys_master/images/images/9404693544990/XC4630-manualMain.pdf
  -----------------

  The UC8230S register set "looks" very similar to an ILI9320.

  very similar to ILI9320 supposedly...

  Yes,  the UC8230 is in the same class as ILI9320.   i.e. no Band Scroll.

  I have only managed to find a "Register list" for the UC8230.   Not a full datasheet.
  The Registers and bitfields seem to be in the same places as ILI9320 / SPFD5408.

  code looks decent, - all reg values are 16 bit though?
  https://github.com/MichalKs/STM32F4_ILI9320/blob/master/STM32F4_ILI9320/app/src/ili9320.c

*/

