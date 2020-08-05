/*
  8-bit parallel.

  8230 that thought was ILI9341.

  trying to work it out,
  https://forum.arduino.cc/index.php?topic=438292.0

  https://github.com/prenticedavid/MCUFRIEND_kbv/blob/master/MCUFRIEND_kbv.cpp
    
    if (_lcd_ID == 0x8230) {    // UC8230 has strange BGR and READ_BGR behaviour


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





// https://github.com/MichalKs/STM32F4_ILI9320/blob/master/STM32F4_ILI9320/app/src/ili9320.c

/*
 * ILI9320 driver commands/registers
 */
#define ILI9320_START_OSCILLATION 0x00
#define ILI9320_READ_ID           0x00
#define ILI9320_DRIVER_OUTPUT     0x01
#define ILI9320_DRIVING_WAVE      0x02
#define ILI9320_ENTRY_MODE        0x03
#define ILI9320_RESIZE            0x04
#define ILI9320_DISP1             0x07
#define ILI9320_DISP2             0x08
#define ILI9320_DISP3             0x09
#define ILI9320_DISP4             0x0a
#define ILI9320_RGB_DISP1         0x0c
#define ILI9320_FRAME_MARKER      0x0d
#define ILI9320_RGB_DISP2         0x0f
#define ILI9320_POWER1            0x10
#define ILI9320_POWER2            0x11
#define ILI9320_POWER3            0x12
#define ILI9320_POWER4            0x13
#define ILI9320_HOR_GRAM_ADDR     0x20
#define ILI9320_VER_GRAM_ADDR     0x21
#define ILI9320_WRITE_TO_GRAM     0x22
#define ILI9320_POWER7            0x29
#define ILI9320_FRAME_RATE        0x2b
#define ILI9320_GAMMA1            0x30
#define ILI9320_GAMMA2            0x31
#define ILI9320_GAMMA3            0x32
#define ILI9320_GAMMA4            0x35
#define ILI9320_GAMMA5            0x36
#define ILI9320_GAMMA6            0x37
#define ILI9320_GAMMA7            0x38
#define ILI9320_GAMMA8            0x39
#define ILI9320_GAMMA9            0x3c
#define ILI9320_GAMMA10           0x3d
#define ILI9320_HOR_ADDR_START    0x50
#define ILI9320_HOR_ADDR_END      0x51
#define ILI9320_VER_ADDR_START    0x52
#define ILI9320_VER_ADDR_END      0x53
#define ILI9320_DRIVER_OUTPUT2    0x60
#define ILI9320_BASE_IMAGE        0x61
#define ILI9320_VERTICAL_SCROLL   0x6a
#define ILI9320_PARTIAL1_POS      0x80
#define ILI9320_PARTIAL1_START    0x81
#define ILI9320_PARTIAL1_END      0x82
#define ILI9320_PARTIAL2_POS      0x83
#define ILI9320_PARTIAL2_START    0x84
#define ILI9320_PARTIAL2_END      0x85
#define ILI9320_PANEL_INTERFACE1  0x90
#define ILI9320_PANEL_INTERFACE2  0x92
#define ILI9320_PANEL_INTERFACE3  0x93
#define ILI9320_PANEL_INTERFACE4  0x95
#define ILI9320_PANEL_INTERFACE5  0x97
#define ILI9320_PANEL_INTERFACE6  0x98


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


static void sendCommand16(uint8_t command, uint16_t data)
{
  gpio_clear(LCD_PORT, LCD_RS);   // low - to assert register, D/CX  p24
  send8(command);
  gpio_set(LCD_PORT, LCD_RS);     // high - to assert data
/*
      sendData0( color >> 8 );
      sendData0( color & 0xFF );
*/
  send8(data >> 8);   // Check
  send8(data & 0xFF);
}


#if 0
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
#endif




/*

  fucking manual doesn't even say,
    https://www.jaycar.com.au/medias/sys_master/images/images/9404693577758/XC4630-dataSheetMain.pdf
    
  but other doc says its UC8230
    https://www.jaycar.com.au/medias/sys_master/images/images/9404693544990/XC4630-manualMain.pdf

  https://forum.arduino.cc/index.php?topic=438292.0

    looks exactly like this, 
      https://www.amazon.it/Arduino-Mega2560-320x240-pollici-lettore/dp/B01C3RDFN6/

  -----------------
    
    The UC8230S register set "looks" very similar to an ILI9320.

  very similar to ILI9320 supposedly...
  
  Yes,  the UC8230 is in the same class as ILI9320.   i.e. no Band Scroll.
  
  I have only managed to find a "Register list" for the UC8230.   Not a full datasheet.
  The Registers and bitfields seem to be in the same places as ILI9320 / SPFD5408.

  
  code looks decent, - all reg values are 16 bit though?
  https://github.com/MichalKs/STM32F4_ILI9320/blob/master/STM32F4_ILI9320/app/src/ili9320.c



*/



// 


static void ILI9320_Initializtion(void) {

/*
  ILI9320_HAL_HardInit(); // GPIO and FSMC init

  // Reset the LCD
  ILI9320_HAL_ResetOff();
  TIMER_Delay(50);
  ILI9320_HAL_ResetOn();
  TIMER_Delay(50);
  ILI9320_HAL_ResetOff();
  TIMER_Delay(50);

  SendCommand16(ILI9320_START_OSCILLATION, 0x0001);
  TIMER_Delay(20);

  // Read LCD ID
  unsigned int id;
  id = ILI9320_HAL_ReadReg(ILI9320_READ_ID);

  printf("ID TFT LCD = %x\r\n", id);


  // Add more LCD init codes here
  if (id == 0x9320) {
*/

    sendCommand16(ILI9320_START_OSCILLATION, 0x0001);
    msleep(20);



    sendCommand16(ILI9320_DRIVER_OUTPUT, 0x0100); // SS = 1 - coordinates from left to right
    sendCommand16(ILI9320_DRIVING_WAVE, 0x0700);  // Line inversion
    sendCommand16(ILI9320_ENTRY_MODE, 0x1018);    //
    sendCommand16(ILI9320_RESIZE, 0x0000);
    sendCommand16(ILI9320_DISP1, 0x0000);
    sendCommand16(ILI9320_DISP2, 0x0202); // two lines back porch, two line front porch
    sendCommand16(ILI9320_DISP3, 0x0000);
    sendCommand16(ILI9320_DISP4, 0x0000);
    sendCommand16(ILI9320_RGB_DISP1, 0x0001);
    sendCommand16(ILI9320_FRAME_MARKER, 0x0000); // 0th line for frame marker
    sendCommand16(ILI9320_RGB_DISP2, 0x0000);
    sendCommand16(ILI9320_DISP1, 0x0101);
    sendCommand16(ILI9320_POWER1, 0x10c0);
    sendCommand16(ILI9320_POWER2, 0x0007);
    sendCommand16(ILI9320_POWER3, 0x0110);
    sendCommand16(ILI9320_POWER4, 0x0b00);
    sendCommand16(ILI9320_POWER7, 0x0000);
    sendCommand16(ILI9320_FRAME_RATE, 0x4010);

    // Set window
    sendCommand16(ILI9320_HOR_ADDR_START, 0);
    sendCommand16(ILI9320_HOR_ADDR_END, 239);
    sendCommand16(ILI9320_VER_ADDR_START, 0);
    sendCommand16(ILI9320_VER_ADDR_END, 319);

    sendCommand16(ILI9320_DRIVER_OUTPUT2, 0x2700);
    sendCommand16(ILI9320_BASE_IMAGE, 0x0001);
    sendCommand16(ILI9320_VERTICAL_SCROLL, 0x0000);
    sendCommand16(ILI9320_PARTIAL1_POS, 0x0000);
    sendCommand16(ILI9320_PARTIAL1_START, 0x0000);
    sendCommand16(ILI9320_PARTIAL1_END, 0x0000);
    sendCommand16(ILI9320_PARTIAL2_POS, 0x0000);
    sendCommand16(ILI9320_PARTIAL2_START, 0x0000);
    sendCommand16(ILI9320_PARTIAL2_END, 0x0000);
    sendCommand16(ILI9320_PANEL_INTERFACE1, 0x0010);
    sendCommand16(ILI9320_PANEL_INTERFACE2, 0x0000);
    sendCommand16(ILI9320_PANEL_INTERFACE3, 0x0001);
    sendCommand16(ILI9320_PANEL_INTERFACE4, 0x0110);
    sendCommand16(ILI9320_PANEL_INTERFACE5, 0x0000);
    sendCommand16(ILI9320_PANEL_INTERFACE6, 0x0000);
    sendCommand16(ILI9320_DISP1, 0x0173);

  // }

  // TIMER_Delay(100);

  msleep(100);
}


static void ILI9320_SetCursor(uint16_t x, uint16_t y) {

 sendCommand16(ILI9320_HOR_GRAM_ADDR, y);
 sendCommand16(ILI9320_VER_GRAM_ADDR, x);

}
/**
 * @brief Draws a pixel on the LCD.
 * @param x X coordinate of pixel.
 * @param y Y coordinate of pixel.
 * @param r Red color value.
 * @param g Green color value.
 * @param b Blue color value.
 */
// static void ILI9320_DrawPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
static void ILI9320_DrawPixel(uint16_t x, uint16_t y) {

  // calling this makes it flicker terribly - so there's someting not right.

  ILI9320_SetCursor(x, y);
  // sendCommand16(ILI9320_WRITE_TO_GRAM, ILI9320_RGBDecode(r, g, b));
  // sendCommand16(ILI9320_WRITE_TO_GRAM, 0xf0f0 );
  sendCommand16(ILI9320_WRITE_TO_GRAM, 0xff00 );
}

// color is grey...
// like it's only painting a single byte?

static void GRAPH_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {

  int i, j;
  // Fill rectangle with color
  for (i = x; i < x + w; i++) {
    for (j = y; j < y + h; j++) {
      ILI9320_DrawPixel(i, j );//, currentColor.r, currentColor.g, currentColor.b);
    }
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

  ILI9320_Initializtion();

  // Does the same weird crap - with or without the Initialisation...
  // so initialialisation is no good.

  ILI9320_DrawPixel(50, 50); 
  GRAPH_DrawRectangle(50, 50, 5, 5); 


  // not sure that the correct commands and data are being sent...
  // display off, or changing the brightness should have done something.

  // OK. nothing looks like its working...


/*
    // OK this actually does something.
    sendCommand0(0x68  );
    sendData0(0x9 );
*/




  ///////////

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


