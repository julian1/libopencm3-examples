/*
  code be run on linux, not mcu .
*/

#include <stdint.h>
#include <stdio.h>

static void sendCommand16(uint8_t command, uint16_t data)
{
/*
  send8(data >> 8);   // Check
  send8(data & 0xFF);
*/
}

 

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
      printf("delay %d\n", data );
      // msleep(data);
    } 
    else {
      printf("cmd %x   data %x\n", cmd, data);
      sendCommand16(cmd, data);
    }

  }
 
}



int main(void)
{
  init();

  return 0;
}


