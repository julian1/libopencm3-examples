/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void gpio_setup(void)
{
	/* Enable GPIOD clock. */
	/* Manually: */
	// RCC_AHB1ENR |= RCC_AHB1ENR_IOPDEN;
	/* Using API functions: */
	// rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE); // JA

	rcc_periph_clock_enable(RCC_GPIOD); // JA

	/* Set GPIO12 (in GPIO port D) to 'output push-pull'. */
	/* Manually: */
	// GPIOD_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((8 - 8) * 4) + 2));
	// GPIOD_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((8 - 8) * 4));
	/* Using API functions: */
	// gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);


   // gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_OTYPE_OD, GPIO0); // JA
   gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA


   gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 ); // JA

}

// think coil has 4 power positions.
// +5,0, +5,+5, 0,+5, 0,0,
// but we won't use them all.
// and we have to get the polarity right...
/*
static int ia[][4] = {
  { 1,0,   0, 1 },
  { 1,1,   0, 0 },
  { 0,1,   1, 0 },
  { 0,0,   1, 1 },
};
*/
// just use modulus?


// wiring, actually doesn't seem to matter, will just spin forward/reverse
// 42byg https://www.youtube.com/watch?v=pID6TTgg1HU
// wave-form
// http://rfanat.ru/stanki_chpu/BIPOLAR_Stepper_Motor_Driver_Circuits_74194.html

/*
  3.75V
    turning quite fast 1 rps. uses 0.16A
    1 revolution every 10 sec uses 1.4 amps.
    runnning a click every second uses 1.69 amps. which is a lot.

    eg. when slow - its almost an open short.
*/

int main(void)
{
	uint32_t i = 0;
	int j = 0;


	gpio_setup();

  // this needs to be set/high - to get the voltage switch
  // alternatively we pulse this ... to reduce current...
  gpio_set(GPIOD, GPIO0);   // JA enable A
  gpio_set(GPIOD, GPIO5);   // enable B


  // full wave drive
  // seems to be working
	while (1) {

    ++i;
    // switch(i % 4) {
    switch(3 - (i % 4)) {
        // reverse would just be 3 - the value

      // NO... the pins are around the wrong way.
      // the wave form pins - are different. for particular pair should never be both set or both clear.

      case 0:
        // gpio_set(  GPIOD, GPIO1);  gpio_clear(GPIOD, GPIO2);  gpio_set(GPIOD, GPIO3);  gpio_clear(GPIOD, GPIO4);
        gpio_set(GPIOD, GPIO1 | GPIO3); gpio_clear(GPIOD, GPIO2 | GPIO4);
        break;
      case 1:
        // gpio_set(  GPIOD, GPIO1);  gpio_clear(GPIOD, GPIO2);  gpio_clear(GPIOD, GPIO3);  gpio_set(GPIOD, GPIO4);
        gpio_set(GPIOD, GPIO1 | GPIO4); gpio_clear(GPIOD, GPIO2 | GPIO3);
        break;
      case 2:
        // gpio_clear(  GPIOD, GPIO1);  gpio_set(GPIOD, GPIO2);  gpio_clear(GPIOD, GPIO3);  gpio_set(GPIOD, GPIO4);
        gpio_set(GPIOD, GPIO2 | GPIO4); gpio_clear(GPIOD, GPIO1 | GPIO3);
        break;
      case 3:
        // gpio_clear(  GPIOD, GPIO1);  gpio_set(GPIOD, GPIO2);  gpio_set(GPIOD, GPIO3);  gpio_clear(GPIOD, GPIO4);
        gpio_set(GPIOD, GPIO2 | GPIO3); gpio_clear(GPIOD, GPIO1 | GPIO4);
        break;
    }



    switch((i / 200) % 2 ) {

      case 0:  {
        gpio_clear(GPIOE, GPIO0);

        for (j = 0; j < 10000; j++)
          __asm__("nop");

        break;
      }

      case 1: {
        gpio_set(GPIOE, GPIO0);

        for (j = 0; j < 30000; j++)
          __asm__("nop");
        break;
      }

    }

    // for (j = 0; j < 3000; j++) { // about the fastest it will go. 10rpm? need to increase the voltage past 10V.
    // for (j = 0; j < 5000; j++) { //
    // for (j = 0; j < 10000; j++) { /* Wait a bit. */
    // for (j = 0; j < 30000; j++) { /* Wait a bit. */
    // for (j = 0; j < 100000; j++) { /* Wait a bit. */
    /*
    for (j = 0; j < 2000000; j++) {
			__asm__("nop");
		}
    */
	}

	return 0;
}


