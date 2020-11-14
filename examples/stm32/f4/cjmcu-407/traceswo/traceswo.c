/*
  JA

  5   OK - our stlink does not have a swo. pin.
   6     
   7     https://lujji.github.io/blog/stlink-clone-trace/
   8       After taking a closer look I noticed that there is no SWO pin on the pinout.
   9       Perhaps SWIM pin would dual-function as SWO when working with STM32?
  10       Unfortunately this wasn’t the case. So what, does it mean I have to use another
  11       USB cable just for UART? No way, I’m not going back to the stone age, I want my
  12       Trace!
  13   
  14     So he had to cut the trace. and solder a new one.
*/

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/tpiu.h>
#include <libopencm3/cm3/itm.h>

static void clock_setup(void)
{
	// rcc_clock_setup_in_hse_8mhz_out_72mhz();  JA

	/* Enable GPIOC clock. */
	// rcc_periph_clock_enable(RCC_GPIOC); // JA
   rcc_periph_clock_enable(RCC_GPIOE);
}

static void trace_setup(void)
{
	/* Enable trace subsystem (we'll use ITM and TPIU). */
	SCS_DEMCR |= SCS_DEMCR_TRCENA;

	/* Use Manchester code for asynchronous transmission. */
	TPIU_SPPR = TPIU_SPPR_ASYNC_MANCHESTER;
	TPIU_ACPR = 7;

	/* Formatter and flush control. */
	TPIU_FFCR &= ~TPIU_FFCR_ENFCONT;

	/* Enable TRACESWO pin for async mode. */
	DBGMCU_CR = DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE_ASYNC;

	/* Unlock access to ITM registers. */
	/* FIXME: Magic numbers... Is this Cortex-M3 generic? */
	*((volatile uint32_t *)0xE0000FB0) = 0xC5ACCE55;

	/* Enable ITM with ID = 1. */
	ITM_TCR = (1 << 16) | ITM_TCR_ITMENA;
	/* Enable stimulus port 1. */
	ITM_TER[0] = 1;
}

static void gpio_setup(void)
{
	/* Set GPIO12 (in GPIO port C) to 'output push-pull'. */
	// gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);    // JA
}

static void trace_send_blocking(char c)
{
	while (!(ITM_STIM8(0) & ITM_STIM_FIFOREADY))
		;

	ITM_STIM8(0) = c;
}

int main(void)
{
	int i, j = 0, c = 0;

  // rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	clock_setup();
	gpio_setup();
	trace_setup();

	/* Blink the LED (PC12) on the board with every transmitted byte. */
	while (1) {
		// gpio_toggle(GPIOC, GPIO12);	/* LED on/off */  JA
    gpio_toggle(GPIOE, GPIO0);

		trace_send_blocking(c + '0');
		c = (c == 9) ? 0 : c + 1;	/* Increment c. */
		if ((j++ % 80) == 0) {		/* Newline after line full. */
			trace_send_blocking('\r');
			trace_send_blocking('\n');
		}
		for (i = 0; i < 800000 * 10; i++)	/* Wait a bit. */   // JA *10 for hse.
			__asm__("nop");
	}

	return 0;
}
