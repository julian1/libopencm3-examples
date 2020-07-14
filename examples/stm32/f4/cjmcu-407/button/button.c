/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
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

/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

// JA
// led -> D -> E, GPIO0 -> GPIO0
// button GPIOD unchanged. GPIO0 -> GPIO15
// button GPIOD -> GPIOD GPIO15



static void gpio_setup(void)
{
	/* Enable GPIOE clock. */
	rcc_periph_clock_enable(RCC_GPIOE);

	/* Set GPIO0 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 );
}

static void button_setup(void)
{
	/* Enable GPIOD clock. - for led */
	rcc_periph_clock_enable(RCC_GPIOD);

	/* Set GPIOD0 to 'input floating'. */
	gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO15 );

  //// GPIO_MODE_FLOATING is another option.

  // PULLUP - means pull the pin up. not pullup resistor.
	// gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO15 );
	// gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO15 );
}

int main(void)
{
	int i;

	clock_setup();
	button_setup();
	gpio_setup();

	/* Blink the LED (PD12) on the board. */
	while (1) {
		gpio_toggle(GPIOE, GPIO0);

		/* Upon button press, blink more slowly. */
		if (gpio_get(GPIOD, GPIO15)) {
			for (i = 0; i < 3000000; i++) {	/* Wait a bit. */
				__asm__("nop");
			}
		}

		for (i = 0; i < 3000000; i++) {		/* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}
