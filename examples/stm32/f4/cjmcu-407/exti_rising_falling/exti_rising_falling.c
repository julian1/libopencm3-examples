/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

#define FALLING 0
#define RISING 1

uint16_t exti_direction = FALLING;

// So why the fuck. doesn't this work...

/* Set STM32 to 72 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOE); // led
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}


static void exti_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_SYSCFG) ;

	gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO15 );

  // really not quite sure what EXTI15_10 means 15 or 10?
  // see code example, https://sourceforge.net/p/libopencm3/mailman/message/28510519/
  // defn, libopencm3/include/libopencm3/stm32/f4/nvic.h

  // nvic_enable_irq(NVIC_EXTI0_IRQ);
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

	/* Configure the EXTI subsystem. */
	exti_select_source(EXTI15, GPIOD);
	exti_direction = FALLING;
	exti_set_trigger(EXTI15, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI15);
}



void exti15_10_isr(void)
// void exti0_isr(void)
{
	exti_reset_request(EXTI15);

  // this might be getting other interupts also... not sure.

  // see, https://sourceforge.net/p/libopencm3/mailman/libopencm3-devel/thread/CAJ%3DSVavkRD3UwzptrAGG%2B-4DXexwncp_hOqqmFXhAXgEWjc8cw%40mail.gmail.com/#msg28508251
  // uint16_t port = gpio_port_read(GPIOE);
  // if(port & GPIO1) {
  // uint16_t EXTI_PR_ = EXTI_PR;
  // if(EXTI_PR_ & GPIO15) {

	if (exti_direction == FALLING) {
    gpio_set(GPIOE, GPIO0);
		exti_direction = RISING;
		exti_set_trigger(EXTI15, EXTI_TRIGGER_RISING);
	} else {
    gpio_clear(GPIOE, GPIO0);
		exti_direction = FALLING;
		exti_set_trigger(EXTI15, EXTI_TRIGGER_FALLING);
	}


}



int main(void)
{
	clock_setup();
	gpio_setup();
	exti_setup();

  // gpio_set(GPIOE, GPIO0); // JA

	while (1)
		__asm("nop");


/*
	while (1) {

		if (gpio_get(GPIOD, GPIO15)) {
		  gpio_set(GPIOE, GPIO0);
		}
    else {
		  gpio_clear(GPIOE, GPIO0);
    }
	}
*/


	return 0;
}
