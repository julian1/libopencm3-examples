/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2015 Jack Ziesing <jziesing@gmail.com>
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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>


#define LED1_PORT GPIOE
#define LED1_PIN  GPIO0




static void clock_setup(void)
{
  // OK. openocd locks up - just the same - even if we don't change the clocking.
	// rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

static void gpio_setup(void)
{
	/* Enable GPIO clock for leds. */
	rcc_periph_clock_enable(RCC_GPIOE); // JA

	/* Enable led as output */
   gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA

	gpio_set(LED1_PORT, LED1_PIN);
}

static void tim_setup(void)
{
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */
	// timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 10000));
	// timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 1000000));
	// timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 1000));
	// timer_set_prescaler(TIM2, 10 ); // appears to work ok.
                                  // but perhaps at 1, the interupt is running into each other. for delay of 10
  timer_set_prescaler(TIM2, 1 );    // doesn't work... if delay is 10. interupts
                                    // but appears ok if delay is set to 100

	/* Disable preload. */
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	/* count full range, as we'll update compare value continuously */
	timer_set_period(TIM2, 65535);

	/* Set the initual output compare value for OC1. */
	timer_set_oc_value(TIM2, TIM_OC1, 1 );   // could set to 1 or something?

	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

/*
  64000000 / 100000  64Mhz  / 100kHz.
  640
  eg. 1 / 640 resolution. but minimum delay is about 30. that's ok.
    that's just the limit

  30 / 640 = 1/20 = 5%. is minimum - duty.
*/


bool dutyx;


void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {

		/* Clear compare interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_CC1IF);

		/*
		 * Get current timer value to calculate next
		 * compare register value.
		 */
		uint16_t compare_time = timer_get_counter(TIM2);

		/* Calculate and set the next compare value. */
    // note we have 16 bits of resolution here.
    // could remove the bool - and just test the flipped values
    // if wanted dead time, then could use an array
    uint16_t delay;

    if(dutyx) {
      // led off
      // delay = 10000;
      delay = 610;
      gpio_set(LED1_PORT, LED1_PIN);
    }
    else {
      // clear turns led on
      delay = 30;   // 50 appears to work with prescalar of 1...
                    // 20 doesn't work.
                    // 30 appears to work. may still be glitchy.
      gpio_clear(LED1_PORT, LED1_PIN);
    }
    dutyx = !dutyx;

		// update the time
    // think this sets up the next interupt.
		timer_set_oc_value(TIM2, TIM_OC1, compare_time + delay ); // overflow seems ok. eg. it's a hardware > presumably.

	}
}


int main(void)
{
	clock_setup();
	gpio_setup();
	tim_setup();

	while (1) {
		;
	}

	return 0;
}
