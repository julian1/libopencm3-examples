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

// So lets try to slow the counter down.

/*
timer functions.
  http://libopencm3.org/docs/latest/stm32f4/html/group__timer__file.html


*/

// prescale is 16 bit only... eg. 1 and 65535.
// it really

// issue - maybe we have to PE0 is associated with tim4_etr  whatever the hell that is. 
/*
  So this is why the example at, uses TIM2 and gpioA
    http://libopencm3.org/docs/latest/stm32f4/html/group__timer__file.html

  see p49 of manual.
  Pa0 -> TIM2 TIM2_CH1_ETR/   wake up.
  PA1 -> TIM2 CH 2
  PA2 -> TIM2 CH 3
  PA3 -> TIM2 CH 4

  ETR  - event trigger? - for input?

  So. we should probabaly put a led on PA0. as the first thing. just to align evertying...
  easy. just solder something using PP.

  3.3V led 1k  pin.

  
  PWM1 - The output is active when the counter is less than the compare register contents and inactive otherwise.
  PWM2 - The output is inactive when the counter is less than the compare register contents and active otherwise.

  void  timer_set_deadtime (uint32_t timer_peripheral, uint32_t deadtime)
    TIM1 or TIM8 only.

  center aligned mode - that creates a triangle counter could be interesting. generating
  Ok - slightly staggered / or deadtime is easy
    just use the same timer, but a different channel - with a slightly different CCR time.

  ---------
  example that should work,
  https://github.com/libopencm3/libopencm3-examples/pull/185/files

*/

int main(void)
{

  // PA8 is TIM1 CH1 see p/

	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_TIM1);

  
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 );
  gpio_set_af(GPIOA, GPIO_AF1, GPIO8 );
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8 ); // 50is faster than 100?

	timer_set_prescaler(TIM1, 65535 ); // JA - blinks 1x/s. eg. consistent with 64MHz, which is documented .


  timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
  timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
  timer_enable_oc_output(TIM1, TIM_OC1);
  timer_enable_break_main_output(TIM1);
  timer_set_oc_value(TIM1, TIM_OC1, 100);

  timer_enable_preload(TIM1);

  timer_set_period(TIM1, 1000);
  timer_enable_counter(TIM1);


   
	while (1) {

			__asm__("nop");
	}

	return 0;
}
