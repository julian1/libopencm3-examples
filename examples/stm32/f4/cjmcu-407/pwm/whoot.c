/*
  THIS WORKS WITH IF fIT EXTERNAL LED to PA8  not the on-board led.
  blinks once / second or so.

  PWM. good for leds, relays, motors, servos, power supplies.

  // PA8 is TIM1 CH1 see p55.

  based on the following code
  https://github.com/libopencm3/libopencm3-examples/pull/185/files

  ----------
  PWM1 - The output is active when the counter is less than the compare register contents and inactive otherwise.
  PWM2 - The output is inactive when the counter is less than the compare register contents and active otherwise.

  void  timer_set_deadtime (uint32_t timer_peripheral, uint32_t deadtime)
    TIM1 or TIM8 only.

  center aligned mode - that creates a triangle counter could be interesting. generating
  Ok - slightly staggered / or deadtime is easy
    just use the same timer, but a different channel - with a slightly different CCR time.
  -------

  // ok - having rotary encoder - means can control servo - etc. really interesting.

*/


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>



int main(void)
{

	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_TIM1);


  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 );
  gpio_set_af(GPIOA, GPIO_AF1, GPIO8 ); // AF1 == timer.
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8 ); // 50is faster than 100? no. same speed


  rcc_periph_reset_pulse(RST_TIM1);   // is this needed
	timer_set_prescaler(TIM1, 65535 ); // JA - blinks 1x/s. eg. consistent with 64MHz, which is documented .
	// timer_set_prescaler(TIM1, (rcc_apb1_frequency * 2) / 100 );
	// timer_set_prescaler(TIM1, (rcc_apb2_frequency ) / 10 );   // higher is faster - which makes no sense?
                                                               // is this overflowing...
                                                               // it's rougly correct -


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
