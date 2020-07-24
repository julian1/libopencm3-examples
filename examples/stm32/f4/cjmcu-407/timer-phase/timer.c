/*
*/


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

// OK. THIS IS EXACTLY THE SAME DAMN CODE. EXCEPT CHANGED FOR TIM4 
// AF... AF2... not AF1

//	timer_set_oc_polarity_high(TIM4, TIM_OC1);

//	timer_continuous_mode(TIM1);  nope.
/* Reset repetition counter value. */
//	timer_set_repetition_counter(TIM1, 0);



int main(void)
{

	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_TIM4);


  gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_af(GPIOD, GPIO_AF2, GPIO12 | GPIO13 | GPIO14 | GPIO15); 
  gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO12 | GPIO13 | GPIO14 | GPIO15);


  rcc_periph_reset_pulse(RST_TIM4);   // good practice
	timer_set_prescaler(TIM4, 65535 ); // JA - blinks 1x/s. eg. consistent with 64MHz, which is documented .
	// timer_set_prescaler(TIM4, (rcc_apb1_frequency * 2) / 100 );
	// timer_set_prescaler(TIM4, (rcc_apb2_frequency ) / 10 );   // higher is faster - which makes no sense?
                                                               // is this overflowing...
                                                               // it's rougly correct -
  // timer_disable_preload(TIM4);
  // timer_continuous_mode(TIM4);



  // CMS centre

  timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  // timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP); 
  timer_enable_preload(TIM4);
  timer_enable_break_main_output(TIM4); // what does this do
  timer_set_period(TIM4, 2000);

  ////////
  // PWM2 is saw. i think

  timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_TOGGLE);
  timer_enable_oc_output(TIM4, TIM_OC1);
  timer_set_oc_value(TIM4, TIM_OC1, 1000);


  timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_TOGGLE); // OK. this inverts from PWM1. eg. its the bottom. 
  timer_enable_oc_output(TIM4, TIM_OC2);
  timer_set_oc_value(TIM4, TIM_OC2, 1); // 1 not zero, to catch on the upward count...


  timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_TOGGLE);
  timer_enable_oc_output(TIM4, TIM_OC3);
  timer_set_oc_value(TIM4, TIM_OC3, 1000);
  timer_set_oc_polarity_low(TIM4, TIM_OC3); // same as OC1. except flip polarity




  timer_enable_counter(TIM4);


	while (1) {

			__asm__("nop");
	}

	return 0;
}
