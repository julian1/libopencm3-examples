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

/*
  trick here - is that we don't use the arr reload. we just toggle the outputs when they reach
  the compare value
  --
  we can calculate distance/position - by taking count of main counter.
  can calculate inside an interupt. eg. schedule an interupt. and then measure in the interupt.
  if ALL SPEED/DIRECTION changes are done in an interupt... (can be simple write of values)
  then we have the OC value.

  --
  setting up, timer_slave_set_mode  might actually be simpler. because it
  is reading almost directly from the different oc clock outputs... eg.
  --
  "input capture" <- good search term.
  "The timers in the Stm32, in addition to the TIM6 and TIM7, have input capture
  capabilities for other timers. This mode is usually used in the calculation of
  the input signal frequency frequency, the duty ratio duty, the high and low
  pulse width, has the very wide use.  --
  https://topic.alibabacloud.com/a/stm32-timer-pwm-mode-input-capture_8_8_10261357.html
  --

  TIM_SMCR(timer_peripheral) &= ~TIM_SMCR_ETF_MASK;
  TIM_SMCR_TS_ITR0
   TIM_SMCR_TS_ETRF  - external trigger input

  this has the advantage - that *might* be able to microstep more easily. actually not sure.
    actually no. we only change phase relationship. for microsetp at slow speed. pwm ena and enb. use interrupts.
      and change the pulsing.

  "pwm input mode - can only be used with ch1 and ch2. due to fact only ti1fp1. and ti2fp2 are connected to
    the slave mode controller.
  -------------

  OR. we just enable another clock to run in parallel with a divider that is equal to the prescaler * period.
  eg. does 32bit.  there is no need to trigger.
  eg. 65000 * 1000 - is half a step.
  trigger would be better.
  ---------

  In the STM32F4 you can configure either TIM9 or TIM12 to act as the most
  significant word (MSW) of a 32-bit timer, which can be connected to another
  timer, which acts as the least significant word (LSW). The ST documentation
  refers to this as a "Synchronization circuit to control the timer with external
  signals and to interconnect several timers".

    https://www.rapitasystems.com/blog/chaining-two-16-bit-timers-together-stm32f4
    THIS IS VERY GOOD.

    The master - flips a bit to indicate that its the master
      TIM3->CR2 |= 0x20; /* MMS (6:4)

    Then the slave sets itself up to read,
      TIM9->SMCR |= (TIM_TS_ITR1 | TIM_SlaveMode_External1);


  eg. there is not a complete flexible multiplexiro. instead we just do a single thing.
  But there's an issue - that it can only could a single phase.
  That's maybe ok.
  - interupts can start and stop precisely....

  -----------
  VERY IMPORTANT.
    enable pulsing - for micro-stepping/ power control. should use the same trick. have two channels on a timer.
    but without the phase relationship. more conventional reset on ARR. 
    - being able to run different pwm duties on different channels of the same timer is very nice
  ------------
  we cannot know the actual phase - with a straight count of one of the channels.
  but we can determine it - by testing whether on an edge the other toggled channel is high or low.
  given the 180deg out of phase.
    - actually i think master count % 2. should give it. 
  -------

  we can work out the position using (slave count and count % 2) AND sampling the (master fast count count). 
  --------
  VERY IMPORTANT
  trigger on the rise and the fall.  then use count % 4 for the phase. actually think still %2 on count.


  or just rise and mod 2, and the master clock count.

*/

int main(void)
{

	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_TIM4);


  gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_af(GPIOD, GPIO_AF2, GPIO12 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO12 | GPIO13 | GPIO14 | GPIO15);


  rcc_periph_reset_pulse(RST_TIM4);   // good practice
	timer_set_prescaler(TIM4, 65535 ); // JA - blinks 1/s.

  // timer_disable_preload(TIM4);
  // timer_continuous_mode(TIM4);

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
