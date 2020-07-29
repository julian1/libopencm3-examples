/*
  ok - this uses stm32 internal timers, with phased channel outputs to drive a bipolar stepper motor.
  the advantage is that the cpu does not have to do the work.
  it should also be possible to configure a second slave counter - to count motor position. and
  it should be able to recover phase with modulus, and double edge counting. or else use master counter also.

  the disadvantage - is that to catch the motor in a specific position, (to stop it, change speed,dir etc)
  requires interupts and code, to modify the prescaling valuesetc. it would be
  possible to do the interrupt one-time as needed, on the second slave.  but it
  starts getting complicated - versus just doing everything on an interupt and using a fsm approach.
  ---
  actually maybe its not too bad - we set oc value for interupt - eg. so motor moves to position,
  then change what we need.
  also - if give up the phase positioning, which is hard to calculate, there is still plenty of control.
  it's not really any different.

  also interupts are probably the easiest way to acheive micro-stepping by allowing code to change the pwm
  power, inbetween detent points.

  So perhaps simpler to use interrupts - except if the motor is going to be spun at high speed
  and cpu cost is higher. at which point could switch approach, perhaps losing position, or calculating it
  with second counter.

*/


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>


/*
  trick here - is that we don't use the arr reload. we just toggle the outputs when they reach
  the compare value.
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

  actually - think could microstep using sawtooth/centre aligned count. but pwm of microstep is much
  higher freq. holding microstep - without advance ok, but slow advancinig would require code/interrupts.

  "pwm input mode - can only be used with ch1 and ch2. due to fact only ti1fp1. and ti2fp2 are connected to
    the slave mode controller.
  -------------


  In the STM32F4 you can configure either TIM9 or TIM12 to act as the most
  significant word (MSW) of a 32-bit timer, which can be connected to another
  timer, which acts as the least significant word (LSW). The ST documentation
  refers to this as a "Synchronization circuit to control the timer with external
  signals and to interconnect several timers".

  In the STM32F4 you can configure either TIM9 or TIM12 to act as the most
  significant word (MSW) of a 32-bit timer
  Why only these two

    https://www.rapitasystems.com/blog/chaining-two-16-bit-timers-together-stm32f4
    THIS IS VERY GOOD.

    CR1 and CR2 -
      compare register.

    For TIM4 may need ITR3 - see this chart,
      http://news.eeworld.com.cn/mcu/ic470164.html
      and then pick it up on TIM2 or TIM3?

    TIM3 master -> TIM9 slave -> when ITR1    # according to chart. which matches.
      For TIM4 can use TIM12 with ITR0 according to the same chart.
      But maybe able to do more.


    The master - flips a bit to indicate that its the master
      TIM3->CR2 |= 0x20; // MMS (6:4)

    Then the slave sets itself up to read,
      TIM9->SMCR |= (TIM_TS_ITR1 | TIM_SlaveMode_External1);


    in
      libopencm3, think would be the following,

      timer_set_master_mode()
        Set the Master Mode.  This sets the Trigger Output TRGO for synchronizing with
        slave timers or passing as an internal trigger to the ADC or DAC.

      TIMx_CR2 MMS[6:4]: Master Mode Selection
         #define  TIM_CR2_MMS_ENABLE   (0x1 << 4)
         #define  TIM_CR2_MMS_COMPARE_PULSE   (0x3 << 4)
         #define  TIM_CR2_MMS_COMPARE_OC1REF


      slave.
      timer_slave_set_mode()
        Slave mode selection, TIM_SMCR_SMS_ECM1   (0x7 << 0)  External Clock
        Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.

      timer_slave_set_trigger()
        TIM_SMCR_TS_ITR1   (0x1 << 4)  -- note there are four internal triggers available.
                                        -- so can set more than one.
                                        -- Actually think input determines from which timer



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
  reverse is still problematic. maybe record, then reset the slave count to zero.

  or just rise and mod 2, and the master clock count.
*/

/*
  OK. experimenting with this. would be a lot easier with a printf loop...
  We could then print the timer value in a loop.
  Do we try to do this in RTOS?
  
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
	timer_set_prescaler(TIM4, 65535 );  // blinks 1/s.

  // timer_disable_preload(TIM4);
  // timer_continuous_mode(TIM4);

  // timer is up counting.
  timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  // timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
  timer_enable_preload(TIM4);
  timer_enable_break_main_output(TIM4); // what does this do
  timer_set_period(TIM4, 2000);

  ////////
  // configure the channnel outputs to toggle on the oc (output compare) value

  // channel 1
  timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_TOGGLE);
  timer_enable_oc_output(TIM4, TIM_OC1);
  timer_set_oc_value(TIM4, TIM_OC1, 1000);

  // channel 2
  timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_TOGGLE); // OK. this inverts from PWM1. eg. its the bottom.
  timer_enable_oc_output(TIM4, TIM_OC2);
  timer_set_oc_value(TIM4, TIM_OC2, 1); // 1 not zero, to catch on the upward count...

  // chan 3, same as 1 except flip polarity
  timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_TOGGLE);
  timer_enable_oc_output(TIM4, TIM_OC3);
  timer_set_oc_value(TIM4, TIM_OC3, 1000);
  timer_set_oc_polarity_low(TIM4, TIM_OC3); // flip

  // chan4, same as 3 except flip polarity
  timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_TOGGLE);
  timer_enable_oc_output(TIM4, TIM_OC4);
  timer_set_oc_value(TIM4, TIM_OC4, 1);
  timer_set_oc_polarity_low(TIM4, TIM_OC4); // flip


  timer_enable_counter(TIM4);

	while (1) {

			__asm__("nop");
	}

	return 0;
}
