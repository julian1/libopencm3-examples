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

/*
  Uses a timer - to generates interrupts - and in the interupt - we read and then update
    to generate the next interupt time.
  A bad way to do pwm - because at a certain high speed interupts will fall on top of each other.
  But may be useful in other contexts
*/

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
    // what is pupd - pull-up, pull-down.
   gpio_mode_setup(LED1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED1_PIN); // JA

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

  // TODO read from the array
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



// ok - we want to be able to control it - either uart. or pot.
/*
  very important
    - adc should probably sample at some fixed point in the duty cycle - due to ripple. even if only every 10 due to slower read time.
        so adc read - should probably be configured on the same timer. eg. TIM2
    - the min pulse - ignores jjjjjjjjj
    - should have a nominal load. 5% power, from a resistor or something.
    - we need to get some coupled inductors. use lm2577 data sheet.
    - if one has a mcu - does it make sense that all voltage rails are driven by it. rather than selecting lm2577.
    --------------
    think about mosfet switch and slew time. cannot switch a big mosfet on and off in 1 millionth of a second. so what we are doing
      maybe reasonable.  and don't need counter directly triggered on hardware
    - slower osc clock - eg. 50kHz, and bigger inductor, then can get minimum duty to 2.5% which would be nicer
    ----------
    - what is the lowest prescaler - is it 0 or is it 1 ? - need to check.
        manual 2.2.21. p31. prescalar can be from 1 to 65536. eg. not 0.
    - mcu - has very high clock rate. therefore minimym on time duty cycle is likely to be favorable o dedicated part
    - albeit analog/continuous - will have more resolution
    -----
    - stlink has a 5V output. available. probably direct from usb.
    -----
    can *not* use gpio at 5V.
      see 2.2.14 manual. p24.  it's all 3.3V. just different circuits for analog,reset and digital.
      It might make sense to use different linear regulators for those circuits though?-
    Vse Vdd
    ------
    ok  - reason not to use a mcu control loop - for steup-up and generating 12V from 5V - we need a mosfet with 5V gate drive.
        mtp3055 o
    -------
    mosfet driver - why not just npn with open collector pullup for 20mA. then 2 tx emitter follower - push-pull.  all at 5V.
        remember its not a particularly - low powered design - which means avoiding open-collector.

*/

typedef struct MyPWM  {

  // uint16_t   timer.          // eg. could read the timer to use.
  uint16_t    on_delay;
  uint16_t    off_delay;
  bool        dutyx;       // phase/cycle.
  // bool        enabled;      // on/off . control. No. should just stop timer - and set port to low?

  // prescaler...  should not be here. because it's common and global.
} MyPWM;


MyPWM  x;



void tim2_isr(void)
{

  MyPWM  *pwm = &x;

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

    // IMPORTANT - We should probably do this as the first thing - after clearing the interrupt flag.
    // eg. before we do timer_get_counter. actually not sure. timer_get_counter is just before the gpio change. which is right.
    if(pwm->dutyx) {
      // led off
      gpio_set(LED1_PORT, LED1_PIN);
      delay = pwm->off_delay;
      pwm->dutyx = false;
    }
    else {
      // clear - turns led on? eg. sinks?
      gpio_clear(LED1_PORT, LED1_PIN);
      delay = pwm->on_delay;
      pwm->dutyx = true;
    }

		// update the time
    // think this sets up the next interupt.
		timer_set_oc_value(TIM2, TIM_OC1, compare_time + delay ); // overflow seems ok. eg. it's a hardware > presumably.
	}
}

// it really

int main(void)
{
	clock_setup();
	gpio_setup();
	tim_setup();


  uint16_t    i = 0;
  uint16_t    period = 640;
  MyPWM       *pwm   = &x;  // pwm01.


  // initial
  pwm->dutyx     = true;
  pwm->on_delay  = 30;
  pwm->off_delay = period - pwm->on_delay;

                    // 50 appears to work with prescalar of 1...
                    // 20 doesn't work.
                    // 25 doesn't work.

	while (1) {
    uint16_t    j;
    ++i;

    for (j = 0; j < 5000; j++) { /* Wait a bit. */
			__asm__("nop");
		}

    pwm->on_delay  = i % 610  + 30;
    pwm->off_delay = period - pwm->on_delay;
		;
	}

	return 0;
}
