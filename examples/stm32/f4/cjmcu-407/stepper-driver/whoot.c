/*
  OK. we can run this on a timer with interupts
  BUT.
    Can it be run directly with a timer. with its four outputs?
    it just needs the OC values to be correctly staggered.
    needs to hold the output values - if the counter is stopped - it can. they are just turned off/on

      Issue is that some pins have to be started in the on condition...
      need ramp up and down.
      centre aligned - eg. ramp.
    - it probably won't be a pwm control... through...

    https://www.youtube.com/watch?v=zkrVHIcLGww
    ARR - auto reload value. on overflow.
      up count.
      down count.
      centre aligned. eg. sawtooth counter.
    CCR - compare and capture register.

    - So. think it can be done. but it would need to counters.
    and they would have to be started at the same time.

    mode1 - edge aligned on left.
    mode 2 - edge aligned on right.

    centre aligned. no. its not right.

    CCER - can program polarity.
      therefore - reduces problem to just having staggered of 2 signals.

  looks like can start two timers at the same time.
  timer_enable_counter(TIM2 | TIM3);

  BUT - if we change period - we cannot synchronize - the prescaler.
  "staggered"
  0 1000
  turn 1 on and 2 off.
  on 250 turn 2 on and 1 off.

  think can do it with centre aligned. we have one signal with OC 
  and the otherflowing or repeating.
  see RCR register.
 
  OK. maybe much simpler.  just using up counter.
    Can generate overflow - not on every cycle. but every second cycle.
    eg. RCR = 2.

  Or can do it - with master and slave synchronization. between timers. fuck.

  1 timer - 4 channels. each channel has its own CCR register.
  -----
  two outputs 50 % out of phase.
    - two timers. problem of synchronisation.
    - using centre aligned counting - to get the offet one. and the other can use auto reload at bottom and  
    - RCC. ? still  
    - 

  RCC=2.  one to clock on overflow. and the other on underflow.

  180 phase shift.
  staggered.

  someone tries to do it with  
    https://community.st.com/s/question/0D50X00009XkiXnSAJ/how-to-generate-two-pwm-with-180deg-phase-shift-with-stm32f30303-advance-timer

  100kHz. with interrupts.
 
  OK - this is looking more like it. using CCR. and phase. 
  http://www.micromouseonline.com/2016/02/05/clock-pulses-with-variable-phase-stm32/
 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void gpio_setup(void)
{
	/* Enable GPIOD clock. */
	/* Manually: */
	// RCC_AHB1ENR |= RCC_AHB1ENR_IOPDEN;
	/* Using API functions: */
	// rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE); // JA

	rcc_periph_clock_enable(RCC_GPIOD); // JA

	/* Set GPIO12 (in GPIO port D) to 'output push-pull'. */
	/* Manually: */
	// GPIOD_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((8 - 8) * 4) + 2));
	// GPIOD_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((8 - 8) * 4));
	/* Using API functions: */
	// gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);


   // gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_OTYPE_OD, GPIO0); // JA
   gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA


   gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 ); // JA

}

// think coil has 4 power positions.
// +5,0, +5,+5, 0,+5, 0,0,
// but we won't use them all.
// and we have to get the polarity right...
/*
static int ia[][4] = {
  { 1,0,   0, 1 },
  { 1,1,   0, 0 },
  { 0,1,   1, 0 },
  { 0,0,   1, 1 },
};
*/
// just use modulus?


// wiring, actually doesn't seem to matter, will just spin forward/reverse
// 42byg https://www.youtube.com/watch?v=pID6TTgg1HU
// wave-form
// http://rfanat.ru/stanki_chpu/BIPOLAR_Stepper_Motor_Driver_Circuits_74194.html

/*
  3.75V
    turning quite fast 1 rps. uses 0.16A
    1 revolution every 10 sec uses 1.4 amps.
    runnning a click every second uses 1.69 amps. which is a lot.

    eg. when slow - its almost an open short.
*/

int main(void)
{
	uint32_t i = 0;
	int j = 0;


	gpio_setup();

  // this needs to be set/high - to get the voltage switch
  // alternatively we pulse this ... to reduce current...
  gpio_set(GPIOD, GPIO0);   // JA enable A
  gpio_set(GPIOD, GPIO5);   // enable B


  // full wave drive
  // seems to be working
	while (1) {

    ++i;
    switch(i % 4) {
    // switch(3 - (i % 4)) {
        // reverse would just be 3 - the value

      // NO... the pins are around the wrong way.
      // the wave form pins - are different. for particular pair should never be both set or both clear.

      case 0:
        // gpio_set(  GPIOD, GPIO1);  gpio_clear(GPIOD, GPIO2);  gpio_set(GPIOD, GPIO3);  gpio_clear(GPIOD, GPIO4);
        gpio_set(GPIOD, GPIO1 | GPIO3); gpio_clear(GPIOD, GPIO2 | GPIO4);
        break;
      case 1:
        // gpio_set(  GPIOD, GPIO1);  gpio_clear(GPIOD, GPIO2);  gpio_clear(GPIOD, GPIO3);  gpio_set(GPIOD, GPIO4);
        gpio_set(GPIOD, GPIO1 | GPIO4); gpio_clear(GPIOD, GPIO2 | GPIO3);
        break;
      case 2:
        // gpio_clear(  GPIOD, GPIO1);  gpio_set(GPIOD, GPIO2);  gpio_clear(GPIOD, GPIO3);  gpio_set(GPIOD, GPIO4);
        gpio_set(GPIOD, GPIO2 | GPIO4); gpio_clear(GPIOD, GPIO1 | GPIO3);
        break;
      case 3:
        // gpio_clear(  GPIOD, GPIO1);  gpio_set(GPIOD, GPIO2);  gpio_set(GPIOD, GPIO3);  gpio_clear(GPIOD, GPIO4);
        gpio_set(GPIOD, GPIO2 | GPIO3); gpio_clear(GPIOD, GPIO1 | GPIO4);
        break;
    }



    switch((i / 200) % 2 ) {

      case 0:  {
        gpio_clear(GPIOE, GPIO0);

        for (j = 0; j < 10000; j++)
          __asm__("nop");

        break;
      }

      case 1: {
        gpio_set(GPIOE, GPIO0);

        for (j = 0; j < 30000; j++)
          __asm__("nop");
        break;
      }

    }

    // for (j = 0; j < 3000; j++) { // about the fastest it will go. 10rpm? need to increase the voltage past 10V.
    // for (j = 0; j < 5000; j++) { //
    // for (j = 0; j < 10000; j++) { /* Wait a bit. */
    // for (j = 0; j < 30000; j++) { /* Wait a bit. */
    // for (j = 0; j < 100000; j++) { /* Wait a bit. */
    /*
    for (j = 0; j < 2000000; j++) {
			__asm__("nop");
		}
    */
	}

	return 0;
}


