/*
  OK. we can run this on a timer with interupts
  BUT.
    Can it be run directly with a timer. with its four outputs?
    it just needs the OC values to be correctly staggered.
    needs to hold the output values - if the counter is stopped - it can. they are just turned off/on

  OK - this is looking more like it. using CCR. and phase.
  http://www.micromouseonline.com/2016/02/05/clock-pulses-with-variable-phase-stm32/

  eg. not using the OC, and ARR - as the points. just CCR
  --------

  Ok - is there an issue - that we lose our position. if we don't have a meta counter.
    perhaps requires another slave counter? or do an interrupt for counting.
  ------------

    might just be timer_set_continuous.
    then setting the preload. for the next time.
    should be possible to microstep - quite easily. just vary the pulse.

    OK - it's just the same configuration as the rip saw. only triggering on the value. 
    except we use an up counter.
  ---------------

  WE DO HAVE THE MOTOR POSITION - we can calculate when needed by recording the time
    it has run been run at a particular speed and direction.
    eg. its just a division.
    - can set an interrupt
      - then do the calculation inside an interrupt to be accurate.
      - can also do a change in frequcney inside the interupt
    
  OR. should be ok to do slave mode. and use another counter.
    albeit need to be careful with direction.


    -------

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
    // i += dir; // where dir = +1 or -1, or 0 hold/microstep.
    // can change speed in herre. easily 
    // if i == 100, then stop, or change speed etc, run acceleration profiles.
    // can run complete data driven profile. start stop reverse. etc.
    // based on motor position i. or just time j.
    // also can increase resolution - to handle pwm on enable pins - for pwm
    // eg. use modulo %40. then have 10 steps. where microstep - using pwm.
    // controlling all of direction, power, duration, on/off, hold.
    // also other thing
    switch(i % 4) {

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


