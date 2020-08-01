/*

  TRY  - LSI
  try bypass mode.
  Need - to understand what clock/ system the LSE uses.
  TRY - - The crystal oscillator driving strength can be changed at runtime using
        - the LSEDRV[1:0] bits in the Backup domain control register (RCC_BDCR) to obtain
        - the best compromise between robustne
        - try 8MHz HSE 

        - On the TFBGA216 package when the LSE low driving capability or LSE high driving
        capability is selected (LSEDRV[1:0]=00 or LSEDRV[1:0]=11 in the RCC_BDCR
        register, 


  https://github.com/libopencm3/libopencm3/issues/1028

  calender functions for f4. PR. has clock configuration.
    https://github.com/libopencm3/libopencm3/pull/1207/files

  Even if the crystal is bad - should there be something measureable at the pins.
  caps are 10p for the 32khz. 22p for 8mhz



  OK. maybe the crystal is no good. its a budget board.

    The oscillator circuits on the STM32 are a bit fussy. You'd want to
    evaluate the capacitors and the need for serial or parallel resistance. Perhaps
    there are adequate markings on your current selection to be able to pull a
    specification or data sheet?

  really need other discovery board. to test.
    freaking crystal oscillator X2 is not populated???? makes no sense. but they have two 8Mhz. why?

  --------
  On board - tested with multimeter.
    pins 8 and 9.  connect to outer oscillator.  inner both appear to go to ground.
      this matches the schematic.
      connect to 32k in, 32k out.
      and matches manual lqfp100. pins 8 and 9.

  rtc clock on PC14 and PC15 according to schematic

  PC14/OSC32_IN I/O (PC14) FT EVENTOUT OSC32_IN
  PC15/ OSC32_OUT (PC15) I/O FT 3) EVENTOUT OSC32_OUT (4)
    but don't appear in the alternative mappings.
    But they can be used as GPIO
    "When using the PC13 to PC15 GPIOs in output mode"

  - turning RCC_LSE rcc_osc_on, and but when call rcc_wait_for_osc_ready() it just hangs.
  - So testing the crystal on a scope - shows no oscillation.
  - as a test, i can see 8MHz crystal on scope no problems (after configure rcc_hse_8mhz_3v3)
  - VBAT - is which is related to osc powering is connected to 3.3 according to schematic. but it should work off vdd anyway.
  - so should replicate whatever rcc_hse_8mhz_3v3 does for the lse with power. doesn't seem to do anything.

  - crystal has 10p caps, according to datasheet. another datasheet uses 4.5p. seems reasonable. 
  - What sort of voltage output should I see?

  -- Everything just hangs... maybe check the pins.

		return RCC_BDCR & RCC_BDCR_LSERDY;

  - we need to know if the lse is on the ahb or apb. or unrelated. its a clock not a bus.


This is all that should be needed... see lcd-hello.c
For some reason it hangs however,

Maybe need

void rcc_osc_ready_int_enable(enum rcc_osc osc)

https://stackoverflow.com/questions/16468978/cant-get-the-rtc-to-work  <- RTC has all registers to setup for f1.

search for
  "stm32f4 RCC_BDCR_LSEON"


	rcc_peripheral_enable_clock (&RCC_AHBLPENR, RCC_AHBLPENR_GPIOALPEN
				     | RCC_AHBLPENR_GPIOBLPEN | RCC_AHBLPENR_GPIOCLPEN);


bool rcc_is_osc_ready(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_LSE:
		return RCC_BDCR & RCC_BDCR_LSERDY;


 51   pwr_disable_backup_domain_write_protect ();
 52   rcc_osc_on(RCC_LSE);
 53   rcc_wait_for_osc_ready(RCC_LSE);
 54   rcc_rtc_select_clock(RCC_CSR_RTCSEL_LSE);
 55   RCC_CSR |= RCC_CSR_RTCEN; // Enable RTC clock
 56   pwr_enable_backup_domain_write_protect ();


void rcc_osc_on(enum rcc_osc osc)
{
	switch (osc) {
  ....
	case RCC_LSE:
		RCC_BDCR |= RCC_BDCR_LSEON;
		break;


Bypass is for external clock, not external oscillator. eg. disable resonator circuit.

The LSE crystal is switched on and off using the LSEON bit in Backup domain control
register (RCC_BDCR). The crystal oscillator driving strength can be changed at runtime
using the LSEDRV[1:0] bits in the Backup domain control register (RCC_BDCR) to obtain
the best compromise between robustness and short start-up time on one side and low-
power-consumption on the other side. The LSE drive can be decreased to the lower drive
capability (LSEDRV=00) when the LSE is ON. However, once LSEDRV is selected, the
drive capability can not be increased if LSEON=1.
The LSERDY flag in the AHB1 peripheral clocks enable in Sleep and Stop modes register
(RCC_AHB1SMENR) indicates whether the LSE crystal is stable or not. At startup, the LSE
crystal output clock signal is not released until this bit is set by hardware. An interrupt can be
generated if enabled in the Clock interrupt enable register (RCC_CIER).


  LSE crystal 32 KHz osc
    BKP registers
    RCC BDCR register
    RTC

The PC14 and PC15 I/Os are only configured as LSE oscillator pins OSC32_IN and
OSC32_OUT when the LSE oscillator is ON. This is done by setting the LSEON bit
in the RCC_BDCR register. The LSE has priority over the GPIO function.

  see AF functions.
  see hse setup code, see what it does.

 */


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>

static void rtc_calendar_config(void)
{
  pwr_disable_backup_domain_write_protect();
  /* Enable LSE for using calendar */
  RCC_BDCR |= RCC_BDCR_LSEON;
  RCC_BDCR |= RCC_BDCR_RTCEN;
  RCC_BDCR |= (1<<8); /* RTCSEL at 0b01 */
  RCC_BDCR &= ~(1<<9); /* RTCSEL at 0b01 */


  // high drive
  RCC_BDCR |= RCC_BDCR_LSEDRV_HIGH;

  while(!(RCC_BDCR & RCC_BDCR_LSERDY));   // hangs

  pwr_enable_backup_domain_write_protect();
}

static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}

int main(void)
{

  // rcc_periph_clock_enable(RCC_GPIOC); // needed?
  // don't enable GPIO?
  rcc_periph_clock_enable(RCC_PWR);
  rcc_periph_clock_enable(RCC_LSE);
  rcc_periph_clock_enable(RCC_RTC);

  RCC_APB1ENR  |= RCC_APB1ENR_PWREN ;

  PWR_CR |= PWR_CR_DBP; // 1<<8. enable access 
  //PWR_CR |= (1UL << 8);

  // domain reset
  /*
    RCC_BDCR |= RCC_BDCR_BDRST;
    RCC_BDCR &= ~RCC_BDCR_BDRST;
  */

	led_setup();
  rtc_calendar_config();    // hangs...

	while (1) {
    int i;

    gpio_toggle(GPIOE, GPIO0);  // toggle led

    for (i = 0; i < 3000000; i++) {
			__asm__("nop");
		}
	}

  return 0;
}


