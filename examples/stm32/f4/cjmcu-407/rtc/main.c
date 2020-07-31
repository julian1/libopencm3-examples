/*
  rtc clock on PC14 and PC15 according to schematic

  PC14/OSC32_IN I/O (PC14) FT EVENTOUT OSC32_IN
  PC15/ OSC32_OUT (PC15) I/O FT 3) EVENTOUT OSC32_OUT (4)
    but don't appear in the alternative mappings.
    But they can be used as GPIO
    "When using the PC13 to PC15 GPIOs in output mode"

  - turning RCC_LSE rcc_osc_on, and but when call rcc_wait_for_osc_ready() it just hangs.
  - So testing the crystal on a scope - shows no oscillation. 
  - i can see 8MHz crystal on scope no problems - after configure rcc_hse_8mhz_3v3
  - VBAT - is which is related to osc powering is connected to 3.3 according to schematic.
  - so should replicate whatever rcc_hse_8mhz_3v3 does for the lse with power.
  

  - we need to know if the lse is on the ahb or apb.




This is all that should be needed... see lcd-hello.c
For some reason it hangs however,

Maybe need 

void rcc_osc_ready_int_enable(enum rcc_osc osc)

https://stackoverflow.com/questions/16468978/cant-get-the-rtc-to-work  <- RTC has all registers to setup for f1.

search for
  "stm32f4 RCC_BDCR_LSEON"


	rcc_peripheral_enable_clock (&RCC_AHBLPENR, RCC_AHBLPENR_GPIOALPEN
				     | RCC_AHBLPENR_GPIOBLPEN | RCC_AHBLPENR_GPIOCLPEN);
	

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

	

Bypass is for external clock, not external oscillator.

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

The PC14 and PC15 I/Os are only configured as LSE oscillator pins OSC32_IN and OSC32_OUT when the LSE oscillator is ON. This is done by setting the LSEON bit in the RCC_BDCR register. The LSE has priority over the GPIO function.

  see AF functions.
  see hse setup code, see what it does.

  calender functions for f4. PR.
    https://github.com/libopencm3/libopencm3/pull/1207/files
 */


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>



#include <libopencm3/stm32/rtc.h>


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>




static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA


}





// NVIC_RTC_WKUP_IRQ <- // this is the irq.
// white configures

#if 0
static void rtc_setup(void)
{
    rcc_periph_clock_enable(RCC_PWR);
    pwr_disable_backup_domain_write_protect();
    RCC_BDCR |= (1<<16);
    RCC_BDCR &= ~(1<<16);
    // rcc_osc_on(LSI); // JA
    rcc_wait_for_osc_ready(LSI);
    RCC_BDCR &= ~(RCC_BDCR_RTCSEL_HSE|RCC_BDCR_RTCSEL_LSE);
    RCC_BDCR |= RCC_BDCR_RTCSEL_LSI;
    RCC_BDCR |= (1<<15); //RTC enable

---
    rcc_periph_clock_enable(RCC_RTC);

*    rtc_unlock();
    RTC_ISR |=RTC_ISR_INIT;
    while ((RTC_ISR & RTC_ISR_INITF) != RTC_ISR_INITF)__asm__("nop");
    rtc_set_prescaler(127, 212);
    RTC_ISR &= ~(RTC_ISR_INIT);
    PWR_CR &= ~PWR_CR_PDDS;
    PWR_CR &= ~PWR_CR_LPDS;
    PWR_CSR |= PWR_CSR_EWUP;
    // SCB_SCR |= SCB_SCR_SLEEPDEEP; // JA
    DBGMCU_CR |= DBGMCU_CR_STOP;
    DBGMCU_CR |= DBGMCU_CR_STANDBY;
    DBGMCU_CR |= DBGMCU_CR_SLEEP;
*    rtc_lock();


    RCC_BDCR |= (1<<15); //RTC enable
    rcc_periph_clock_enable(RCC_RTC);
    rtc_wait_for_synchro();
    exti_enable_request(EXTI17);
    exti_set_trigger(EXTI17,EXTI_TRIGGER_RISING);
    nvic_enable_irq(NVIC_RTC_IRQ);
    rtc_unlock();
    RTC_CR &= ~RTC_CR_ALRAE;
    while((RTC_ISR & RTC_ISR_ALRAWF)!=RTC_ISR_ALRAWF);
    RTC_ALRMAR = RTC_ALRMXR_MSK4|RTC_ALRMXR_MSK3|RTC_ALRMXR_MSK2|RTC_ALRMXR_MSK1;
    RTC_ALRMASSR |= ((3)<<24);
    //RTC_TAFCR|=((1<<18)|(1<<19));
    RTC_CR = RTC_CR_ALRAIE|RTC_CR_ALRAE;//|RTC_CR_COE|RTC_CR_COSEL;
    RTC_ISR &= ~(RTC_ISR_ALRAF|RTC_ISR_ALRAWF);
    rtc_lock();
*    rtc_wait_for_synchro();
}
#endif


/*
  VBAT supply. ?? is it connected???
    according to oc. yes.

"The LSE clock source normally comes from a 32.768kHz external crystal This
clock is in the backup domain and so continues to run when only the V_BAT
supply is present. A prescaler value of 7FFF will give a 1 second count
quantum."

It really should be working by default - with sensible valuesi - because it's meant 
to run - even after mcu poweroff.
  

*/


int main(void)
{

  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	led_setup();


  rcc_periph_clock_enable(RCC_GPIOC); // needed?

  rcc_periph_clock_enable(RCC_PWR);
  // rcc_periph_clock_enable(RCC_LSE); // ?


  // RCC_CSR_LSEON;  // there is RCC_CSR_LSION but no RCC_CSR_LSEON ?????


  //Enable the power and backup interface clocks by setting the PWREN and BKPEN bitsin the RCC_APB1ENR register
   // RCC_APB1ENR |= (RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN); 
   RCC_APB1ENR |= (RCC_APB1ENR_PWREN ); 
  
 
  pwr_disable_backup_domain_write_protect ();

	rcc_osc_on(RCC_LSE);
	// rcc_wait_for_osc_ready(RCC_LSE);      // THIS STILL HANGS????
                                          // WHY...

	// rcc_rtc_select_clock(RCC_CSR_RTCSEL_LSE);        // JA
	// RCC_CSR |= RCC_CSR_RTCEN; JA	/* Enable RTC clock */
	pwr_enable_backup_domain_write_protect ();



    // see,
    //  rcc_set_rtc_clock_source(enum rcc_osc clk)
      // @brief RCC Set the Source for the RTC clock
		//RCC_BDCR =  RCC_BDCR_RTCSEL_LSE;
   //rcc_enable_rtc_clock();



	while (1) {
    int i;

    gpio_toggle(GPIOE, GPIO0);  // toggle led

    for (i = 0; i < 3000000; i++) {
			__asm__("nop");
		}
	}

  return 0;
}

