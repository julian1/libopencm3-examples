/*
  rtc clock on PC14 and PC15 according to schematic

  PC14/OSC32_IN I/O (PC14) FT EVENTOUT OSC32_IN 
  PC15/ OSC32_OUT (PC15) I/O FT 3) EVENTOUT OSC32_OUT (4)
    but don't appear in the alternative mappings.
    But they can be used as GPIO
    "When using the PC13 to PC15 GPIOs in output mode"


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

  It should work in oscillator mode
  And it should have additional circuitry - eg. the pull up.

  Goal.
    To get the external oscillator working
    get the clock to work

  need bypass 

  see
    https://community.st.com/s/question/0D50X00009XkfRhSAJ/hsi-hse-lsi-lse-and-clock-nonsense
      has external oscillator on pc14,p15

  ../../../../../libopencm3/include/libopencm3/stm32/f4/rtc.h
  https://github.com/libopencm3/libopencm3-examples/pull/154/files


  lots of functions here,
  http://libopencm3.org/docs/latest/stm32f4/html/group__rtc__file.html

  calender functions for f4. PR.
    https://github.com/libopencm3/libopencm3/pull/1207/files
 */


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>



#include <libopencm3/stm32/rtc.h>


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>




static void gpio_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA


}



void rtc_wkup_isr(void)
{
  // OK - this name is correct

 /* clear flag, not write protected */
 RTC_ISR &= ~(RTC_ISR_WUTF);


  // Actually think its EXTI22.
 // exti_reset_request(NVIC_RTC_WKUP_IRQ);

  exti_reset_request(EXTI22);

 // state.rtc_ticked = true;

  gpio_toggle(GPIOE, GPIO0);  // JA
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



int main(void)
{

	gpio_setup();

	pwr_disable_backup_domain_write_protect ();
	rcc_osc_on(RCC_LSE);
	rcc_wait_for_osc_ready(RCC_LSE);      // THIS STILL HANGS????

	// rcc_rtc_select_clock(RCC_CSR_RTCSEL_LSE);        // JA
	// RCC_CSR |= RCC_CSR_RTCEN; JA	/* Enable RTC clock */
	pwr_enable_backup_domain_write_protect ();


	while (1) {
    int i;

    gpio_toggle(GPIOE, GPIO0);  // JA

    for (i = 0; i < 1000000; i++) { /* Wait a bit. */
			__asm__("nop");
		}
	}

  return 0;
}


static int main1(void)
{
	int i;

  // setting ...
  // should turn on abp1 and abp2
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	gpio_setup();

  // rtc clock.
  // rcc_periph_clock_enable(RCC_GPIOC); // JA board rtc gpio
                                      // Should be apb2 
                                        // not sure this is correct.
  rcc_periph_clock_disable(RCC_GPIOC); // disable

  // it should work in oscillator mode on boot.
  // gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO14 | GPIO15); // JA


  rcc_periph_clock_enable(RCC_LSE);
  rcc_periph_clock_enable(RCC_RTC);
  rcc_periph_clock_enable(RCC_PWR);

  // lsi = low speed internal 
  // WE WANT LSE  lowspeed external

  rcc_osc_on(RCC_LSE);
  rcc_wait_for_osc_ready(RCC_LSE);  // OK. this hangs.

  // RCC_CSR, RCC_CSR_RTCSEL_MASK, and RCC_CSR_RTCSEL_SHIFT.
  // rcc_rtc_select_clock(RCC_CSR);



  rtc_unlock();           // must be before rtc_enable_wakeup_timer

  // RCC_BDCR |= RCC_BDCR_RTCSEL_LSE;
  // RCC_BDCR |= (1<<15); //RTC enable



  rtc_set_prescaler(1, 1);


  rtc_enable_wakeup_timer();
  rtc_enable_wakeup_timer_interrupt();

 

  rtc_lock();           // must be before rtc_enable_wakeup_timer
  // rtc_wait_for_synchro(); // OK. this is blocking on what... why?




  // rcc_osc_on(RCC_RTC);
  // rcc_wait_for_osc_ready(RCC_LSE);

  // nvic_setup();

	// c = rtc_get_counter_val();

	/* Display the current counter value in binary via USART1. */

	while (1) {

    gpio_toggle(GPIOE, GPIO0);  // JA

    for (i = 0; i < 5000000; i++) { /* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}
