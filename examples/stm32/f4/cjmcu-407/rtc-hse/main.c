/*

 */


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>

static void rtc_config(void)
{
  pwr_disable_backup_domain_write_protect();

#if 0
  /* Enable LSE for using calendar */
  RCC_BDCR |= RCC_BDCR_LSEON;
  RCC_BDCR |= RCC_BDCR_RTCEN;
  RCC_BDCR |= (1<<8); /* RTCSEL at 0b01 */
  RCC_BDCR &= ~(1<<9); /* RTCSEL at 0b01 */

  while(!(RCC_BDCR & RCC_BDCR_LSERDY));   // hangs

#endif

  // select high speed internal clock source
  // #define RCC_BDCR_RTCSEL_HSE			(3 << RCC_BDCR_RTCSEL_SHIFT)
  RCC_BDCR |= (RCC_BDCR_RTCSEL_HSE << RCC_BDCR_RTCSEL_SHIFT);     // BUG in library? Not shifted?
  RCC_BDCR |= RCC_BDCR_RTCSEL_HSE ;
  // RCC_BDCR &= ~RCC_BDCR_RTCSEL_LSE;

   // RCC_BDCR &= ~(RCC_BDCR_RTCSEL_HSE|RCC_BDCR_RTCSEL_LSE);

  // enable rtc
  //   RCC_BDCR |= (1<<15); //RTC enable
  // #define RCC_BDCR_RTCEN				(1 << 15)
  RCC_BDCR |= RCC_BDCR_RTCEN;

  rcc_periph_clock_enable(RCC_RTC);


    rtc_unlock();
    RTC_ISR |=RTC_ISR_INIT;
    //while ((RTC_ISR & RTC_ISR_INITF) != RTC_ISR_INITF)__asm__("nop"); // hangs

    rtc_set_prescaler(127, 212);
    rtc_lock();


   // rtc_wait_for_synchro(); // hangs

  pwr_enable_backup_domain_write_protect();
}

static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}


#if 0
static void rtc_setup(void)
{
    rcc_periph_clock_enable(RCC_PWR);
    pwr_disable_backup_domain_write_protect();
    RCC_BDCR |= (1<<16);
    RCC_BDCR &= ~(1<<16);
    rcc_osc_on(LSI);
    rcc_wait_for_osc_ready(LSI);
    RCC_BDCR &= ~(RCC_BDCR_RTCSEL_HSE|RCC_BDCR_RTCSEL_LSE);
    RCC_BDCR |= RCC_BDCR_RTCSEL_LSI;
    RCC_BDCR |= (1<<15); //RTC enable
    rcc_periph_clock_enable(RCC_RTC);
    rtc_unlock();
    RTC_ISR |=RTC_ISR_INIT;
    while ((RTC_ISR & RTC_ISR_INITF) != RTC_ISR_INITF)__asm__("nop");
    rtc_set_prescaler(127, 212);
    RTC_ISR &= ~(RTC_ISR_INIT);
    PWR_CR &= ~PWR_CR_PDDS;
    PWR_CR &= ~PWR_CR_LPDS;
    PWR_CSR |= PWR_CSR_EWUP;
    SCB_SCR |= SCB_SCR_SLEEPDEEP;
    DBGMCU_CR |= DBGMCU_CR_STOP;
    DBGMCU_CR |= DBGMCU_CR_STANDBY;
    DBGMCU_CR |= DBGMCU_CR_SLEEP;
    rtc_lock();
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
    rtc_wait_for_synchro();
}
#endif


int main(void)
{

  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_PWR);

  // No 
  // rcc_periph_clock_enable(RCC_HSE); // 
  rcc_wait_for_osc_ready(RCC_HSE);  // ok. 

  // rcc_periph_clock_enable(RCC_RTC); // do later

/*
  RCC_APB1ENR  |= RCC_APB1ENR_PWREN ;

  PWR_CR |= PWR_CR_DBP; // 1<<8. enable access
  //PWR_CR |= (1UL << 8);
*/

	led_setup();
  rtc_config();    // hangs...

	while (1) {
    int i;

    gpio_toggle(GPIOE, GPIO0);  // toggle led

    for (i = 0; i < 3000000; i++) {
			__asm__("nop");
		}
	}

  return 0;
}


