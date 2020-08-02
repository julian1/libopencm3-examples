/*

 */


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>




static void rtc_calendar_config(void)
{

  pwr_disable_backup_domain_write_protect();

  RCC_BDCR &= ~RCC_BDCR_LSEBYP;		// 	bypass off	(1 << 2)
  // RCC_BDCR |= RCC_BDCR_LSEBYP;		// 	bypass on (1 << 2)


  RCC_BDCR &= ~(RCC_BDCR_RTCSEL_LSI << RCC_BDCR_RTCSEL_SHIFT);    // 2 << shift
  RCC_BDCR |= RCC_BDCR_RTCSEL_LSE << RCC_BDCR_RTCSEL_SHIFT;       // eg. 1 << shift

  RCC_BDCR |= RCC_BDCR_LSEON;   // IS THIS CORRECTLY BITSHIFTED???? ok. (1 << 0)
  RCC_BDCR |= RCC_BDCR_RTCEN;     // #define RCC_BDCR_RTCEN				(1 << 15)


  // PWR_CR |= (1UL << 8);
  // RCC_BDCR |= 3 << 3;  /* RCC_BDCR[4:3]: LSEDRV */

  while(!(RCC_BDCR & RCC_BDCR_LSERDY));   // wait for oscillator to sync

  pwr_enable_backup_domain_write_protect();
}



static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}


int main(void)
{
  rcc_periph_clock_enable(RCC_PWR); // possible this sets the bit..
  rcc_periph_clock_enable(RCC_LSE);
  rcc_periph_clock_enable(RCC_RTC);

  // RCC_APB1ENR  |= RCC_APB1ENR_PWREN ;  // (1 << 28) // enables registers described in chapter Power control (PWR).
                                        // not needed here, but required for standby etc.

	led_setup();
  rtc_calendar_config();

  // RTC_CR is a configuration register.

	while (1) {
    if( RTC_TR % 2 == 0)
      gpio_clear(GPIOE, GPIO0);
    else
      gpio_set(GPIOE, GPIO0);
 	}

  return 0;
}


