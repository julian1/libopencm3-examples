/*
  OK. put it in bypass mode.
  and then inject 30khz with a sig-gen and we can get past the hang.
  and it continues to work between resets so long as there is power. eg. because it remembers the sync

  Can actually do the same thing - in non-bypass mode.

 */


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>




#if 0
  /* Enable RTC and BackUp Registers  */                                      // Page 118 of RM0090 Reference Manual
  PWREN_bit = 1;                                                              // PWREN: Power interface clock enable.      (RCC APB1 peripheral clock enable register (RCC_APB1ENR))
  DBP_bit = 1;                                                                // Access to RTC Backup registers enabled.   (PWR power control register (PWR_CR))
  RTCSEL1_bit = 0;  RTCSEL0_bit = 1;                                          // RTC Clock MUX select LSE as clock source. (RCC Backup domain control register (RCC_BDCR))
  LSEBYP_bit = 0;                                                             // LSE oscillator not bypassed.              (RCC Backup domain control register (RCC_BDCR))
  LSEON_bit = 1;                                                              // External low-speed oscillator enable.     (RCC Backup domain control register (RCC_BDCR))
  RTCEN_bit = 1;                                                              // RTC clock enabled.                        (RCC Backup domain control register (RCC_BDCR))


  /* Disable the write protection for RTC registers */                        // Page 785 of RM0090 Reference Manual
  DBP_bit = 1;                                                                // Access to RTC Backup registers enabled.   (PWR power control register (PWR_CR))
  RTC_WPR = 0xCA;
  RTC_WPR = 0x53;


  while ((LSERDY_bit == 0) && (RTC_Wait_ctr < 150));
  {
       delay_us(1);
       RTC_Wait_ctr++;
  }

#endif


static void rtc_calendar_config(void)
{
  RCC_APB1ENR  |= RCC_APB1ENR_PWREN ;

  pwr_disable_backup_domain_write_protect();

  // RCC_BDCR &= ~RCC_BDCR_LSEBYP;		// 	bypass off	(1 << 2)
  RCC_BDCR |= RCC_BDCR_LSEBYP;		// 	bypass on (1 << 2)

    //  RCC_BDCR |= (1<<8); /* RTCSEL at 0b01 */  // eg. RCC_BDCR_RTCSEL_LSE << RCC_BDCR_RTCSEL_SHIFT;
  // RCC_BDCR &= ~(1<<9); /* RTCSEL at 0b01 */    // eg. RCC_BDCR_RTCSEL_LSI <<RCC_BDCR_RTCSEL_SHIFT

/*
#define RCC_BDCR_RTCSEL_SHIFT			8
#define RCC_BDCR_RTCSEL_MASK			0x3
#define RCC_BDCR_RTCSEL_NONE			0
#define RCC_BDCR_RTCSEL_LSE			1
#define RCC_BDCR_RTCSEL_LSI			2
#define RCC_BDCR_RTCSEL_HSE			3
*/
  RCC_BDCR &= ~(RCC_BDCR_RTCSEL_LSI << RCC_BDCR_RTCSEL_SHIFT);    // 2 << shift
  RCC_BDCR |= RCC_BDCR_RTCSEL_LSE << RCC_BDCR_RTCSEL_SHIFT;       // eg. 1 << shift

  RCC_BDCR |= RCC_BDCR_LSEON;   // IS THIS CORRECTLY BITSHIFTED???? ok. (1 << 0)
  RCC_BDCR |= RCC_BDCR_RTCEN;     // #define RCC_BDCR_RTCEN				(1 << 15)


  // PWR_CR = 0x53;

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


  // PWR_CR |= PWR_CR_DBP; // 1<<8. enable access
  //PWR_CR |= (1UL << 8);

  // domain reset
  /*
    RCC_BDCR |= RCC_BDCR_BDRST;
    RCC_BDCR &= ~RCC_BDCR_BDRST;
  */

	led_setup();
  rtc_calendar_config();    // hangs...

  // RTC_CR is a configuration register.

	while (1) {
    if( RTC_TR % 2 == 0)
      gpio_clear(GPIOE, GPIO0);
    else
      gpio_set(GPIOE, GPIO0);
 	}


	while (1) {
    int i;

    gpio_toggle(GPIOE, GPIO0);  // toggle led

    for (i = 0; i < 3000000; i++) {
			__asm__("nop");
		}
	}

  return 0;
}


