/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2013 Piotr Esden-Tempski <piotr@esden.net>
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
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>





static void usart_setup(void)
{

	/* Enable clock for USARTs gpio */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);


  //////////////////

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ); // JA

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9); // JA


	/* Setup GPIO pins for USART1 receive. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO10);

	/* Setup USART1 TX and RX pin as alternate function. */
	// gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);  // JA  tx

	// gpio_set_af(GPIOA, GPIO_AF7, GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO10); // JA    rx

	/* Setup USART1 parameters. */
	// usart_set_baudrate(USART1, 38400);
	usart_set_baudrate(USART1, 115200); // JA
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Enable USART1 Receive interrupt. */
	usart_enable_rx_interrupt(USART1);

	/* Finally enable the USART. */
	usart_enable(USART1);

}




/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART1, '\r');
			}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}



// static char mybuf[100]; 
static char *p = NULL;
static int sz = 0;

static int pos = 0;
static int last_char = 0;

// OK - the interrupt will actually prevent 
// interupt writes into a buffer.
// the read - has the while loop - waiting until done, 



static int read_line_buffered(char *p_, int sz_) {
  
  // bad return value
  p     = p_;
  sz   = sz_;
  last_char = 0;
  pos   = 0;

  while(pos < 5) {

      //printf("whoot %u\n", pos);
  };
  // while(last_char != '\r') { }

  return pos;
}



void usart1_isr(void)
{
  // Only thing this is doing - is echoing the output.
  // for console read I think we need blocking.
	static uint8_t data = 'A';


	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART1);

    if (pos < (sz - 1) ) {
      p[pos] = data; 
      pos++;
      p[pos] = 0; 
      last_char = data;
    } 

		/* Enable transmit interrupt so it sends back the data. */
    // JA - so this is a cheap way of echoing data???
		usart_enable_tx_interrupt(USART1);
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		/* Put data into the transmit register. */
		usart_send(USART1, data);

		/* Disable the TXE interrupt as we don't need it anymore. */
    // if disabled then it keeps sendingj
		usart_disable_tx_interrupt(USART1);
	}
}


// have no idea.

int main(void)
{
	// clock_setup();
	usart_setup();

  printf("hello %u\n", 1234); 
  printf("there\n"); 

	while (1) {
      char buf[ 10 ] ;
      read_line_buffered(buf , 10);
      printf("*********************************\r\n");
      printf("\r\n");
      // printf(buf);
      // printf("\n");

		__asm__("NOP");
	}

 
	return 0;
}


