/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the ST-Link
 * implementation.
 */

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>

uint8_t running_status;
volatile uint32_t timeout_counter;

uint16_t led_idle_run;

void platform_init(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_CRC);

	/* Setup GPIO ports */
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
	gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, TCK_PIN);
	gpio_set_mode(TDI_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, TDI_PIN);
	uint16_t srst_pin = SRST_PIN_V1;
	gpio_set(SRST_PORT, srst_pin);
	gpio_set_mode(SRST_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_OPENDRAIN, srst_pin);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, led_idle_run);
	/* initialize swd port */
	gpio_set_mode(SWCLK_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, SWCLK_PIN);
	AFIO_MAPR = (AFIO_MAPR & ~ (7 << 24)) | (0b010 << 24);
	gpio_set_mode(SWDIO_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, SWDIO_PIN);

	SCB_VTOR = 0x0000; /* Relocate interrupt vector table here */

	platform_timing_init();
	cdcacm_init();
	usbuart_init();
}

void platform_srst_set_val(bool assert)
{
	uint16_t pin;
	pin = SRST_PIN_V1;
	if (assert)
		gpio_clear(SRST_PORT, pin);
	else
		gpio_set(SRST_PORT, pin);
}

bool platform_srst_get_val()
{
	uint16_t pin;
	pin = SRST_PIN_V1;
	return gpio_get(SRST_PORT, pin) == 0;
}

const char *platform_target_voltage(void)
{
	return "unknown";
}

void platform_request_boot(void)
{
	while (1);
}


#define SWCLK_HI		do { GPIOB_BSRR = 1 << 5; } while (0);
#define SWCLK_LOW		do { GPIOB_BRR = 1 << 5; } while (0);
#define SWCLK_PULSE		do { GPIOB_BSRR = 1 << 5; GPIOB_BRR = 1 << 5; } while (0);
#define SWDIO_HI		do { GPIOA_BSRR = 1 << 15; } while (0);
#define SWDIO_LOW		do { GPIOA_BRR = 1 << 15; } while (0);
#define SWDIO_READ		((GPIOA_IDR & (1 << 15)) ? 1 : 0)

#pragma GCC optimize ("O3")

struct
{
	unsigned	seq_in;
	unsigned	seq_in_parity;
	unsigned	seq_out;
	unsigned	seq_out_parity;
}
counters;

static inline void swdptap_turnaround(uint8_t dir)
{
	static uint8_t olddir = 0;

	/* Don't turnaround if direction not changing */
	if(dir == olddir) return;
	olddir = dir;

	DEBUG("%s", dir ? "\n-> ":"\n<- ");

	if(dir)
		SWDIO_MODE_FLOAT();
	SWCLK_PULSE
	if(!dir)
		SWDIO_MODE_DRIVE();
}

static inline int swdptap_bit_in_vx(void)
{
	uint16_t ret;

	swdptap_turnaround(1);

	ret = SWDIO_READ;
	SWCLK_PULSE
                
	return ret;
}

uint32_t swdptap_seq_in(int ticks)
{
	counters.seq_in ++;
	if (/* ;-) */ ticks == 3)
	{
		int res = 0;
		swdptap_turnaround(1);
		if (SWDIO_READ)
			res |= 1;
		SWCLK_PULSE
		if (SWDIO_READ)
			res |= 2;
		SWCLK_PULSE
		if (SWDIO_READ)
			res |= 4;
		SWCLK_PULSE
		return res;
	}
	else
	{
		uint32_t index = 1;
		uint32_t ret = 0;

		while (ticks--) {
			if (swdptap_bit_in_vx())
				ret |= index;
			index <<= 1;
		}

		return ret;
	}
}

bool swdptap_seq_in_parity(uint32_t *ret, int ticks)
{
	uint32_t index = 1;
	uint8_t parity = 0;
	*ret = 0;
	counters.seq_in_parity ++;

	while (ticks--) {
		if (swdptap_bit_in_vx()) {
			*ret |= index;
			parity ^= 1;
		}
		index <<= 1;
	}
	if (swdptap_bit_in_vx())
		parity ^= 1;

	return parity;
}

void swdptap_seq_out(uint32_t MS, int ticks)
{
	counters.seq_out ++;
	swdptap_turnaround(0);

	while (ticks--) {
		if (MS & 1)
			SWDIO_HI
		else
			SWDIO_LOW
		SWCLK_PULSE
		MS >>= 1;
	}
}

void swdptap_seq_out_parity(uint32_t MS, int ticks)
{
	int parity;
	counters.seq_out_parity ++;
	parity = MS ^ (MS >> 16);
	parity = parity ^ (parity >> 8);
	parity = parity ^ (parity >> 4);
	parity = parity ^ (parity >> 2);
	parity = parity ^ (parity >> 1);

	swdptap_seq_out(MS, ticks);
	if (parity & 1)
		SWDIO_HI
	else
		SWDIO_LOW
	SWCLK_PULSE
}
