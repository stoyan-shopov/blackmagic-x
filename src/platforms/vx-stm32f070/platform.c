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
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>

uint8_t running_status;
volatile uint32_t timeout_counter;

uint16_t led_idle_run;

static bool platform_sforth_entry_requested(void)
{
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO14);
	return (GPIOC_IDR & (1 << 14)) ? true : false;
}

static bool sforth_mode_active;
bool is_sforth_mode_active(void) { return sforth_mode_active; }

static void rcc_clock_setup_in_hse_8mhz_out_48mhz(void)
{
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_HSE);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

	/* 8MHz * 12 / 2 = 48MHz */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL12);

	RCC_CFGR &= ~RCC_CFGR_PLLSRC;

	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 48000000;
	rcc_ahb_frequency = 48000000;
}

void platform_init(void)
{
	rcc_set_usbclk_source(RCC_PLL);
	rcc_clock_setup_in_hse_8mhz_out_48mhz();

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Setup GPIO ports */
	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TMS_PIN);
	gpio_set_output_options(TMS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TMS_PIN);
	
	gpio_mode_setup(TCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TCK_PIN);
	gpio_set_output_options(TCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TCK_PIN);
	
	gpio_mode_setup(TDI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TDI_PIN);
	gpio_set_output_options(TDI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TDI_PIN);
	
	
	uint16_t srst_pin = SRST_PIN_V1;
	gpio_set(SRST_PORT, srst_pin);
	
	gpio_mode_setup(SRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, srst_pin);
	gpio_set_output_options(SRST_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, srst_pin);
	
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, led_idle_run);
	gpio_set_output_options(LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, led_idle_run);

	/* initialize swd port */
	gpio_mode_setup(SWCLK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SWCLK_PIN);
	gpio_set_output_options(SWCLK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, SWCLK_PIN);
	
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SWDIO_PIN);
	gpio_set_output_options(SWDIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, SWDIO_PIN);

	//SCB_VTOR = 0x0000; /* Relocate interrupt vector table here */
	if (platform_sforth_entry_requested())
		sforth_mode_active = true;

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


#define SWCLK_HI		do { GPIOA_BSRR = 1 << 6; } while (0);
#define SWCLK_LOW		do { GPIOA_BRR = 1 << 6; } while (0);
#define SWCLK_PULSE		do { GPIOA_BSRR = 1 << 6; GPIOA_BRR = 1 << 6; } while (0);
#define SWCLK_PULSE_DELAY	do { volatile int i; GPIOB_ASRR = 1 << 6; i = 5; while (i--); GPIOA_BRR = 1 << 6; i = 5; while (i--); } while (0);
#define SWDIO_HI		do { GPIOA_BSRR = 1 << 5; } while (0);
#define SWDIO_LOW		do { GPIOA_BRR = 1 << 5; } while (0);
#define SWDIO_READ		((GPIOA_IDR & (1 << 5)) ? 1 : 0)
#define SWDIO_READ_MSB		((GPIOA_IDR & (1 << 5)) << 26)

#define SWDIO_BIT_PORT_ADDR		& GPIOA_BSRR
#define SWDIO_SET_BIT_PORT_MASK		(1 << 5)
#define SWDIO_RESET_BIT_PORT_MASK	(1 << (16 + 5))

#define SWCLK_BIT_PORT_ADDR		& GPIOA_BSRR
#define SWCLK_SET_BIT_PORT_MASK		(1 << 6)
#define SWCLK_RESET_BIT_PORT_MASK	(1 << (16 + 6))

static const struct sw_driving_data
{
	uint32_t	swdio_set_reset_port_address;
	uint32_t	swdio_set_bit_mask;
	uint32_t	swdio_reset_bit_mask;
	uint32_t	swclk_set_reset_port_address;
	uint32_t	swclk_set_bit_mask;
	uint32_t	swclk_reset_bit_mask;
}
vx_sw_driving_data =
{
	.swdio_set_reset_port_address	=	SWDIO_BIT_PORT_ADDR,
	.swdio_set_bit_mask		=	SWDIO_SET_BIT_PORT_MASK,
	.swdio_reset_bit_mask		=	SWDIO_RESET_BIT_PORT_MASK,
	.swclk_set_reset_port_address	=	SWCLK_BIT_PORT_ADDR,
	.swclk_set_bit_mask		=	SWCLK_SET_BIT_PORT_MASK,
	.swclk_reset_bit_mask		=	SWCLK_RESET_BIT_PORT_MASK,
};


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

static bool swdptap_seq_in_parity_32bits_optimized(uint32_t * ret)
{
uint32_t x = 0;
		swdptap_turnaround(1);
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE x >>= 1;
		x |= SWDIO_READ_MSB; SWCLK_PULSE

		* ret = x;
		x ^= x >> 16;
		x ^= x >> 8;
		x ^= x >> 4;
		x ^= x >> 2;
		x ^= x >> 1;
		
		if (swdptap_bit_in_vx())
			x ^= 1;

		return x & 1;
}

bool swdptap_seq_in_parity(uint32_t *ret, int ticks)
{
	if (ticks == 32)
		return swdptap_seq_in_parity_32bits_optimized(ret);
	else
	{
		uint32_t index = 1;
		uint32_t parity = 0;
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
}


static void swdptap_seq_out_32bits_optimized_asm(struct sw_driving_data * sw, uint32_t data) __attribute__((naked));
static void swdptap_seq_out_32bits_optimized_asm(struct sw_driving_data * sw, uint32_t data)
{
	asm("push	{ r4, r5, r6, lr }");
	asm("mov	r6,	r1");
	asm("ldmia	r0,	{ r0, r1, r2, r3, r4, r5 }");

	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");
	/* drive data */
	asm("lsr	r6,	r6,	#1");
	asm("str	r1,	[r0]");
	asm("bcs	1f");
	asm("str	r2,	[r0]");
	asm("1:");
	/* pulse clock */
	asm("str	r4,	[r3]");
	asm("str	r5,	[r3]");

	asm("pop	{ r4, r5, r6, pc }");
}


void swdptap_seq_out_32bits_optimized(uint32_t x)
{
//static const uint32_t masks[2] = { SWDIO_RESET_BIT_PORT_MASK, SWDIO_SET_BIT_PORT_MASK, };
	//if (x & 1) SWDIO_HI else SWDIO_LOW SWCLK_PULSE x >>= 1;
	//SWDIO_BIT_PORT_ADDR = masks[x & 1]; SWCLK_PULSE x >>= 1;
int i = 32;
	while (i --)
	{
		* SWDIO_BIT_PORT_ADDR = (uint32_t[2]){SWDIO_RESET_BIT_PORT_MASK, SWDIO_SET_BIT_PORT_MASK, } [x & 1]; SWCLK_PULSE x >>= 1;
	}
}

void swdptap_seq_out(uint32_t MS, int ticks)
{
	swdptap_turnaround(0);
	if (ticks == 32)
		swdptap_seq_out_32bits_optimized_asm(& vx_sw_driving_data, MS);
	else
	{
		counters.seq_out ++;

		while (ticks--) {
			if (MS & 1)
				SWDIO_HI
			else
				SWDIO_LOW
			SWCLK_PULSE
			MS >>= 1;
		}
	}
}

void swdptap_seq_out_parity(uint32_t MS, int ticks)
{
	int parity;
	counters.seq_out_parity ++;
	parity = MS ^ (MS >> 16);
	parity ^= parity >> 8;
	parity ^= parity >> 4;
	parity ^= parity >> 2;
	parity ^= parity >> 1;

	swdptap_seq_out(MS, ticks);
	if (parity & 1)
		SWDIO_HI
	else
		SWDIO_LOW
	SWCLK_PULSE
}
