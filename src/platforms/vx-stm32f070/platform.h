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

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H


#define PLATFORM_HAS_DEBUG
#define USBUART_DEBUG
extern bool debug_bmp;

//#define DEBUG(...) if (debug_bmp) {usbuart_debug_write("bmp: ");usbuart_debug_write(__VA_ARGS__);}


#include "timing.h"
#include "timing_stm32.h"
#include "version.h"

static inline int platform_hwversion(void) { return 0; }

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>


#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

#define BOARD_IDENT       "Black Magic Probe (vx), (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_DFU   "Black Magic (Upgrade) for STLink/Discovery, (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_UPD   "Black Magic (DFU Upgrade) for STLink/Discovery, (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT         "Black Magic Firmware Upgrade (STLINK)"
#define DFU_IFACE_STRING  "@Internal Flash   /0x08000000/8*001Ka,56*001Kg"
#define UPD_IFACE_STRING  "@Internal Flash   /0x08000000/8*001Kg"

/* Important pin mappings for STM32 implementation:
 *
 * LED0 = 	PB2	(Yellow LED : Running)
 * LED1 = 	PB10	(Yellow LED : Idle)
 * LED2 = 	PB11	(Red LED    : Error)
 *
 * TPWR = 	RB0 (input) -- analogue on mini design ADC1, ch8
 * nTRST = 	PB1
 * SRST_OUT = 	PA2
 * TDI = 	PA3
 * TMS = 	PA4 (input for SWDP)
 * TCK = 	PA5
 * TDO = 	PA6 (input)
 * nSRST = 	PA7 (input)
 *
 * USB cable pull-up: PA8
 * USB VBUS detect:  PB13 -- New on mini design.
 *                           Enable pull up for compatibility.
 * Force DFU mode button: PB12
 */

/* Hardware definitions... */
#define TDI_PORT	GPIOA
#define TMS_PORT	GPIOA
#define TCK_PORT	GPIOA
#define TDO_PORT	GPIOA
#define TDI_PIN		GPIO0
#define TMS_PIN		GPIO0
#define TCK_PIN		GPIO0
#define TDO_PIN		GPIO0

#define SWDIO_PORT 	GPIOA
#define SWCLK_PORT 	GPIOA
#define SWDIO_PIN	GPIO5
#define SWCLK_PIN	GPIO6

#define SRST_PORT	GPIOA
#define SRST_PIN_V1	GPIO0
#define SRST_PIN_V2	GPIO0

#define LED_PORT	GPIOA
/* Use PC14 for a "dummy" uart led. So we can observere at least with scope*/
#define LED_PORT_UART	GPIOA
#define LED_UART	GPIO0

#define TMS_SET_MODE() \
	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, \
	              GPIO_PUPD_NONE, TMS_PIN); \
	gpio_set_output_options(TMS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TMS_PIN);
#define SWDIO_MODE_FLOAT() \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, \
	              GPIO_PUPD_NONE, SWDIO_PIN);
#define SWDIO_MODE_DRIVE() \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, \
	              GPIO_PUPD_NONE, SWDIO_PIN); \
	gpio_set_output_options(SWDIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, SWDIO_PIN);

#define UART_PIN_SETUP() \
	gpio_mode_setup(USBUSART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USBUSART_TX_PIN); \
	gpio_set_output_options(USBUSART_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, USBUSART_TX_PIN);

#define USB_DRIVER      st_usbfs_v2_usb_driver
#define USB_IRQ	        NVIC_USB_IRQ 
#define USB_ISR	        usb_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART2 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB		(2 << 4)
#define IRQ_PRI_USBUSART	(1 << 4)
#define IRQ_PRI_USBUSART_TIM	(3 << 4)
#define IRQ_PRI_USB_VBUS	(14 << 4)
#define IRQ_PRI_TIM3		(0 << 4)

#define USBUSART USART2
#define USBUSART_CR1 USART2_CR1
#define USBUSART_IRQ NVIC_USART2_IRQ
#define USBUSART_CLK RCC_USART2
#define USBUSART_PORT GPIOA
#define USBUSART_TX_PIN GPIO2
#define USBUSART_ISR usart2_isr
#define USBUSART_TIM TIM3
#define USBUSART_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM3)
#define USBUSART_TIM_IRQ NVIC_TIM3_IRQ
#define USBUSART_TIM_ISR tim3_isr

//#define DEBUG(...)

extern uint16_t led_idle_run;
#define LED_IDLE_RUN            led_idle_run
#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, led_idle_run, state);}
#define SET_ERROR_STATE(x)

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf
#define snprintf sniprintf

#endif

