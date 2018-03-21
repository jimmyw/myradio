/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/i2c.h>
#include "u8g2.h"

u8g2_t u8g2;
uint8_t u8x8_gpio_and_delay_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);
      gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);
      // Init 
      break;
    case U8X8_MSG_DELAY_NANO:
      /* not required for SW I2C */
      break;
    
    case U8X8_MSG_DELAY_10MICRO:
      /* not used at the moment */
      break;
    
    case U8X8_MSG_DELAY_100NANO:
      /* not used at the moment */
      break;
   
    case U8X8_MSG_DELAY_MILLI:
      //for(int i = 0; i < 1000; i++) __asm__("nop");
      break;
    case U8X8_MSG_DELAY_I2C:
      /* delay by 5 us when arg_int is 1, or 1.25us otherwise */
      //if (arg_int)
      //  for(int i = 0; i < 10; i++) __asm__("nop");
      //else
      //  for(int i = 0; i < 2; i++) __asm__("nop");
      break;
    
    case U8X8_MSG_GPIO_I2C_CLOCK:
      
      if ( arg_int == 0 )
      {
        gpio_clear(GPIOB, GPIO10);
      }
      else
      {
        gpio_set(GPIOB, GPIO10);
      }
      break;
    case U8X8_MSG_GPIO_I2C_DATA:
      
      if ( arg_int == 0 )
      {
        gpio_clear(GPIOB, GPIO11);
      }
      else
      {
        gpio_set(GPIOB, GPIO11);
      }
      break;
    default:
      break;
  }
  return 1;
}

volatile uint32_t systick_millis = 0;

static void systick_setup(void) {
  // By default the Dash CPU will use an internal 16mhz oscillator for the CPU
  // clock speed.  To make the systick timer reset every millisecond (or 1000
  // times a second) set its reload value to:
  //   CPU_CLOCK_HZ / 1000
  systick_set_reload(16000);
  // Set the systick clock source to the main CPU clock and enable it and its
  // reload interrupt.
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();
  systick_interrupt_enable();
}

void sys_tick_handler(void) {
  // Increment the global millisecond count.
  systick_millis++;
}

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
  systick_setup();
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,0);
  gpio_set(GPIOB, GPIO3);
  gpio_set(GPIOB, GPIO4);
  
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);

}

static void adc_setup(void)
{
	int i;

	rcc_periph_clock_enable(RCC_ADC1);

	/* Make sure the ADC doesn't run during config. */
	adc_power_off(ADC1);

	/* We configure everything for one single conversion. */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	/* We want to read the temperature sensor, so we have to enable it. */
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

static void my_usart_print_int(uint32_t usart, int value)
{
	int8_t i;
	uint8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = nr_digits; i >= 0; i--)
		usart_send_blocking(usart, buffer[i]);
}

int main(void)
{
	uint8_t channel_array[16];
	uint16_t temperature;

  rcc_clock_setup_in_hse_12mhz_out_72mhz();
 
  gpio_setup(); 
	usart_setup();
	adc_setup();

  u8g2_Setup_ssd1306_i2c_128x64_noname_2(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_i2c);

  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  u8g2_ClearBuffer(&u8g2);


  /* Select the channel we want to convert. 16=temperature_sensor. */
  channel_array[0] = 16;
  adc_set_regular_sequence(ADC1, 1, channel_array);
  
  int j = 0;
	while(1) {
    u8g2_FirstPage(&u8g2);
    do {
      u8g2_DrawLine(&u8g2, 64-j, 64-j, j, j);
      u8g2_SetFont(&u8g2, u8g2_font_amstrad_cpc_extended_8f);
      u8g2_DrawStr(&u8g2, 10, 10, "Hello world");
    } while ( u8g2_NextPage(&u8g2) );

    if (j == 64) {
      j=0;
    } else {
      j++;
    }
    /* Send a message on USART1. */
    //usart_send_blocking(USART1, 'a');
    //usart_send_blocking(USART1, 'd');
    //usart_send_blocking(USART1, 'c');
    //usart_send_blocking(USART1, ':');
    //usart_send_blocking(USART1, ' ');

    //adc_start_conversion_direct(ADC1);
    //while (!(ADC_SR(ADC1) & ADC_SR_EOC)) {};
    //temperature = ADC_DR(ADC1);
    //my_usart_print_int(USART1, temperature);
    //usart_send_blocking(USART1, '\n');


  }; /* Halt. */

	return 0;
}
