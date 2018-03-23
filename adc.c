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
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>
#include "u8g2.h"
#include "spi.h"

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
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	//rcc_periph_clock_enable(RCC_AFIO);

  // Turn off jtag mode
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,0);
  gpio_set(GPIOB, GPIO3);
  gpio_set(GPIOB, GPIO4);
 
  // Enable output on the leds 
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);

}

#define ADC_CHANNEL_COUNT 4
static uint16_t adc_data[ADC_CHANNEL_COUNT];

static void adc_setup(void)
{
	int i;
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_DMA1);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO2);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO6);
    
	/* Make sure the ADC doesn't run during config. */
	adc_power_off(ADC1);

	//adc_disable_scan_mode(ADC1);
  adc_enable_scan_mode(ADC1); /*scan mode means we scan all channels of the group to the end */
  //adc_set_continuous_conversion_mode(ADC1); /* means we scan the group the whole day til someone disable continous mode */
  //adc_disable_discontinuous_mode_regular(ADC1); /* discontinous mode means all channels in a group will be */
  
  adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	adc_set_right_aligned(ADC1);


	/* We configure everything for one single conversion. */
	adc_set_single_conversion_mode(ADC1);
  //adc_set_continuous_conversion_mode(ADC1);
	//adc_disable_external_trigger_regular(ADC1);
  //adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
  //adc_disable_analog_watchdog_regular(ADC1);
	//adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
  //adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_041DOT5);
	adc_power_on(ADC1);
   

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
  
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");
  // On NAZE32 board, PPM input 4 (channel 3) goes thru a digital or gate, and
  // is not suitable for ADC
  // PPM 5 is mapped to channel 6 
  uint8_t channels[16] = { 0, 1, 2, 6};
  adc_set_regular_sequence(ADC1, 4, channels);
  adc_enable_dma(ADC1);

    
}

static void adc_dma_arm(void) {
    // start conversion
    dma_enable_channel(DMA1, DMA_CHANNEL1);
    adc_start_conversion_regular(ADC1);
}

static void adc_init_dma(void) {

    // clean init
    dma_channel_reset(DMA1, DMA_CHANNEL1);

    // DO NOT use circular mode, we will retrigger dma on our own!
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);


    // high priority
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);

    // source and destination 16bit
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);

    // automatic memory destination increment enable.
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);

    // source address increment disable
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);

    // Location assigned to peripheral register will be source
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

    // source and destination start addresses
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_data);

    // chunk of data to be transfered
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_CHANNEL_COUNT);

    //dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);


}



static void int2buf(uint16_t value, char *buffer)
{
	uint8_t nr_digits = 0;

	if (value < 0) {
		buffer[nr_digits++] = '-';
		value = value * -1;
	}

  if (value == 0) {
		buffer[nr_digits++] = '0';
  }

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}
  buffer[nr_digits] = '\0';
}

static void int2bufbin(uint16_t value, char *buffer)
{
  int nr_digits = 0;
	for (int i = 16; i > 0; i--) {
		buffer[nr_digits++] = (value & (1<<i)) ? '1' : '0';
	}
  buffer[nr_digits] = '\0';
}

void int2bufhex(int16_t num, char *outbuf)
{

  int i = 12;
  int j = 2;
  outbuf[0]='0';
  outbuf[1]='x';
  if (num < 0) {
    outbuf[j++] = '-';
    num = num * -1;
  }


  do {
    outbuf[i] = "0123456789ABCDEF"[num % 16];
    i--;
    num = num/16;
  } while( num > 0);

  while( ++i < 13) {
    outbuf[j++] = outbuf[i];
  }

  outbuf[j] = 0;

}


int main(void)
{
  rcc_clock_setup_in_hse_12mhz_out_72mhz();
 
  gpio_setup(); 
	//usart_setup();
  //systick_setup();
	adc_setup();
  adc_init_dma();
  spi_init();

  u8g2_Setup_ssd1306_i2c_128x64_noname_2(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_i2c);

  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  u8g2_ClearBuffer(&u8g2);


  int j = 0;
  char buf[32];
	while(1) {

  // Clear dma interrupt, run adc and wait for new interrupt
  dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
  adc_dma_arm();
  while (!dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
    __asm__("nop");
    static int16_t adjusted[ADC_CHANNEL_COUNT];
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
      adjusted[i] = (int16_t)adc_data[i] - 0x800;
      if (adjusted[i] < -0x600)
        adjusted[i] = -0x600;
      if (adjusted[i] > 0x600)
        adjusted[i] = 0x600;
    }

    // Render screen
    u8g2_FirstPage(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_amstrad_cpc_extended_8f);
    gpio_clear(GPIOB, GPIO4);
    do {
      //u8g2_DrawLine(&u8g2, 64-j, 64-j, j, j);

      u8g2_DrawCircle(&u8g2, 32 + (adjusted[2] * 32 / 0x600), 32 + (adjusted[3] * 32 / 0x600), 2, U8G2_DRAW_ALL);
      u8g2_DrawCircle(&u8g2, 96 + (adjusted[0] * 32 / 0x600), 32 + (adjusted[1] * 32 / 0x600), 2, U8G2_DRAW_ALL);

      for (int i = 0; i < 4; i++) {
        //int2bufhex(adjusted[i], buf);
        //u8g2_DrawStr(&u8g2, 0, (i * 10) + 10, buf);
        u8g2_DrawLine(&u8g2, 0, (i * 2) + 57, (0x600 + adjusted[i]) * 128 / 0xc00, (i * 2) + 57); // 128 px wide screen
      }
    } while ( u8g2_NextPage(&u8g2) );
    gpio_set(GPIOB, GPIO4);
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

  }; /* Halt. */

	return 0;
}
