/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "serial_telemetry.h"
#include "common.h"
#include "sport_telemetry.h"

// TX pin control for half-duplex operation
void telem_TX_enable(void)
{
    // Set TX pin to output mode (drive)
    gpio_init_type gpio_init_struct;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_6;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    
    usart_transmitter_enable(USART1, TRUE);
}

void telem_TX_disable(void)
{
    // Wait for transmission to complete
    while (usart_flag_get(USART1, USART_TDC_FLAG) == RESET);
    
    usart_transmitter_enable(USART1, FALSE);
    
    // Set TX pin to tri-state (input/high-impedance)
    gpio_init_type gpio_init_struct;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_6;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP; // Weak pull-up to keep line high when tri-state
    gpio_init(GPIOB, &gpio_init_struct);
}

void send_telem_DMA(uint8_t bytes)
{ 
    // Enable TX mode before transmission
    telem_TX_enable();
    
    // Set data length and enable channel to start transfer
    DMA1_CHANNEL4->dtcnt = bytes;
    DMA1_CHANNEL4->ctrl_bit.chen = TRUE;
}

void telem_UART_Init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    /* Initially configure TX pin as input (tri-state) with pull-up */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_6;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP; // Keep line high when tri-state
    gpio_init(GPIOB, &gpio_init_struct);

    dma_reset(DMA1_CHANNEL4);
    dma_flexible_config(DMA1,FLEX_CHANNEL4,DMA_FLEXIBLE_UART1_TX);
    dma_init_type dma_init_struct;
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = sizeof(aTxBuffer);
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)aTxBuffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA1_CHANNEL4, &dma_init_struct);

    DMA1_CHANNEL4->ctrl |= DMA_FDT_INT;
    DMA1_CHANNEL4->ctrl |= DMA_DTERR_INT;

    /* configure usart1 param */
    gpio_pin_remap_config(USART1_MUX, TRUE);
    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, FALSE); // Start with TX disabled
    usart_receiver_enable(USART1, TRUE);
    usart_single_line_halfduplex_select(USART1, TRUE);
    usart_dma_transmitter_enable(USART1, TRUE);
    
    // Enable USART RX interrupt for SPORT polling detection
    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    nvic_irq_enable(USART1_IRQn, 2, 0);
    
    usart_enable(USART1, TRUE);

    nvic_irq_enable(DMA1_Channel4_IRQn, 3, 0);
}
