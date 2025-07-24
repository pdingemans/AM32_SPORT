/**
 * Debug UART implementation for AT32F421
 * Uses PB4 as USART2 TX for debug output
 */

#include "debug_uart.h"
#include <stdarg.h>
#include <string.h>

#ifdef DEBUG_SERIAL

void debug_uart_init(void)
{
    gpio_init_type gpio_init_struct;
    
    // Enable USART2 and GPIOB clocks
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    // Configure PB4 as USART2_TX (alternate function)
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_4;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    
    // Set PB4 to USART2_TX function (AF0)
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_4);
    
    // Configure USART2
    usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART2, USART_PARITY_NONE);
    usart_hardware_flow_control_set(USART2, USART_HARDWARE_FLOW_NONE);
    usart_receiver_enable(USART2, FALSE);  // TX only
    usart_transmitter_enable(USART2, TRUE);
    usart_enable(USART2, TRUE);
}

void debug_printf(const char* format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Send each character via USART2
    for (int i = 0; i < strlen(buffer); i++) {
        while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET);
        usart_data_transmit(USART2, buffer[i]);
    }
}

#endif // DEBUG_SERIAL
