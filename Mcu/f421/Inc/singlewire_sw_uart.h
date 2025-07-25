/**
 * Single-wire Software UART implementation
 * Generic bit-banging UART for inverted and non-inverted protocols
 * Hardware independent layer for single-wire communication
 */

#ifndef SINGLEWIRE_SW_UART_H
#define SINGLEWIRE_SW_UART_H

#include "at32f421.h"
#include <stdint.h>
#include <stddef.h>

// UART configuration structure
typedef struct {
    gpio_type* gpio_port;           // GPIO port (e.g., GPIOB)
    uint16_t gpio_pin;              // GPIO pin (e.g., GPIO_PINS_6)
    uint8_t gpio_source;            // GPIO source (e.g., GPIO_PINS_SOURCE6)
    uint8_t scfg_source;            // SCFG source (e.g., SCFG_PINS_SOURCE6)
    uint32_t exti_line;             // EXTI line (e.g., EXINT_LINE_6)
    IRQn_Type exti_irq;             // EXTI IRQ (e.g., EXINT15_4_IRQn)
    
    uint32_t baud_rate;             // Baud rate (e.g., 115200)
    uint8_t inverted;               // 1 for inverted protocol, 0 for normal
    uint8_t idle_state;             // Idle state: 0=LOW, 1=HIGH
    
    tmr_type* timer;                // Timer for both TX and RX (e.g., TMR17)
    IRQn_Type timer_irq;            // Timer IRQ
    uint32_t timer_clk;             // Timer clock enable
} sw_uart_config_t;

// UART states
typedef enum {
    SW_UART_RX_IDLE = 0,
    SW_UART_RX_START_BIT,
    SW_UART_RX_DATA_BITS,
    SW_UART_RX_STOP_BIT
} sw_uart_rx_state_t;

// Callback function types
typedef void (*sw_uart_rx_callback_t)(uint8_t data);
typedef void (*sw_uart_tx_complete_callback_t)(void);

// Public API functions
void sw_uart_init(const sw_uart_config_t* config);
void sw_uart_deinit(void);

// Pin configuration
void sw_uart_enable_rx(void);
void sw_uart_disable_rx(void);
void sw_uart_enable_tx(void);
void sw_uart_disable_tx(void);

// Data transmission
void sw_uart_send_byte(uint8_t data);
void sw_uart_send_frame(uint8_t* buffer, uint8_t length);
uint8_t sw_uart_tx_complete(void);

// Callback registration
void sw_uart_set_rx_callback(sw_uart_rx_callback_t callback);
void sw_uart_set_tx_complete_callback(sw_uart_tx_complete_callback_t callback);

// Interrupt handlers (to be called from MCU interrupt handlers)
void sw_uart_exti_handler(void);
void sw_uart_timer_handler(void);

#endif // SINGLEWIRE_SW_UART_H
