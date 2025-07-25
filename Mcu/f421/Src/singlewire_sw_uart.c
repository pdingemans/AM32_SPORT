/**
 * Single-wire Software UART implementation
 * Generic bit-banging UART for inverted and non-inverted protocols
 */

#include "singlewire_sw_uart.h"

#ifdef ENABLE_FRSKY_SPORT_TELEMETRY

// Static configuration and state
static const sw_uart_config_t* sw_uart_config = NULL;
static volatile sw_uart_rx_state_t sw_uart_rx_state = SW_UART_RX_IDLE;
static volatile uint8_t sw_uart_tx_mode = 0;  // 0 = RX mode, 1 = TX mode

// RX state variables
static volatile uint8_t sw_uart_rx_buffer[32];
static volatile uint8_t sw_uart_rx_index = 0;
static volatile uint8_t sw_uart_bit_counter = 0;
static volatile uint8_t sw_uart_rx_byte = 0;

// TX state variables
static volatile uint32_t sw_uart_tx_data = 0;
static volatile uint8_t sw_uart_tx_bit_count = 0;
static volatile uint8_t sw_uart_tx_active = 0;

// Callbacks
static sw_uart_rx_callback_t sw_uart_rx_callback = NULL;
static sw_uart_tx_complete_callback_t sw_uart_tx_complete_callback = NULL;

// Initialize software UART with given configuration
void sw_uart_init(const sw_uart_config_t* config)
{
    if (!config) return;
    
    sw_uart_config = config;
    
    // Enable required clocks - generic GPIO clock enable is done by caller
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(config->timer_clk, TRUE);
    
    // Configure pin for reception
    sw_uart_enable_rx();
    
    // Configure EXTI for edge detection
    exint_init_type exint_init_struct;
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, config->scfg_source);
    
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_select = config->exti_line;
    exint_init_struct.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
    exint_init(&exint_init_struct);
    
    // Enable EXTI interrupt
    nvic_irq_enable(config->exti_irq, 1, 0);
    
    sw_uart_rx_state = SW_UART_RX_IDLE;
}

// Deinitialize software UART
void sw_uart_deinit(void)
{
    if (!sw_uart_config) return;
    
    // Disable interrupts
    nvic_irq_disable(sw_uart_config->exti_irq);
    nvic_irq_disable(sw_uart_config->timer_irq);
    
    // Stop timer
    tmr_counter_enable(sw_uart_config->timer, FALSE);
    
    sw_uart_config = NULL;
}

// Configure pin for reception
void sw_uart_enable_rx(void)
{
    if (!sw_uart_config) return;
    
    gpio_init_type gpio_init_struct;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = sw_uart_config->gpio_pin;
    
    // Set pull based on idle state
    if (sw_uart_config->idle_state == 0) {
        gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;  // Idle LOW
    } else {
        gpio_init_struct.gpio_pull = GPIO_PULL_UP;    // Idle HIGH
    }
    
    gpio_init(sw_uart_config->gpio_port, &gpio_init_struct);
}

// Disable reception
void sw_uart_disable_rx(void)
{
    if (!sw_uart_config) return;
    
    // Stop timer
    tmr_counter_enable(sw_uart_config->timer, FALSE);
    tmr_interrupt_enable(sw_uart_config->timer, TMR_OVF_INT, FALSE);
    
    sw_uart_rx_state = SW_UART_RX_IDLE;
}

// Configure pin for transmission
void sw_uart_enable_tx(void)
{
    if (!sw_uart_config) return;
    
    gpio_init_type gpio_init_struct;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = sw_uart_config->gpio_pin;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(sw_uart_config->gpio_port, &gpio_init_struct);
    
    // Set initial idle state
    if (sw_uart_config->idle_state == 0) {
        gpio_bits_reset(sw_uart_config->gpio_port, sw_uart_config->gpio_pin);
    } else {
        gpio_bits_set(sw_uart_config->gpio_port, sw_uart_config->gpio_pin);
    }
}

// Disable transmission
void sw_uart_disable_tx(void)
{
    if (!sw_uart_config) return;
    
    // Stop timer
    tmr_counter_enable(sw_uart_config->timer, FALSE);
    tmr_interrupt_enable(sw_uart_config->timer, TMR_OVF_INT, FALSE);
    
    sw_uart_tx_active = 0;
    sw_uart_tx_mode = 0;  // Switch back to RX mode
    
    // Return to RX mode
    sw_uart_enable_rx();
}

// Send a single byte
void sw_uart_send_byte(uint8_t data)
{
    if (!sw_uart_config) return;
    
    // Wait for any previous transmission to complete
    while (sw_uart_tx_active);
    
    // Configure pin for transmission
    sw_uart_enable_tx();
    
    // Prepare TX data based on protocol type
    sw_uart_tx_data = 0;
    
    if (sw_uart_config->inverted) {
        // Inverted protocol (like SPORT)
        // Start bit = HIGH, Data inverted, Stop bit = LOW
        sw_uart_tx_data |= (1 << 0);  // Start bit (HIGH)
        sw_uart_tx_data |= ((~data & 0xFF) << 1);  // Data bits inverted
        // Stop bit = LOW (bit 9 = 0)
    } else {
        // Normal UART protocol
        // Start bit = LOW, Data normal, Stop bit = HIGH
        // Start bit = LOW (bit 0 = 0)
        sw_uart_tx_data |= ((data & 0xFF) << 1);  // Data bits normal
        sw_uart_tx_data |= (1 << 9);  // Stop bit (HIGH)
    }
    
    // Start transmission
    sw_uart_tx_bit_count = 0;
    sw_uart_tx_active = 1;
    sw_uart_tx_mode = 1;  // Switch to TX mode
    
    // Configure and start timer
    uint32_t timer_period = (120000000 / sw_uart_config->baud_rate) - 1;
    tmr_base_init(sw_uart_config->timer, timer_period, 0);
    tmr_cnt_dir_set(sw_uart_config->timer, TMR_COUNT_UP);
    
    tmr_interrupt_enable(sw_uart_config->timer, TMR_OVF_INT, TRUE);
    nvic_irq_enable(sw_uart_config->timer_irq, 2, 0);
    
    tmr_counter_enable(sw_uart_config->timer, TRUE);
}

// Send a frame of bytes
void sw_uart_send_frame(uint8_t* buffer, uint8_t length)
{
    if (!sw_uart_config || !buffer) return;
    
    // Disable EXTI during transmission to avoid conflicts
    nvic_irq_disable(sw_uart_config->exti_irq);
    
    for (uint8_t i = 0; i < length; i++) {
        sw_uart_send_byte(buffer[i]);
        while (sw_uart_tx_active);  // Wait for completion
    }
    
    // Re-enable EXTI for reception
    nvic_irq_enable(sw_uart_config->exti_irq, 1, 0);
    
    // Return to RX mode
    sw_uart_enable_rx();
}

// Check if transmission is complete
uint8_t sw_uart_tx_complete(void)
{
    return !sw_uart_tx_active;
}

// Set RX callback
void sw_uart_set_rx_callback(sw_uart_rx_callback_t callback)
{
    sw_uart_rx_callback = callback;
}

// Set TX complete callback
void sw_uart_set_tx_complete_callback(sw_uart_tx_complete_callback_t callback)
{
    sw_uart_tx_complete_callback = callback;
}

// EXTI interrupt handler (call from MCU interrupt handler)
void sw_uart_exti_handler(void)
{
    if (!sw_uart_config) return;
    
    uint8_t pin_state = gpio_input_data_bit_read(sw_uart_config->gpio_port, sw_uart_config->gpio_pin);
    
    // Detect start bit edge based on protocol type
    uint8_t start_bit_detected = 0;
    
    if (sw_uart_config->inverted) {
        // Inverted protocol: start bit is LOW→HIGH transition
        if (sw_uart_rx_state == SW_UART_RX_IDLE && pin_state == SET) {
            start_bit_detected = 1;
        }
    } else {
        // Normal UART: start bit is HIGH→LOW transition
        if (sw_uart_rx_state == SW_UART_RX_IDLE && pin_state == RESET) {
            start_bit_detected = 1;
        }
    }
    
    if (start_bit_detected) {
        sw_uart_rx_state = SW_UART_RX_START_BIT;
        sw_uart_tx_mode = 0;  // Ensure we're in RX mode
        
        // Start bit sampling timer
        uint32_t timer_period = (120000000 / sw_uart_config->baud_rate) - 1;
        tmr_base_init(sw_uart_config->timer, timer_period, 0);
        tmr_cnt_dir_set(sw_uart_config->timer, TMR_COUNT_UP);
        
        tmr_interrupt_enable(sw_uart_config->timer, TMR_OVF_INT, TRUE);
        nvic_irq_enable(sw_uart_config->timer_irq, 2, 0);
        
        tmr_counter_enable(sw_uart_config->timer, TRUE);
    }
}

// Timer interrupt handler (handles both TX and RX)
void sw_uart_timer_handler(void)
{
    if (!sw_uart_config) return;
    
    if (sw_uart_tx_mode) {
        // TX mode: handle transmission
        if (sw_uart_tx_bit_count < 10) {  // 1 start + 8 data + 1 stop = 10 bits
            // Get current bit to transmit
            uint8_t bit = (sw_uart_tx_data >> sw_uart_tx_bit_count) & 1;
            
            // Set pin state
            if (bit) {
                gpio_bits_set(sw_uart_config->gpio_port, sw_uart_config->gpio_pin);
            } else {
                gpio_bits_reset(sw_uart_config->gpio_port, sw_uart_config->gpio_pin);
            }
            
            sw_uart_tx_bit_count++;
        } else {
            // Transmission complete
            sw_uart_disable_tx();
            
            // Call completion callback if registered
            if (sw_uart_tx_complete_callback) {
                sw_uart_tx_complete_callback();
            }
        }
    } else {
        // RX mode: handle reception
        uint8_t pin_state = gpio_input_data_bit_read(sw_uart_config->gpio_port, sw_uart_config->gpio_pin);
        
        switch (sw_uart_rx_state) {
            case SW_UART_RX_START_BIT:
                // Sample middle of start bit
                if ((sw_uart_config->inverted && pin_state == SET) || 
                    (!sw_uart_config->inverted && pin_state == RESET)) {
                    sw_uart_bit_counter = 0;
                    sw_uart_rx_byte = 0;
                    sw_uart_rx_state = SW_UART_RX_DATA_BITS;
                } else {
                    sw_uart_rx_state = SW_UART_RX_IDLE;  // False start
                    sw_uart_disable_rx();
                }
                break;
                
            case SW_UART_RX_DATA_BITS:
                // Sample data bits (LSB first)
                if (sw_uart_config->inverted) {
                    // Inverted: LOW = 1, HIGH = 0
                    if (pin_state == RESET) {
                        sw_uart_rx_byte |= (1 << sw_uart_bit_counter);
                    }
                } else {
                    // Normal: HIGH = 1, LOW = 0
                    if (pin_state == SET) {
                        sw_uart_rx_byte |= (1 << sw_uart_bit_counter);
                    }
                }
                
                sw_uart_bit_counter++;
                if (sw_uart_bit_counter >= 8) {
                    sw_uart_rx_state = SW_UART_RX_STOP_BIT;
                }
                break;
                
            case SW_UART_RX_STOP_BIT:
                // Validate stop bit and process received byte
                {
                    uint8_t valid_stop = 0;
                    if (sw_uart_config->inverted) {
                        valid_stop = (pin_state == RESET);  // Stop bit = LOW
                    } else {
                        valid_stop = (pin_state == SET);    // Stop bit = HIGH
                    }
                    
                    if (valid_stop) {
                        // Store received byte
                        sw_uart_rx_buffer[sw_uart_rx_index++] = sw_uart_rx_byte;
                        if (sw_uart_rx_index >= sizeof(sw_uart_rx_buffer)) {
                            sw_uart_rx_index = 0;
                        }
                        
                        // Call RX callback if registered
                        if (sw_uart_rx_callback) {
                            sw_uart_rx_callback(sw_uart_rx_byte);
                        }
                    }
                    
                    sw_uart_rx_state = SW_UART_RX_IDLE;
                    sw_uart_disable_rx();
                }
                break;
                
            default:
                sw_uart_rx_state = SW_UART_RX_IDLE;
                sw_uart_disable_rx();
                break;
        }
    }
}

#endif // ENABLE_FRSKY_SPORT_TELEMETRY
