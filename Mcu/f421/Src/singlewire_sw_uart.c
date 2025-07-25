/**
 * Single-wire Software UART implementation
 * Generic bit-banging UART for inverted and non-inverted protocols
 * 
 * Features:
 * - AT32F421 GPIO enums for configuration (GPIO_MODE_*, GPIO_PULL_*)
 * - Direct register manipulation for optimal performance
 * - Pin number based configuration (vs bitmask)
 * - Half-duplex operation with single timer
 * - 64-byte TX FIFO queue for reliable transmission
 * - Inverted protocol support for FrSky Sport telemetry
 */

#include "singlewire_sw_uart.h"

// Internal function prototypes (not exposed in header)
static uint8_t sw_uart_fifo_put(uint8_t data);
static uint8_t sw_uart_fifo_get(uint8_t* data);
static void sw_uart_start_next_tx(void);
static uint8_t sw_uart_queue_byte(uint8_t data);
static uint8_t sw_uart_queue_frame(uint8_t* buffer, uint8_t length);

// UART frame bit definitions
#define SW_UART_START_BIT           (1 << 0)  /*!< Start bit position */
#define SW_UART_DATA_BITS_SHIFT     1         /*!< Data bits start position */
#define SW_UART_BYTE_MASK           0xFF      /*!< Byte mask for data */
#define SW_UART_FRAME_BITS          10        /*!< Total bits: 1 start + 8 data + 1 stop */



// Static configuration and state
static const sw_uart_config_t* sw_uart_config = NULL;
static volatile sw_uart_rx_state_t sw_uart_rx_state = SW_UART_RX_IDLE;
static volatile uint8_t sw_uart_tx_mode = 0;  // 0 = RX mode, 1 = TX mode

// RX state variables
static volatile uint8_t sw_uart_bit_counter = 0;
static volatile uint8_t sw_uart_rx_byte = 0;

// TX state variables
static volatile uint32_t sw_uart_tx_data = 0;
static volatile uint8_t sw_uart_tx_bit_count = 0;
static volatile uint8_t sw_uart_tx_active = 0;

// TX FIFO queue

// TX FIFO configuration
#define SW_UART_TX_FIFO_SIZE 64
static volatile uint8_t sw_uart_tx_fifo[SW_UART_TX_FIFO_SIZE];
static volatile uint8_t sw_uart_tx_fifo_head = 0;
static volatile uint8_t sw_uart_tx_fifo_tail = 0;
static volatile uint8_t sw_uart_tx_fifo_size = 0;

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
    
    // Initialize FIFO
    sw_uart_tx_fifo_head = 0;
    sw_uart_tx_fifo_tail = 0;
    sw_uart_tx_fifo_size = 0;
    
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
    
    // Reset FIFO
    sw_uart_tx_fifo_head = 0;
    sw_uart_tx_fifo_tail = 0;
    sw_uart_tx_fifo_size = 0;
    sw_uart_tx_active = 0;
    sw_uart_tx_mode = 0;
    
    sw_uart_config = NULL;
}

// Configure pin for reception
void sw_uart_enable_rx(void)
{
    if (!sw_uart_config) return;
    
    // Fast direct register manipulation for INPUT mode with pull-down
    uint8_t pin_pos = sw_uart_config->gpio_pin_num;
    
    // Atomic operation: Clear and set mode bits to GPIO_MODE_INPUT in one operation
    sw_uart_config->gpio_port->cfgr = (sw_uart_config->gpio_port->cfgr & ~(0x3 << (pin_pos * 2))) | (GPIO_MODE_INPUT << (pin_pos * 2));
    
    // Atomic operation: Clear and set pull bits to GPIO_PULL_DOWN in one operation
    sw_uart_config->gpio_port->pull = (sw_uart_config->gpio_port->pull & ~(0x3 << (pin_pos * 2))) | (GPIO_PULL_DOWN << (pin_pos * 2));
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
    
    // Fast direct register manipulation for OUTPUT mode
    uint8_t pin_pos = sw_uart_config->gpio_pin_num;
    uint16_t pin_mask = (1 << pin_pos);
    
    // Atomic operation: Clear and set mode bits to GPIO_MODE_OUTPUT in one operation
    sw_uart_config->gpio_port->cfgr = (sw_uart_config->gpio_port->cfgr & ~(0x3 << (pin_pos * 2))) | (GPIO_MODE_OUTPUT << (pin_pos * 2));
    
    // Atomic operation: Clear and set pull bits to GPIO_PULL_NONE in one operation
    sw_uart_config->gpio_port->pull = (sw_uart_config->gpio_port->pull & ~(0x3 << (pin_pos * 2))) | (GPIO_PULL_NONE << (pin_pos * 2));
    
    // Set output type to push-pull (0 in ODT register)
    sw_uart_config->gpio_port->odt &= ~pin_mask;
    
    // Inverted logic: set initial idle state to LOW
    sw_uart_config->gpio_port->clr = pin_mask;
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

// Send a single byte (legacy function - now uses FIFO)
void sw_uart_send_byte(uint8_t data)
{
    sw_uart_queue_byte(data);
}

// Send a frame of bytes (legacy function - now uses FIFO)
void sw_uart_send_frame(uint8_t* buffer, uint8_t length)
{
    sw_uart_queue_frame(buffer, length);
}

// Check if transmission is complete
uint8_t sw_uart_tx_complete(void)
{
    return !sw_uart_tx_active && (sw_uart_tx_fifo_size == 0);
}

// FIFO helper functions
static uint8_t sw_uart_fifo_put(uint8_t data)
{
    if (sw_uart_tx_fifo_size >= SW_UART_TX_FIFO_SIZE) {
        return 0; // FIFO full
    }
    
    sw_uart_tx_fifo[sw_uart_tx_fifo_head] = data;
    sw_uart_tx_fifo_head = (sw_uart_tx_fifo_head + 1) % SW_UART_TX_FIFO_SIZE;
    sw_uart_tx_fifo_size++;
    
    return 1; // Success
}

static uint8_t sw_uart_fifo_get(uint8_t* data)
{
    if (sw_uart_tx_fifo_size == 0) {
        return 0; // FIFO empty
    }
    
    *data = sw_uart_tx_fifo[sw_uart_tx_fifo_tail];
    sw_uart_tx_fifo_tail = (sw_uart_tx_fifo_tail + 1) % SW_UART_TX_FIFO_SIZE;
    sw_uart_tx_fifo_size--;
    
    return 1; // Success
}

// Start transmission of next byte from FIFO
static void sw_uart_start_next_tx(void)
{
    uint8_t data;
    
    if (!sw_uart_fifo_get(&data)) {
        // No more data to send, return to RX mode
        sw_uart_tx_mode = 0;
        sw_uart_enable_rx();
        return;
    }
    
    // Prepare TX data for inverted protocol (like SPORT)
    // Start bit = HIGH, Data inverted, Stop bit = LOW
    sw_uart_tx_data = 0;
    sw_uart_tx_data |= SW_UART_START_BIT;  // Start bit (HIGH)
    sw_uart_tx_data |= ((~data & SW_UART_BYTE_MASK) << SW_UART_DATA_BITS_SHIFT);  // Data bits inverted
    // Stop bit = LOW (bit 9 = 0)
    
    // Start transmission
    sw_uart_tx_bit_count = 0;
    sw_uart_tx_active = 1;
    sw_uart_tx_mode = 1;  // Switch to TX mode
    
    // Configure and start timer if not already running
    if (!tmr_flag_get(sw_uart_config->timer, TMR_OVF_FLAG)) {
        uint32_t timer_period = (120000000 / sw_uart_config->baud_rate) - 1;
        tmr_base_init(sw_uart_config->timer, timer_period, 0);
        tmr_cnt_dir_set(sw_uart_config->timer, TMR_COUNT_UP);
        
        tmr_interrupt_enable(sw_uart_config->timer, TMR_OVF_INT, TRUE);
        nvic_irq_enable(sw_uart_config->timer_irq, 2, 0);
        
        tmr_counter_enable(sw_uart_config->timer, TRUE);
    }
}

// Queue a single byte for transmission
static uint8_t sw_uart_queue_byte(uint8_t data)
{
    if (!sw_uart_config) return 0;
    
    // Try to add to FIFO
    if (!sw_uart_fifo_put(data)) {
        return 0; // FIFO full
    }
    
    // If not currently transmitting, start transmission
    if (!sw_uart_tx_active && sw_uart_tx_mode == 0) {
        // Configure pin for transmission
        sw_uart_enable_tx();
        
        // Disable EXTI during transmission to avoid conflicts
        nvic_irq_disable(sw_uart_config->exti_irq);
        
        // Start transmitting
        sw_uart_start_next_tx();
    }
    
    return 1; // Success
}

// Queue multiple bytes for transmission
static uint8_t sw_uart_queue_frame(uint8_t* buffer, uint8_t length)
{
    if (!sw_uart_config || !buffer) return 0;
    
    // Check if there's enough space in FIFO
    if (sw_uart_tx_fifo_size + length > SW_UART_TX_FIFO_SIZE) {
        return 0; // Not enough space
    }
    
    // Add all bytes to FIFO
    for (uint8_t i = 0; i < length; i++) {
        if (!sw_uart_fifo_put(buffer[i])) {
            return 0; // Should not happen due to space check above
        }
    }
    
    // If not currently transmitting, start transmission
    if (!sw_uart_tx_active && sw_uart_tx_mode == 0) {
        // Configure pin for transmission
        sw_uart_enable_tx();
        
        // Disable EXTI during transmission to avoid conflicts
        nvic_irq_disable(sw_uart_config->exti_irq);
        
        // Start transmitting
        sw_uart_start_next_tx();
    }
    
    return 1; // Success
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
    
    // Calculate pin mask from pin number
    uint16_t pin_mask = (1 << sw_uart_config->gpio_pin_num);
    uint8_t pin_state = (sw_uart_config->gpio_port->idt & pin_mask) ? SET : RESET;
    
    // Inverted protocol: start bit is LOW→HIGH transition
    if (sw_uart_rx_state == SW_UART_RX_IDLE && pin_state == SET) {
        sw_uart_rx_state = SW_UART_RX_START_BIT;
        sw_uart_tx_mode = 0;  // Ensure we're in RX mode
        
        // Start timer with half bit period to sample start bit in the middle
        uint32_t timer_period = ((120000000 / sw_uart_config->baud_rate) / 2) - 1;  // Half bit period
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
        if (sw_uart_tx_bit_count < SW_UART_FRAME_BITS) {  // 1 start + 8 data + 1 stop = 10 bits
            // Check current bit (LSB) and set pin state directly
            uint16_t pin_mask = (1 << sw_uart_config->gpio_pin_num);
            
            if (sw_uart_tx_data & 1) {
                sw_uart_config->gpio_port->scr = pin_mask;  // Set pin HIGH
            } else {
                sw_uart_config->gpio_port->clr = pin_mask;  // Set pin LOW
            }
            
            // Shift data left for next bit
            sw_uart_tx_data >>= 1;
            sw_uart_tx_bit_count++;
        } else {
            // Current byte transmission complete
            sw_uart_tx_active = 0;
            
            // Check if there are more bytes in FIFO to transmit
            if (sw_uart_tx_fifo_size > 0) {
                // Start next byte transmission
                sw_uart_start_next_tx();
            } else {
                // All bytes sent, disable transmission and return to RX
                tmr_counter_enable(sw_uart_config->timer, FALSE);
                tmr_interrupt_enable(sw_uart_config->timer, TMR_OVF_INT, FALSE);
                
                sw_uart_tx_mode = 0;  // Switch back to RX mode
                
                // Re-enable EXTI for reception
                nvic_irq_enable(sw_uart_config->exti_irq, 1, 0);
                
                // Return to RX mode
                sw_uart_enable_rx();
                
                // Call completion callback if registered
                if (sw_uart_tx_complete_callback) {
                    sw_uart_tx_complete_callback();
                }
            }
        }
    } else {
        // RX mode: handle reception
        uint16_t pin_mask = (1 << sw_uart_config->gpio_pin_num);
        uint8_t pin_state = (sw_uart_config->gpio_port->idt & pin_mask) ? SET : RESET;
        
        switch (sw_uart_rx_state) {
            case SW_UART_RX_START_BIT:
                // Sample middle of start bit (inverted: should be HIGH)
                if (pin_state == SET) {
                    sw_uart_bit_counter = 0;
                    sw_uart_rx_byte = 0;
                    sw_uart_rx_state = SW_UART_RX_DATA_BITS;
                    
                    // Reconfigure timer for full bit periods for data bits
                    uint32_t full_bit_period = (120000000 / sw_uart_config->baud_rate) - 1;
                    tmr_base_init(sw_uart_config->timer, full_bit_period, 0);
                } else {
                    sw_uart_rx_state = SW_UART_RX_IDLE;  // False start
                    sw_uart_disable_rx();
                }
                break;
                
            case SW_UART_RX_DATA_BITS:
                // Sample data bits (LSB first) - Inverted: LOW = 1, HIGH = 0
                if (pin_state == RESET) {
                    sw_uart_rx_byte |= (1 << sw_uart_bit_counter);
                }
                
                sw_uart_bit_counter++;
                if (sw_uart_bit_counter >= 8) {
                    sw_uart_rx_state = SW_UART_RX_STOP_BIT;
                }
                break;
                
            case SW_UART_RX_STOP_BIT:
                // Validate stop bit (inverted: should be LOW)
                if (pin_state == RESET) {
                    // Call RX callback if registered
                    if (sw_uart_rx_callback) {
                        sw_uart_rx_callback(sw_uart_rx_byte);
                    }
                }
                
                sw_uart_rx_state = SW_UART_RX_IDLE;
                sw_uart_disable_rx();
                break;
                
            default:
                sw_uart_rx_state = SW_UART_RX_IDLE;
                sw_uart_disable_rx();
                break;
        }
    }
}


