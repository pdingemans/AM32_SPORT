/**
 * Single-wire Software UART implementation with pure DMA operation
 * High-performance UART for inverted and non-inverted protocols
 *
 * Features:
 * - AT32F421 GPIO enums for configuration (GPIO_MODE_*, GPIO_PULL_*)
 * - Pure DMA timer-triggered GPIO sampling for RX (zero CPU overhead)
 * - Pure DMA timer-triggered GPIO set/clear for TX (zero CPU overhead)
 * - Pin number based configuration (vs bitmask)
 * - Half-duplex operation with single DMA channel (TX/RX optimized)
 * - Direct DMA buffer filling for immediate transmission
 * - High-speed DMA sampling buffer for RX with 8x oversampling
 * - Edge-triggered RX start with 1.5 bit delay for frame synchronization
 * - 2 stop bits for better receiver alignment and reliability
 * - Inverted protocol support for FrSky Sport telemetry
 * - Race condition protection using specific interrupt disabling (not global)
 */

#include "singlewire_uart.h"
#include "stdbool.h"
#include "functions.h"
#include "io.h"
#include "at32f421_it.h"
#include "ADC.h"

// Internal function prototypes (not exposed in header)
typedef struct
{
    gpio_type *gpio_port; // GPIO port (e.g., GPIOB)
    uint8_t gpio_pin_num; // GPIO pin number (e.g., 6 for pin 6)
    uint8_t scfg_source;  // SCFG source (e.g., SCFG_PINS_SOURCE6)
    uint32_t exti_line;   // EXTI line (e.g., EXINT_LINE_6)
    IRQn_Type exti_irq;   // EXTI IRQ (e.g., EXINT15_4_IRQn)

    uint32_t baud_rate; // Baud rate (e.g., 57600)

    tmr_type *timer; // Timer for both TX and RX (e.g., TMR3)

    uint32_t timer_clk; // Timer clock enable (e.g., CRM_TMR3_PERIPH_CLOCK)

    dma_channel_type *dma_channel; // DMA channel (e.g., DMA1_CHANNEL4)
    IRQn_Type dma_irq;             // DMA IRQ (e.g., DMA1_Channel5_4_IRQn)
    uint32_t dma_full_flag;        // DMA full flag (e.g., DMA1_FDT4_FLAG)
    uint32_t dma_half_flag;        // DMA half flag (e.g., DMA1_HDT4_FLAG)
} sw_uart_config_t;

// Callback function types
typedef void (*sw_uart_rx_callback_t)(void *context, uint8_t data);
typedef void (*sw_uart_tx_complete_callback_t)(void);

// Public API functions
void sw_uart_init(sw_uart_config_t *config);
void sw_uart_deinit(void);

// Data transmission
void sw_uart_send_byte(uint8_t data);
uint8_t singlewire_uart_send_frame(uint8_t *buffer, uint8_t length);

// Callback registration
void sw_uart_set_rx_callback(sw_uart_rx_callback_t callback, void *context);
void sw_uart_set_tx_complete_callback(sw_uart_tx_complete_callback_t callback);

// Interrupt handlers (to be called from MCU interrupt handlers)
void sw_uart_exti_handler(void);
void sw_uart_dma_complete_handler(void);

// static void sw_uart_process_dma_samples(void);
static void sw_uart_decode_uart_frame(uint32_t *samples, uint16_t sample_count, uint8_t *decoded_byte);
static void sw_uart_setup_edge_detection(void);

static void sw_uart_start_dma_sampling(void);

static void sw_uart_prepare_tx_dma_buffer(uint8_t *data, uint8_t length);
static void sw_uart_start_tx_dma(void);
static void sw_uart_stop_dma_sampling(void);

// UART frame bit definitions
#define SW_UART_START_BIT (1 << 0) /*!< Start bit position */
#define SW_UART_DATA_BITS_SHIFT 1  /*!< Data bits start position */
#define SW_UART_BYTE_MASK 0xFF     /*!< Byte mask for data */
#define SW_UART_FRAME_BITS 11      /*!< Total bits: 1 start + 8 data + 2 stop */
#define SW_UART_RX_SAMPLE_BITS 9   /*!< RX sampling bits: 8 data + 1 stop bit */

// DMA sampling configuration
#define SW_UART_SAMPLES_PER_BIT 1                         /*!< One sample per bit (no oversampling) */
#define SW_UART_RX_BUFFER_SIZE SW_UART_RX_SAMPLE_BITS * 2 /*!< RX DMA buffer size for 9 bits (8 data + 1 stop) */
#define SW_UART_MIN_FRAME_SAMPLES SW_UART_RX_SAMPLE_BITS  /*!< Min samples for RX frame (9 bits) */

// TX DMA configuration
#define SW_UART_TX_MAX_FRAME_SIZE 20                                /*!< Maximum frame size in bytes */
#define SW_UART_TX_DMA_BUFFER_SIZE (SW_UART_TX_MAX_FRAME_SIZE * 11) /*!< TX DMA buffer size (1 byte = 11 bits: 1 start + 8 data + 2 stop) */

#define TX_USEC_DELAY 200
#define TX_START_DELAY ((120 * TX_USEC_DELAY) - 1) // this will calculate timer reload value
// Calculate 0.5 bit period delay to reach middle of start bit
const uint32_t bit_period = 120000000 / 57600;
// AT32F421 DMA register structure for fast configuration
typedef struct
{
    uint32_t ctrl;  // Control register
    uint32_t dtcnt; // Data transfer count
    uint32_t paddr; // Peripheral address
    uint32_t maddr; // Memory address
} dma_fast_config_t;

// UART state enumeration
typedef enum
{
    IDLE = 0,
    SENDING,
    RECEIVING
} uart_state_t;
static volatile uart_state_t uart_state = IDLE;

// testing stuff
static volatile uint8_t indmahandler = 0;
// Static configuration and state
// lets initialize it to zero
// so we can test if baudrate == 0 we are not initialized and return from public functions
static sw_uart_config_t sw_uart_config = {0};

// DMA sampling variables
static volatile uint32_t sw_uart_rx_buffer[SW_UART_RX_BUFFER_SIZE] __attribute__((aligned(4))); // Buffer for RX (9 bits: 8 data + 1 stop)

// TX DMA variables
static volatile uint32_t sw_uart_tx_dma_buffer[SW_UART_TX_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

static volatile uint16_t sw_uart_tx_bits_count = 0;

// Callbacks

static sw_uart_rx_callback_t sw_uart_rx_callback = NULL;
static void *sw_uart_rx_context = NULL;
static sw_uart_tx_complete_callback_t sw_uart_tx_complete_callback = NULL;

// Pin configuration
static inline void sw_uart_enable_rx(void);
static inline void sw_uart_disable_rx(void);
static inline void sw_uart_enable_tx(void);

// Pre-configured DMA settings for RX and TX
// direct register manipulation for fast configuration and bring down interrupt overhead
// constant adress for GPIOB IDR register
#define GPIOB_IDT_ADDR (GPIOB_BASE + offsetof(gpio_type, idt))
#define GPIOB_SCR_ADDR (GPIOB_BASE + offsetof(gpio_type, scr))
// we do double buffering for RX
// so our buffer needs to be twice the amount of samples for one byte
// we need inteerupts on half and full transfer
static const dma_channel_type dma_rx_config __attribute__((aligned(4))) = {
    .ctrl = (DMA_PRIORITY_VERY_HIGH << 12) |        // Priority: High
            (DMA_MEMORY_DATA_WIDTH_WORD << 10) |    // Memory: 32-bit (to match SysTick)
            (DMA_PERIPHERAL_DATA_WIDTH_WORD << 8) | // Peripheral: 32-bit (SysTick->VAL)
            (1 << 7) |                              // Memory increment
            (0 << 6) |                              // Peripheral no increment
            (1 << 5) |                              // Circular mode enabled
            (0 << 4) |                              // Direction: Peripheral to Memory
            (1 << 3) |                              // Transfer error interrupt
            (1 << 2) |                              // Half transfer interrupt enabled
            (1 << 1) |                              // Transfer complete interrupt
            (0 << 0),                               // Channel disabled initially
    .dtcnt = SW_UART_RX_BUFFER_SIZE,                // 9 samples
    .paddr = GPIOB_IDT_ADDR,                        // GPIOB->idt (GPIOB_BASE + 0x10)
                                                    // .paddr = (uint32_t)&TMR17->cval,                    // TMR16->cval (current value register)
    .maddr = (uint32_t)sw_uart_rx_buffer            // Set compile time (RX buffer)
};

static const dma_fast_config_t dma_tx_config __attribute__((aligned(4))) = {
    .ctrl = (DMA_PRIORITY_VERY_HIGH << 12) |        // Priority: Very High
            (DMA_MEMORY_DATA_WIDTH_WORD << 10) |    // Memory: 32-bit
            (DMA_PERIPHERAL_DATA_WIDTH_WORD << 8) | // Peripheral: 32-bit
            (1 << 7) |                              // Memory increment
            (0 << 6) |                              // Peripheral no increment
            (0 << 5) |                              // Circular mode disabled
            (1 << 4) |                              // Direction: Memory to Peripheral
            (1 << 3) |                              // Transfer error interrupt
            (0 << 2) |                              // Half transfer interrupt disabled
            (1 << 1) |                              // Transfer complete interrupt
            (0 << 0),                               // Channel disabled initially
    .dtcnt = 0,                                     // Set at runtime (TX bit count)
    .paddr = GPIOB_SCR_ADDR,                        // Set compile time (GPIO SCR)
    .maddr = 0                                      // Set at runtime (TX buffer)
};

// FIFO buffer for RX data
#define SW_UART_FIFO_SIZE 64
typedef struct
{
    uint8_t buffer[SW_UART_FIFO_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} sw_uart_fifo_t;

static sw_uart_fifo_t sw_uart_rx_fifo = {0};

// FIFO buffer functions
static inline uint8_t sw_uart_fifo_write(uint8_t data);
static inline uint8_t sw_uart_fifo_read(uint8_t *data);

// Fast DMA configuration functions
static inline void sw_uart_configure_dma_rx(void);
static inline void sw_uart_configure_dma_tx(uint16_t bit_count);

// Utility function to pulse PB4 on and off for 1 microsecond each

// Pulse function implementation - switches PB4 on and off for 1 usec each
static inline void pulse(uint8_t cnt)
{
    return; // disable for now

    // Configure PB4 as output push-pull
    GPIOB->cfgr = (GPIOB->cfgr & ~(0x3 << (4 * 2))) | (GPIO_MODE_OUTPUT << (4 * 2));
    GPIOB->omode &= ~(1 << 4);
    for (uint8_t i = 0; i < cnt; i++)
    {
        // Set PB4 high

        // Delay 1 microsecond (120 cycles at 120MHz)
        for (volatile uint32_t i = 0; i < 10; i++)
        {
            __NOP();
        }

        // Set PB4 low

        // Delay 1 microsecond (120 cycles at 120MHz)
        for (volatile uint32_t i = 0; i < 10; i++)
        {
            __NOP();
        }
    }
}

// FIFO write function - returns 1 if successful, 0 if buffer full
static inline uint8_t sw_uart_fifo_write(uint8_t data)
{
    __disable_irq();
    if (sw_uart_rx_fifo.count >= SW_UART_FIFO_SIZE)
    {
        __enable_irq();
        return 0; // Buffer full
    }

    sw_uart_rx_fifo.buffer[sw_uart_rx_fifo.head] = data;
    sw_uart_rx_fifo.head = (sw_uart_rx_fifo.head + 1) % SW_UART_FIFO_SIZE;
    // we dont have to disable interrupts here, as this is atomic in the context of the DMA handler
    sw_uart_rx_fifo.count++;
    __enable_irq(); // Re-enable interrupts after writing
    return 1;       // Success
}

// FIFO read function - returns 1 if data read, 0 if buffer empty
static inline uint8_t sw_uart_fifo_read(uint8_t *data)
{
    __disable_irq();
    if (sw_uart_rx_fifo.count == 0)
    {
        __enable_irq();
        return 0; // Buffer empty
    }

    *data = sw_uart_rx_fifo.buffer[sw_uart_rx_fifo.tail];
    sw_uart_rx_fifo.tail = (sw_uart_rx_fifo.tail + 1) % SW_UART_FIFO_SIZE;

    sw_uart_rx_fifo.count--;
    __enable_irq(); // Re-enable interrupts after reading

    return 1; // Success
}

// Fast DMA configuration for RX - single register block copy
static inline void sw_uart_configure_dma_rx(void)
{
    dma_channel_type *dma_ch = sw_uart_config.dma_channel;
    *dma_ch = dma_rx_config; // Copy template
}

// Fast DMA configuration for TX - single register block copy
static inline void sw_uart_configure_dma_tx(uint16_t bit_count)
{
    dma_channel_type *dma_ch = sw_uart_config.dma_channel;
    dma_fast_config_t tx_config = dma_tx_config; // Copy template

    // Set runtime values
    tx_config.dtcnt = bit_count;
    tx_config.paddr = (uint32_t)&sw_uart_config.gpio_port->scr;
    tx_config.maddr = (uint32_t)sw_uart_tx_dma_buffer;

    // Disable channel first
    dma_ch->ctrl &= ~0x1; // Clear enable bit

    // Fast block copy to DMA registers (4 registers in one go)
    dma_ch->ctrl = tx_config.ctrl;
    dma_ch->dtcnt = tx_config.dtcnt;
    dma_ch->paddr = tx_config.paddr;
    dma_ch->maddr = tx_config.maddr;
}

// Initialize software UART with given configuration
void sw_uart_init(sw_uart_config_t *config)
{
    // if (!config) return;

    // Initialize DMA state

    // Initialize TX DMA state

    sw_uart_tx_bits_count = 0;

    sw_uart_config = *config; // Copy configuration first!

    // Enable required clocks
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE); // Enable GPIOB clock for PB6
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(config->timer_clk, TRUE);

    // Initialize DMA sampling system

    // Initialize TX DMA system

    // Configure pin for reception (start in RX mode with DMA)
    tmr_cnt_dir_set(sw_uart_config.timer, TMR_COUNT_UP);
    sw_uart_config.timer->ctrl1_bit.prben = TRUE; // enable buffer so next overflow loads the next bit time

    // Configure timer for bit-rate sampling (1 sample per bit, no oversampling)
    uart_state = IDLE;
    sw_uart_enable_rx();

    ////qqqq
    // sw_uart_config.gpio_port->cfgr = (sw_uart_config.gpio_port->cfgr & ~(0x3 << (4 * 2))) | (GPIO_MODE_OUTPUT << (4 * 2));

    // Configure output as push-pull
    //    uint16_t pin_mask = (1 << 4);
    // sw_uart_config.gpio_port->omode &= ~pin_mask;

    return;
}
void singlewire_uart_init()
{
    // Software UART configuration for Sport telemetry
    // PB6, 57600 baud, inverted logic, TMR15 + DMA6
    sw_uart_config_t uart_config;

    uart_config.gpio_port = GPIOB;
    uart_config.gpio_pin_num = 6; // Pin number for PB6
    uart_config.baud_rate = 57600;

    // EXTI configuration for PB6 which is the Sport telemetry input pin
    uart_config.exti_line = EXINT_LINE_6;
    uart_config.exti_irq = EXINT15_4_IRQn;
    uart_config.scfg_source = SCFG_PINS_SOURCE6;

    // Timer configuration (using TMR15 for both TX and RX)
    uart_config.timer = TMR15;
    uart_config.timer_clk = CRM_TMR15_PERIPH_CLOCK;

    // DMA configuration (using DMA1_CHANNEL5)
    uart_config.dma_channel = DMA1_CHANNEL5;
    uart_config.dma_irq = DMA1_Channel5_4_IRQn; // Initialize software UART
    uart_config.dma_full_flag = DMA1_FDT5_FLAG;
    uart_config.dma_half_flag = DMA1_HDT5_FLAG;
    sw_uart_init(&uart_config);
}
// Deinitialize software UART
void sw_uart_deinit(void)
{
    // Disable interrupts
    nvic_irq_disable(sw_uart_config.exti_irq);
    nvic_irq_disable(sw_uart_config.dma_irq);

    // Stop timer
    tmr_counter_enable(sw_uart_config.timer, FALSE);

    // Disable DMA channel
    dma_channel_enable(sw_uart_config.dma_channel, FALSE);

    // Reset DMA states
    uart_state = IDLE; // Reset UART state
}

// Configure pin for reception with DMA sampling
static inline void sw_uart_enable_rx(void)
{

    // Fast direct register manipulation for INPUT mode with pull-down
    uint8_t pin_pos = sw_uart_config.gpio_pin_num;

    // Atomic operation: Clear and set mode bits to GPIO_MODE_INPUT in one operation
    sw_uart_config.gpio_port->cfgr = (sw_uart_config.gpio_port->cfgr & ~(0x3 << (pin_pos * 2))) | (GPIO_MODE_INPUT << (pin_pos * 2));
    // Atomic operation: Clear and set pull bits to GPIO_PULL_DOWN in one operation
    sw_uart_config.gpio_port->pull = (sw_uart_config.gpio_port->pull & ~(0x3 << (sw_uart_config.gpio_pin_num * 2))) | (GPIO_PULL_DOWN << (sw_uart_config.gpio_pin_num * 2));

    // Configure EXTI for rising edge detection to trigger delayed DMA start
    sw_uart_setup_edge_detection();
    sw_uart_start_dma_sampling(); // we can do this here as there will only be DMA request after the first overflow
}

// Disable reception
static inline void sw_uart_disable_rx(void)
{
    // Disable EXTI interrupt
    // nvic_irq_disable(sw_uart_config.exti_irq);
    atomic_exti_bit_modify(sw_uart_config.gpio_pin_num, 0);
    // Stop DMA sampling
    sw_uart_stop_dma_sampling();
}

// Set RX callback
void sw_uart_set_rx_callback(sw_uart_rx_callback_t callback, void *context)
{
    sw_uart_rx_callback = callback;
    sw_uart_rx_context = context; // Store context for callback
}

// Set TX complete callback
// llets try static defines to avoid calculation overhead

void sw_uart_set_tx_complete_callback(sw_uart_tx_complete_callback_t callback)
{
    sw_uart_tx_complete_callback = callback;
}

// EXTI interrupt handler (call from MCU interrupt handler)
void sw_uart_exti_handler(void)
{
    if (sw_uart_config.baud_rate == 0)
    {
        return; // Not initialized
    }
    if (indmahandler)
    {
        __NOP();
    } // we are already in the DMA handler, ignore any further interrupts

    // Check if line is actually high (start bit for inverted protocol)
    uint16_t pin_mask = (1 << sw_uart_config.gpio_pin_num);
    if (!(sw_uart_config.gpio_port->idt & pin_mask))
    {
        // Line is low, this should not happen on rising edge - ignore interrupt
        exint_flag_clear(sw_uart_config.exti_line);
        return;
    }
    // lets not do ADC when we are receiving, it interferes heavily with timer DMA requests
    disable_ADC();

    tmr_counter_enable(sw_uart_config.timer, TRUE); // start and then everything will go as it has been set at the start
    exint_flag_clear(sw_uart_config.exti_line);

    // nvic_irq_disable(sw_uart_config.exti_irq); // we cant do this for other interrupts on the same handler
    atomic_exti_bit_modify(sw_uart_config.gpio_pin_num, 0);
    // EXINT->inten &= ~sw_uart_config.exti_line;   // disable interrupt
    //  we're busy now with DMA stuff.

    uart_state = RECEIVING; // Set state to receiving
}

// Start DMA sampling triggered by timer
static inline void sw_uart_start_dma_sampling(void)
{

    // Fast DMA configuration for RX
    sw_uart_configure_dma_rx();
    // Enable Timer DMA request on overflow
    tmr_dma_request_enable(sw_uart_config.timer, TMR_OVERFLOW_DMA_REQUEST, TRUE);

    // Enable DMA channel and interrupts
    dma_interrupt_enable(sw_uart_config.dma_channel, DMA_FDT_INT, TRUE);
    nvic_irq_enable(sw_uart_config.dma_irq, 4, 0);
    dma_channel_enable(sw_uart_config.dma_channel, TRUE);

    // Start timer
    // tmr_counter_enable(sw_uart_config.timer, TRUE);
}

// Stop DMA sampling
inline static void sw_uart_stop_dma_sampling(void)
{
    // Stop timer
    tmr_counter_enable(sw_uart_config.timer, FALSE);
    tmr_dma_request_enable(sw_uart_config.timer, TMR_OVERFLOW_DMA_REQUEST, FALSE);

    // Disable DMA channel
    dma_channel_enable(sw_uart_config.dma_channel, FALSE);
}

// Decode UART frame from DMA samples (9 bits: 8 data + 1 stop)
inline static void sw_uart_decode_uart_frame(uint32_t *samples, uint16_t sample_count, uint8_t *decoded_byte)
{

    uint32_t pin_mask = (1 << sw_uart_config.gpio_pin_num);
    *decoded_byte = 0;
    // Samples are taken at bit centers: [bit0][bit1][bit2][bit3][bit4][bit5][bit6][bit7][stop]
    // Samples 0-7 = data bits (LSB first, inverted)
    // Sample 8 = stop bit (should be HIGH/0 for inverted protocol)

    // Verify stop bit is HIGH (0 for inverted protocol)
    uint8_t stop_bit = (samples[8] & pin_mask) ? 1 : 0;
    if (stop_bit != 0)
    {
        return; // Invalid stop bit for inverted protocol
    }

    // Decode 8 data bits (LSB first, inverted logic)
    for (uint8_t bit = 0; bit < 8; bit++)
    {
        // these instructions make complete call 563 ns
        ///  uint8_t data_bit = (samples[bit] & pin_mask) ? 0 : 1; // Inverted logic: HIGH=0, LOW=1
        //  if (data_bit)
        //  {
        //      decoded_byte |= (1 << bit); // Set bit in LSB-first order
        //  }

        // these instructions make complete call 500 ns , 437 ns for all 1's, 500 ns for all 0's
        // Use inverted logic: HIGH=0, LOW=1
        // This is more efficient than using a separate variable for data_bit
        // Directly set bit in decoded byte based on inverted logic
        if (!(samples[bit] & pin_mask))
        {
            *decoded_byte |= (1 << bit); // Set bit in LSB-first order
        }
    }
}

// DMA completion handler (call from DMA IRQ handler) - handles both RX and TX
inline void sw_uart_dma_complete_handler(void)
{
    if (sw_uart_config.baud_rate == 0)
    {
        return; // Not initialized so we skip the whole thing
    }
    indmahandler = 1;

    if (dma_flag_get(sw_uart_config.dma_full_flag) == SET || dma_flag_get(sw_uart_config.dma_half_flag) == SET)
    {
        // lets stop the timer as soon as possible so we dont get any more DMA requests
        // but only if its our DMA channel that triggered this
        tmr_counter_enable(sw_uart_config.timer, FALSE);
        // re-enable ADC DMA after TX/RX completes
        enable_ADC();

        // Check if we're in TX or RX mode
        if (uart_state == SENDING)
        {
            // lets clear all flags as send only uses full transfer so half is silly.
            dma_flag_clear(sw_uart_config.dma_full_flag | sw_uart_config.dma_half_flag);
            // TX completion - inline handler
            // Stop timer and DMA
            tmr_counter_enable(sw_uart_config.timer, FALSE);
            tmr_dma_request_enable(sw_uart_config.timer, TMR_OVERFLOW_DMA_REQUEST, FALSE);
            dma_channel_enable(sw_uart_config.dma_channel, FALSE);

            sw_uart_tx_bits_count = 0;

            // Set line to idle state (LOW for inverted protocol)
            uint16_t pin_mask = (1 << sw_uart_config.gpio_pin_num);
            sw_uart_config.gpio_port->clr = pin_mask;

            sw_uart_enable_rx();
            uart_state = IDLE; // Reset state to IDLE

            // Call completion callback if registered
            if (sw_uart_tx_complete_callback)
            {
                sw_uart_tx_complete_callback();
            }
        }
        else
        {
            bool is_full = dma_flag_get(sw_uart_config.dma_full_flag) == SET;

            dma_flag_clear(sw_uart_config.dma_full_flag | sw_uart_config.dma_half_flag);
            uart_state = IDLE; // Reset state to IDLE

            sw_uart_setup_edge_detection();
            // we start a new reception asap, as we have double buffering we can do this
            // without losing data
            // we enable the detection here and we allow the edge interrupt to interfere with the decoding

            uint8_t data = 0;
            // RX completion
            // we always have 9 samples..
            // Decode UART frame from RX samples (9 bits: 8 data + 1 stop)
            sw_uart_decode_uart_frame((uint32_t *)sw_uart_rx_buffer + is_full * SW_UART_MIN_FRAME_SAMPLES, SW_UART_MIN_FRAME_SAMPLES, &data);

            // Call RX callback with decoded byte
            if (sw_uart_rx_callback)
            {
                sw_uart_rx_callback(sw_uart_rx_context, data);
            }
            else
            {
                // No callback registered, store in FIFO
                sw_uart_fifo_write(data);
            }
            // Process samples in main loop (don't do heavy processing in IRQ)
        }
    }
    indmahandler = 0;
}

// Setup EXTI for rising edge detection to trigger delayed DMA start
inline static void sw_uart_setup_edge_detection(void)
{

    // Configure SCFG for EXTI line mapping
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, sw_uart_config.scfg_source);
    // Configure EXTI for rising edge detection - use direct register access for AT32F421
    atomic_polcfg1_bit_modify(sw_uart_config.gpio_pin_num, 1); // rising edge

    // Disable DMA channel
    tmr_dma_request_enable(sw_uart_config.timer, TMR_OVERFLOW_DMA_REQUEST, FALSE);
    // lets setup the timer overhere and start it as soon as the interrupt is triggered
    // this way we can get a high baudrate
    tmr_counter_enable(sw_uart_config.timer, FALSE);
    tmr_base_init(sw_uart_config.timer, ((bit_period * 3) / 2 - 1), 0); // 1.5  bit time will give first sample
    tmr_flag_clear(sw_uart_config.timer, TMR_OVF_FLAG);                 // Clear any pending update flags
    // after we get the first overflow the reload register will be loaded with 1 bit time, this saves interrupt overhad
    sw_uart_config.timer->pr = bit_period - 1;
    // enable DMA again
    tmr_dma_request_enable(sw_uart_config.timer, TMR_OVERFLOW_DMA_REQUEST, TRUE);

    // Clear any pending interrupt flags before starting
    exint_flag_clear(sw_uart_config.exti_line); // Clear pending interrupt
    // EXINT->inten |= sw_uart_config.exti_line;   // Enable interrupt
    atomic_exti_bit_modify(sw_uart_config.gpio_pin_num, 1);
    // Enable EXTI interrupt
    // nvic_irq_enable(sw_uart_config.exti_irq, 1, 0);
}

// Prepare TX DMA buffer with GPIO set/clear commands for UART data
static inline void sw_uart_prepare_tx_dma_buffer(uint8_t *data, uint8_t length)
{

    if (!data || !length)
        return;

    uint16_t buffer_index = 0;
    uint16_t pin_mask = (1 << sw_uart_config.gpio_pin_num);
    uint32_t set_command = pin_mask;       // Set bit (HIGH)
    uint32_t clr_command = pin_mask << 16; // Clear bit (LOW) - upper 16 bits of SCR
    // as we have a half duplex bus and its for sport telemetry the receiving side
    // needs some time to switch from TX to RX
    // lets give it 400 usec as thats what frsky does

    // Process each byte
    for (uint8_t byte_idx = 0; byte_idx < length && buffer_index < SW_UART_TX_DMA_BUFFER_SIZE - 11; byte_idx++)
    {
        uint8_t byte_data = data[byte_idx];

        // Prepare frame for inverted protocol (SPORT):
        // Start bit = HIGH, Data inverted, Stop bits = LOW

        // 1. Start bit (HIGH for inverted protocol)
        sw_uart_tx_dma_buffer[buffer_index++] = set_command;
        // this whole function with a for loop implementation takes 12.9 usec when sending 20 bytes

        // 2. Data bits (LSB first, inverted)
        // do not try to optimize this yourself, you will fail miserably :) compiler knows better....
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (byte_data & (1 << bit))
            {
                // Data bit = 1 → Output LOW (inverted)
                sw_uart_tx_dma_buffer[buffer_index++] = clr_command;
            }
            else
            {
                // Data bit = 0 → Output HIGH (inverted)
                sw_uart_tx_dma_buffer[buffer_index++] = set_command;
            }
        }
        /*
        // 2. Data bits (LSB first, inverted) - unrolled for speed :()
        // this implementaion with 20 byte takes: 14.9 usec...
        sw_uart_tx_dma_buffer[buffer_index++] = (byte_data & (1 << 0)) ? clr_command : set_command; // Bit 0
        sw_uart_tx_dma_buffer[buffer_index++] = (byte_data & (1 << 1)) ? clr_command : set_command; // Bit 1
        sw_uart_tx_dma_buffer[buffer_index++] = (byte_data & (1 << 2)) ? clr_command : set_command; // Bit 2
        sw_uart_tx_dma_buffer[buffer_index++] = (byte_data & (1 << 3)) ? clr_command : set_command; // Bit 3
        sw_uart_tx_dma_buffer[buffer_index++] = (byte_data & (1 << 4)) ? clr_command : set_command; // Bit 4
        sw_uart_tx_dma_buffer[buffer_index++] = (byte_data & (1 << 5)) ? clr_command : set_command; // Bit 5
        sw_uart_tx_dma_buffer[buffer_index++] = (byte_data & (1 << 6)) ? clr_command : set_command; // Bit 6
        sw_uart_tx_dma_buffer[buffer_index++] = (byte_data & (1 << 7)) ? clr_command : set_command; // Bit 7
        */

        // 3. Stop bits (2 bits, both LOW for inverted protocol)
        sw_uart_tx_dma_buffer[buffer_index++] = clr_command; // First stop bit
        sw_uart_tx_dma_buffer[buffer_index++] = clr_command; // Second stop bit
    }

    sw_uart_tx_bits_count = buffer_index;
    // pulse_pb6_test(1);
    // delayMicros(200); // Allow time for logic analyzer to capture the pulse
}

// Start TX DMA transmission
inline static void sw_uart_start_tx_dma(void)
{
    // ensure ADC DMA won't hog the bus during the critical TX window
    // ADC DMA channel on this project is DMA1_CHANNEL1 — adjust if different
    disable_ADC();

    // we need to disable DMA as configuring timer can lead to a DMA request we dont want
    dma_channel_enable(sw_uart_config.dma_channel, FALSE);

    // Fast DMA configuration for TX
    sw_uart_configure_dma_tx(sw_uart_tx_bits_count);

    // as we are starting a transmission we assume we switched from RX to TX
    // to give the receive the time to switch from TX to RX we add a delay of 400 usec
    // this is what frsky does in their implementation

    tmr_counter_enable(sw_uart_config.timer, FALSE);
    tmr_base_init(sw_uart_config.timer, TX_START_DELAY, 0); // TX will only  start after this delay
    tmr_flag_clear(sw_uart_config.timer, TMR_OVF_FLAG);     // Clear any pending update flags
    // after we get the first overflow the reload register will be loaded with 1 bit time, this saves interrupt overhad
    sw_uart_config.timer->pr = bit_period - 1;
    // uint32_t timer_period = (120000000 / sw_uart_config.baud_rate) - 1;
    // tmr_base_init(sw_uart_config.timer, timer_period, 0);
    tmr_cnt_dir_set(sw_uart_config.timer, TMR_COUNT_UP);

    // Enable Timer DMA request on overflow
    tmr_dma_request_enable(sw_uart_config.timer, TMR_OVERFLOW_DMA_REQUEST, TRUE);
    dma_flag_clear(sw_uart_config.dma_full_flag); // lets clear the flag, just to be very certain
    // Enable DMA channel and interrupts
    dma_interrupt_enable(sw_uart_config.dma_channel, DMA_FDT_INT, TRUE);
    nvic_irq_enable(sw_uart_config.dma_irq, 4, 0);
    dma_channel_enable(sw_uart_config.dma_channel, TRUE);

    // Start timer
    tmr_counter_enable(sw_uart_config.timer, TRUE);
}

// Configure pin for transmission
void sw_uart_enable_tx(void)
{

    // Fast direct register manipulation for OUTPUT mode
    uint8_t pin_pos = sw_uart_config.gpio_pin_num;
    uint16_t pin_mask = (1 << pin_pos);

    // Atomic operation: Clear and set mode bits to GPIO_MODE_OUTPUT in one operation
    sw_uart_config.gpio_port->cfgr = (sw_uart_config.gpio_port->cfgr & ~(0x3 << (pin_pos * 2))) | (GPIO_MODE_OUTPUT << (pin_pos * 2));

    // Atomic operation: Clear and set pull bits to GPIO_PULL_NONE in one operation
    sw_uart_config.gpio_port->pull = (sw_uart_config.gpio_port->pull & ~(0x3 << (pin_pos * 2))) | (GPIO_PULL_NONE << (pin_pos * 2));

    // Set output type to push-pull (0 in omode register)
    sw_uart_config.gpio_port->omode &= ~pin_mask;

    // Inverted logic: set initial idle state to LOW
    sw_uart_config.gpio_port->clr = pin_mask;
}

// Send a single byte
void sw_uart_send_byte(uint8_t data)
{
    singlewire_uart_send_frame(&data, 1);
}

// Send a frame of bytes
uint8_t singlewire_uart_send_frame(uint8_t *buffer, uint8_t length)
{
    if (!buffer || length == 0 || length > SW_UART_TX_MAX_FRAME_SIZE)
        return 0;

    // Atomic check and set of state
    __disable_irq();
    if (uart_state != IDLE)
    {
        __enable_irq();
        return 0; // Busy - cannot send
    }
    uart_state = SENDING; // Set state to sending

    // Stop DMA sampling during transmission
    sw_uart_disable_rx();

    // Configure pin for transmission
    sw_uart_enable_tx();
    __enable_irq();
    // Prepare DMA buffer with frame data
    sw_uart_prepare_tx_dma_buffer(buffer, length);

    // Start DMA transmission
    sw_uart_start_tx_dma();
    return 1;
}

// Read a byte from RX FIFO buffer
// Returns 1 if data was read, 0 if buffer is empty
uint8_t singlewire_uart_read_byte(uint8_t *data)
{
    return sw_uart_fifo_read(data);
}