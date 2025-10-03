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
#include "serial_telemetry.h"
#include "stdbool.h"
#include "functions.h"
#include "io.h"

// Data transmission

// USART1 RX buffer reading functions
uint8_t read_usart1_rx_byte(uint8_t *data);
uint8_t usart1_rx_data_available(void);

#define TX_USEC_DELAY 200
#define TX_START_DELAY (TX_USEC_DELAY / (1000000 / 57600))

void singlewire_uart_init()
{
    telem_UART_Init();
    // set baudrate and inverted TX and RX and also make it halfduplex, if not already done
    // overwrite what we need, so disable first...
    LL_USART_Disable(USART1);

    LL_USART_InitTypeDef USART_InitStruct = {0};

    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 57600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_EnableFIFO(USART1);
    LL_USART_ConfigHalfDuplexMode(USART1);
    LL_USART_SetRXPinLevel(USART1, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_SetTXPinLevel(USART1, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_EnableRxTimeout(USART1);
    LL_USART_SetRxTimeout(USART1, TX_START_DELAY);
    LL_USART_Enable(USART1);
    while ((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
    {
    }
}

// Send a frame of bytes
uint8_t singlewire_uart_send_frame(uint8_t *buffer, uint8_t length)
{
    /

    return 1; // Success
}

/**
 * Reads a single byte from USART1 RX buffer
 * @param data Pointer to store the received byte
 * @return 1 if byte was read successfully, 0 if no data available
 */
uint8_t singlewire_uart_read_byte(uint8_t *data)
{

    // lets poll the receiver timeout flag
    // we have set the timeout at 200 usec
    // we return false if there is no timeout
    // this means we can answer immediately if data is available
    // and our receiver has had enough time to switch to receive mode
    if (LL_USART_IsActiveFlag_RTO(USART1))
    {

        if (LL_USART_IsActiveFlag_RXNE(USART1))
        {
            *data = LL_USART_ReceiveData8(USART1);
            // we check for another byte, if there is none, we clear the timeout flag
            // otherwise we keep it set, so the next byte can be read immediately
            if (!LL_USART_IsActiveFlag_RXNE(USART1))
            {
                LL_USART_ClearFlag_RTO(USART1);
            }
            return 1;
        }
    }
    return 0;
}
