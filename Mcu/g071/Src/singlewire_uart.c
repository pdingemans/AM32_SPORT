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
    LL_USART_InitTypeDef USART_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    // we need pushpull for inverted and pull down for sport non-inverted
    // opendrain can only pull and not push
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN; // for inverted sport stuff
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // no interrupts for uart, all is polling basis
    NVIC_SetPriority(USART1_IRQn, 3);
    NVIC_EnableIRQ(USART1_IRQn);

    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_USART1_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3,
    LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);



   
    LL_USART_Disable(USART1);
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 57600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;  // default tx/rx
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
        // Enable DMA TX complete interrupt
        LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);

    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXPinLevel(USART1, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_SetTXPinLevel(USART1, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_EnableFIFO(USART1);
    LL_USART_ConfigHalfDuplexMode(USART1);
    LL_USART_SetRXPinLevel(USART1, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_SetTXPinLevel(USART1, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_EnableRxTimeout(USART1);
    LL_USART_SetRxTimeout(USART1, TX_START_DELAY);


    LL_USART_Enable(USART1);
    while ((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1)))) {
    }

    //LL_DMA_ConfigAddresses(
    //    DMA1, LL_DMA_CHANNEL_3, (uint32_t)aTxBuffer,
    //    LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
    //    LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        //LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
    //LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(aTxBuffer));

    /* (5) Enable DMA transfer complete/error interrupts  */
    //LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
   // LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);



    // // set baudrate and inverted TX and RX and also make it halfduplex, if not already done
    // // overwrite what we need, so disable first...
    // LL_USART_Disable(USART1);

    // LL_USART_InitTypeDef USART_InitStruct = {0};

    // USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    // USART_InitStruct.BaudRate = 57600;
    // USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    // USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    // USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    // USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    // USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    // LL_USART_Init(USART1, &USART_InitStruct);
    // LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    // LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    // LL_USART_EnableFIFO(USART1);
    // LL_USART_ConfigHalfDuplexMode(USART1);
    // LL_USART_SetRXPinLevel(USART1, LL_USART_RXPIN_LEVEL_INVERTED);
    // LL_USART_SetTXPinLevel(USART1, LL_USART_TXPIN_LEVEL_INVERTED);
    // LL_USART_EnableRxTimeout(USART1);
    // LL_USART_SetRxTimeout(USART1, TX_START_DELAY);
    // LL_USART_Enable(USART1);
    // while ((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
    // {
    // }
}
void sw_uart_dma_complete_handler(struct serial_telemetry_class* self)
{
    // this shouldnt be needed as according to the reference manual, the USART automatically switches back to RX
    // after the transmission is complete
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_RX);
}

// Send a frame of bytes
uint8_t singlewire_uart_send_frame(uint8_t *buffer, uint8_t length)
{
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_3, (uint32_t)buffer,
        LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3,length);

    // set data length and enable channel to start transfer
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX);

    LL_USART_EnableDMAReq_TX(USART1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

    
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
