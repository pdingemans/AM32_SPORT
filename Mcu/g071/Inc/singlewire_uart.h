/**
 * Single-wire UART implementation
 *
 */

#ifndef SINGLEWIRE_UART_H
#define SINGLEWIRE_UART_H


#include <stdint.h>
#include <stddef.h>
#include "common.h"


// Interrupt handlers (to be called from MCU interrupt handlers)
// discussion neede wether interrupt handling shouldn't be done in the _it file but
// in the uart itself. for now lets keep it like this, as all interrupt handling is
// in the _it file.
void sw_uart_exti_handler(void);
void sw_uart_dma_complete_handler(struct serial_telemetry_class* self);

// the following functions are necessay for every different MCU
void singlewire_uart_init(void);
/*
* following function sends a frame with a delay of 200 usec.
*/
uint8_t singlewire_uart_send_frame(uint8_t* buffer, uint8_t length);
uint8_t singlewire_uart_read_byte(uint8_t* data);



#endif // SINGLEWIRE_UART_H
