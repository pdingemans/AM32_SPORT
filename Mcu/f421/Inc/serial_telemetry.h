/*
 * serial_telemetry.h
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 * october 2025
 * changed by Peter to support additional Sport telemetry
 * usec OO approach to abstract telemetry types from main code
 * this makes main code cleaner and allows easier addition of new telemetry types
 * and switching between them
 */

#include "main.h"

#ifndef SERIAL_TELEMETRY_H_
#define SERIAL_TELEMETRY_H_



void telem_UART_Init(void);
void send_telem_DMA(uint8_t bytes);

#endif /* SERIAL_TELEMETRY_H_ */
