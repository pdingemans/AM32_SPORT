/*
 * serial_telemetry.h
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "main.h"

#ifndef SERIAL_TELEMETRY_H_
#define SERIAL_TELEMETRY_H_

typedef struct serial_telemetry_class {
    uint8_t id;
    uint8_t update_on_interval; // in ms, 0 means off, no auto updates
    void (*set_id)(struct serial_telemetry_class* self, uint8_t new_id);
    void (*handle_esc_telemetry)(struct serial_telemetry_class* self);
    void (*makeTelemPackage)(struct serial_telemetry_class* self, uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm);
 } serial_telemetry_class;

void telem_UART_Init(void);
void send_telem_DMA(uint8_t bytes);

#endif /* SERIAL_TELEMETRY_H_ */
