#include "main.h"

#ifndef KISS_TELEMETRY_H_
#define KISS_TELEMETRY_H_

#include "common.h"

serial_telemetry_class* init_kiss_telemetry();
extern uint8_t aTxBuffer[49] __attribute__((aligned(4)));

void makeTelemPackage(uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm);
void makeInfoPacket(void);

#endif