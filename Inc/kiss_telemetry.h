#ifndef KISS_TELEMETRY_H_
#define KISS_TELEMETRY_H_

#include "common.h"

serial_telemetry_class* init_kiss_telemetry();
extern uint8_t aTxBuffer[49] __attribute__((aligned(4)));
#endif