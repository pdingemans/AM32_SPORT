/*
  FrSky Sport Telemetry for AM32 ESC firmware
  Converted from C++ to C implementation
  (c) Pawelsky 20141120
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_TELEMETRY_H_
#define _FRSKY_SPORT_TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>
#include "FrSkySportSensor.h"
#include "FrSkySportSingleWireSerial.h"

#define FRSKY_TELEMETRY_MAX_SENSORS 10
#define FRSKY_TELEMETRY_START_FRAME 0x7E

// FrSkySportTelemetry ADT structure
typedef struct {
    FrSkySportSensor* sensors[FRSKY_TELEMETRY_MAX_SENSORS];
    FrSkySportSingleWireSerial serial;
    uint8_t sensor_count;
    uint8_t prev_data;
} FrSkySportTelemetry;

// Function prototypes
FrSkySportTelemetry* frsky_sport_telemetry_create(void);
void frsky_sport_telemetry_destroy(FrSkySportTelemetry* self);
void frsky_sport_telemetry_begin(FrSkySportTelemetry* self);
uint16_t frsky_sport_telemetry_send(FrSkySportTelemetry* self);
void frsky_sport_telemetry_add_sensor(FrSkySportTelemetry* self, FrSkySportSensor* sensor);

#endif // _FRSKY_SPORT_TELEMETRY_H_
