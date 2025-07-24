/*
  FrSky single wire serial for AM32 ESC firmware
  Converted from C++ to C implementation
  (c) Pawelsky 20210108
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SINGLE_WIRE_SERIAL_H_
#define _FRSKY_SPORT_SINGLE_WIRE_SERIAL_H_

#include <stdint.h>
#include "targets.h"

#define BAUDRATE 57600

#define FRSKY_TELEMETRY_START_FRAME 0x7E
#define FRSKY_SENSOR_DATA_FRAME 0x10
#define FRSKY_SENSOR_EMPTY_FRAME 0x00
#define FRSKY_STUFFING 0x7D

#define EXTINV_FLAG 0x80

// Serial mode enumeration
typedef enum {
    FRSKY_SERIAL_MODE_RX = 0,
    FRSKY_SERIAL_MODE_TX = 1
} FrSkySportSerialMode;

// FrSkySportSingleWireSerial ADT structure
typedef struct {
    uint16_t crc;
} FrSkySportSingleWireSerial;

// Function prototypes
FrSkySportSingleWireSerial* frsky_sport_single_wire_serial_create(void);
void frsky_sport_single_wire_serial_destroy(FrSkySportSingleWireSerial* self);
void frsky_sport_single_wire_serial_begin(FrSkySportSingleWireSerial* self);
void frsky_sport_single_wire_serial_send_header(FrSkySportSingleWireSerial* self, uint8_t id);
void frsky_sport_single_wire_serial_send_data(FrSkySportSingleWireSerial* self, uint16_t data_type_id, uint32_t id);
void frsky_sport_single_wire_serial_send_empty(FrSkySportSingleWireSerial* self, uint16_t data_type_id);
uint8_t frsky_sport_single_wire_serial_available(FrSkySportSingleWireSerial* self);
uint8_t frsky_sport_single_wire_serial_read(FrSkySportSingleWireSerial* self);

#endif // _FRSKY_SPORT_SINGLE_WIRE_SERIAL_H_
