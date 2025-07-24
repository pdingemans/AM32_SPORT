/*
  FrSky sensor base for AM32 ESC firmware
  Converted from C++ to C implementation
  (c) Pawelsky 20150725
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_H_
#define _FRSKY_SPORT_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportSingleData.h"

#define SENSOR_NO_DATA_ID 0x0000
#define BIT(x, index) (((x) >> index) & 0x01)
#define MAX_SINGLE_DATA_COUNT 10

// Sensor ID enumeration
typedef enum {
    FRSKY_SENSOR_ID1 = 0x00,   // 0000 0000
    FRSKY_SENSOR_ID2 = 0xA1,   // 1010 0001
    FRSKY_SENSOR_ID3 = 0x22,   // 0010 0010
    FRSKY_SENSOR_ID4 = 0x83,   // 1000 0011
    FRSKY_SENSOR_ID5 = 0xE4,   // 1110 0100
    FRSKY_SENSOR_ID6 = 0x45,
    FRSKY_SENSOR_ID7 = 0xC6,   // 1100 0110
    FRSKY_SENSOR_ID8 = 0x67,
    FRSKY_SENSOR_ID9 = 0x48,
    FRSKY_SENSOR_ID10 = 0xE9,
    FRSKY_SENSOR_ID11 = 0x6A,
    FRSKY_SENSOR_ID12 = 0xCB,
    FRSKY_SENSOR_ID13 = 0xAC,
    FRSKY_SENSOR_ID14 = 0x0D,
    FRSKY_SENSOR_ID15 = 0x8E,
    FRSKY_SENSOR_ID16 = 0x2F,  // 0010 1111
    FRSKY_SENSOR_ID17 = 0xD0,  // ESC1
    FRSKY_SENSOR_ID18 = 0x71,  // esc2
    FRSKY_SENSOR_ID19 = 0xF2,
    FRSKY_SENSOR_ID20 = 0x53,  // 0101 0011
    FRSKY_SENSOR_ID21 = 0x34,  // 0011 0100
    FRSKY_SENSOR_ID22 = 0x95,  // 1001 0101
    FRSKY_SENSOR_ID23 = 0x16,  // 0001 0110
    FRSKY_SENSOR_ID24 = 0xB7,
    FRSKY_SENSOR_ID25 = 0x98,
    FRSKY_SENSOR_ID26 = 0x39,
    FRSKY_SENSOR_ID27 = 0xBA,
    FRSKY_SENSOR_ID28 = 0x1B,
    FRSKY_SENSOR_ID_IGNORE = 0xFF
} FrSkySportSensorId;

// FrSkySportSensor ADT structure
typedef struct {
    FrSkySportSensorId sensor_id;
    uint8_t sensor_data_idx;
    FrskySportSingleData* single_datas[MAX_SINGLE_DATA_COUNT];
    uint8_t single_data_count;
} FrSkySportSensor;

// Function prototypes
FrSkySportSensor* frsky_sport_sensor_create(FrSkySportSensorId id);
void frsky_sport_sensor_destroy(FrSkySportSensor* self);
uint16_t frsky_sport_sensor_send(FrSkySportSensor* self, FrSkySportSingleWireSerial* serial, uint8_t id, uint32_t now);
uint16_t frsky_sport_sensor_decode_data(FrSkySportSensor* self, uint8_t id, uint16_t app_id, uint32_t data);
void frsky_sport_sensor_add_single_data(FrSkySportSensor* self, FrskySportSingleData* s_data);

// Utility function
uint8_t frsky_sport_calc_sensor_id(const uint8_t physical_id);

#endif // _FRSKY_SPORT_SENSOR_H_
