/*
  FrSky ESC sensor for AM32 ESC firmware
  Converted from C++ to C implementation
  (c) Pawelsky 20210108
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_ESC_H_
#define _FRSKY_SPORT_SENSOR_ESC_H_

#include <stdint.h>
#include <stdbool.h>
#include "FrSkySportSensor.h"
#include "FrSkySportSingleData.h"

#define ESC_DEFAULT_ID FRSKY_SENSOR_ID17
#define ESC_DATA_COUNT 4

#define ESC_POWER_DATA_ID 0x0B50
#define ESC_RPM_CONS_DATA_ID 0x0B60
#define ESC_TEMP_DATA_ID 0x0B70
#define ESC_SBEC_DATA_ID 0x0E50

#define ESC_POWER_DATA_PERIOD 300
#define ESC_RPM_CONS_DATA_PERIOD 300
#define ESC_TEMP_DATA_PERIOD 300
#define ESC_SBEC_DATA_PERIOD 300

// FrSkySportSensorEsc ADT structure
typedef struct {
    FrSkySportSensor base_sensor;
    
    // Single data instances
    FrskySportSingleData* power_data;
    FrskySportSingleData* rpm_data;
    FrskySportSingleData* temp_data;
    FrskySportSingleData* sbec_data;
    
    // Data values
    float voltage;
    float current;
    uint32_t rpm;
    uint16_t consumption;
    int32_t temperature;
    float sbec_voltage;
    float sbec_current;
} FrSkySportSensorEsc;

// Function prototypes
FrSkySportSensorEsc* frsky_sport_sensor_esc_create(FrSkySportSensorId id);
void frsky_sport_sensor_esc_destroy(FrSkySportSensorEsc* self);
void frsky_sport_sensor_esc_set_data(FrSkySportSensorEsc* self, float volt, float curr, uint32_t rpm, uint16_t cons, float temp, float sbec_volt, float sbec_curr);
uint16_t frsky_sport_sensor_esc_decode_data(FrSkySportSensorEsc* self, uint8_t id, uint16_t app_id, uint32_t data);

// Getter functions
float frsky_sport_sensor_esc_get_voltage(const FrSkySportSensorEsc* self);
float frsky_sport_sensor_esc_get_current(const FrSkySportSensorEsc* self);
uint32_t frsky_sport_sensor_esc_get_rpm(const FrSkySportSensorEsc* self);
uint16_t frsky_sport_sensor_esc_get_consumption(const FrSkySportSensorEsc* self);
int32_t frsky_sport_sensor_esc_get_temperature(const FrSkySportSensorEsc* self);
float frsky_sport_sensor_esc_get_sbec_voltage(const FrSkySportSensorEsc* self);
float frsky_sport_sensor_esc_get_sbec_current(const FrSkySportSensorEsc* self);

#endif // _FRSKY_SPORT_SENSOR_ESC_H_
