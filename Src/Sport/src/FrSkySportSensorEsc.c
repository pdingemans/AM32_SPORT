/*
  FrSky ESC sensor for AM32 ESC firmware
  Converted from C++ to C implementation
  Based on work of Pawelsky version : 2021050
*/

#include "../FrSkySportSensorEsc.h"
#include <stdlib.h>
#include <math.h>

FrSkySportSensorEsc* frsky_sport_sensor_esc_create(FrSkySportSensorId id)
{
    FrSkySportSensorEsc* self = malloc(sizeof(FrSkySportSensorEsc));
    if (self == NULL) {
        return NULL;
    }
    
    // Initialize base sensor
    self->base_sensor.sensor_id = id;
    self->base_sensor.sensor_data_idx = 0;
    self->base_sensor.single_data_count = 0;
    for (int i = 0; i < MAX_SINGLE_DATA_COUNT; i++) {
        self->base_sensor.single_datas[i] = NULL;
    }
    
    // Initialize data instances
    self->power_data = frsky_sport_single_data_create(ESC_POWER_DATA_ID, ESC_POWER_DATA_PERIOD);
    self->rpm_data = frsky_sport_single_data_create(ESC_RPM_CONS_DATA_ID, ESC_RPM_CONS_DATA_PERIOD);
    self->temp_data = NULL; // Not used in this implementation
    self->sbec_data = frsky_sport_single_data_create(ESC_SBEC_DATA_ID, ESC_SBEC_DATA_PERIOD);
    
    // Add single data instances to base sensor
    if (self->power_data != NULL) {
        frsky_sport_sensor_add_single_data(&self->base_sensor, self->power_data);
    }
    if (self->rpm_data != NULL) {
        frsky_sport_sensor_add_single_data(&self->base_sensor, self->rpm_data);
    }
    if (self->sbec_data != NULL) {
        frsky_sport_sensor_add_single_data(&self->base_sensor, self->sbec_data);
    }
    
    // Initialize data values
    self->voltage = 0.0f;
    self->current = 0.0f;
    self->rpm = 0;
    self->consumption = 0;
    self->temperature = 0;
    self->sbec_voltage = 0.0f;
    self->sbec_current = 0.0f;
    
    return self;
}

void frsky_sport_sensor_esc_destroy(FrSkySportSensorEsc* self)
{
    if (self == NULL) {
        return;
    }
    
    // Destroy data instances
    if (self->power_data != NULL) {
        frsky_sport_single_data_destroy(self->power_data);
    }
    if (self->rpm_data != NULL) {
        frsky_sport_single_data_destroy(self->rpm_data);
    }
    if (self->temp_data != NULL) {
        frsky_sport_single_data_destroy(self->temp_data);
    }
    if (self->sbec_data != NULL) {
        frsky_sport_single_data_destroy(self->sbec_data);
    }
    
    free(self);
}

void frsky_sport_sensor_esc_set_data(FrSkySportSensorEsc* self, float volt, float curr, uint32_t rpm, uint16_t cons, float temp, float sbec_volt, float sbec_curr)
{
    if (self == NULL) {
        return;
    }
    
    // Store values
    self->voltage = volt;
    self->current = curr;
    self->rpm = rpm;
    self->consumption = cons;
    self->temperature = (int32_t)round(temp);
    self->sbec_voltage = sbec_volt;
    self->sbec_current = sbec_curr;
    
    // Update single data values
    if (self->power_data != NULL) {
        uint32_t power_value = (((uint32_t)round(curr * 100)) << 16) + (uint32_t)round(volt * 100);
        frsky_sport_single_data_set_value(self->power_data, power_value);
    }
    
    if (self->rpm_data != NULL) {
        uint32_t rpm_cons_value = (((uint32_t)cons) << 16) + (rpm / 100);
        frsky_sport_single_data_set_value(self->rpm_data, rpm_cons_value);
    }
    
    if (self->sbec_data != NULL) {
        uint32_t sbec_value = (((uint32_t)round(sbec_curr * 1000)) << 16) + (uint32_t)round(sbec_volt * 1000);
        frsky_sport_single_data_set_value(self->sbec_data, sbec_value);
    }
}

uint16_t frsky_sport_sensor_esc_decode_data(FrSkySportSensorEsc* self, uint8_t id, uint16_t app_id, uint32_t data)
{
    if (self == NULL) {
        return SENSOR_NO_DATA_ID;
    }
    
    if ((self->base_sensor.sensor_id == id) || (self->base_sensor.sensor_id == FRSKY_SENSOR_ID_IGNORE)) {
        switch (app_id) {
            case ESC_POWER_DATA_ID:
                self->voltage = (data & 0xFFFF) / 100.0f;
                self->current = (data >> 16) / 100.0f;
                return app_id;
                
            case ESC_RPM_CONS_DATA_ID:
                self->rpm = (data & 0xFFFF) * 100;
                self->consumption = data >> 16;
                return app_id;
                
            case ESC_TEMP_DATA_ID:
                self->temperature = (int32_t)data;
                return app_id;
                
            case ESC_SBEC_DATA_ID:
                self->sbec_voltage = (data & 0xFFFF) / 1000.0f;
                self->sbec_current = (data >> 16) / 1000.0f;
                return app_id;
        }
    }
    
    return SENSOR_NO_DATA_ID;
}

// Getter functions
float frsky_sport_sensor_esc_get_voltage(const FrSkySportSensorEsc* self)
{
    return (self != NULL) ? self->voltage : 0.0f;
}

float frsky_sport_sensor_esc_get_current(const FrSkySportSensorEsc* self)
{
    return (self != NULL) ? self->current : 0.0f;
}

uint32_t frsky_sport_sensor_esc_get_rpm(const FrSkySportSensorEsc* self)
{
    return (self != NULL) ? self->rpm : 0;
}

uint16_t frsky_sport_sensor_esc_get_consumption(const FrSkySportSensorEsc* self)
{
    return (self != NULL) ? self->consumption : 0;
}

int32_t frsky_sport_sensor_esc_get_temperature(const FrSkySportSensorEsc* self)
{
    return (self != NULL) ? self->temperature : 0;
}

float frsky_sport_sensor_esc_get_sbec_voltage(const FrSkySportSensorEsc* self)
{
    return (self != NULL) ? self->sbec_voltage : 0.0f;
}

float frsky_sport_sensor_esc_get_sbec_current(const FrSkySportSensorEsc* self)
{
    return (self != NULL) ? self->sbec_current : 0.0f;
}
