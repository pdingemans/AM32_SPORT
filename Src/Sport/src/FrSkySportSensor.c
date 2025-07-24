/*
  FrSky Sport Sensor for AM32 ESC firmware
  Converted from C++ to C implementation
  (c) Pawelsky 20150725
  Not for commercial use
*/

#include "FrSkySportSensor.h"
#include <stdlib.h>
#include <stdio.h>

// Create a new FrSkySportSensor instance
FrSkySportSensor* frsky_sport_sensor_create(FrSkySportSensorId id)
{
    FrSkySportSensor* self = (FrSkySportSensor*)malloc(sizeof(FrSkySportSensor));
    if (self != NULL) {
        self->sensor_id = id;
        self->sensor_data_idx = 0;
        self->single_data_count = 0;
        
        // Initialize the array to NULL
        for (uint8_t i = 0; i < MAX_SINGLE_DATA_COUNT; i++) {
            self->single_datas[i] = NULL;
        }
    }
    return self;
}

// Destroy a FrSkySportSensor instance
void frsky_sport_sensor_destroy(FrSkySportSensor* self)
{
    if (self != NULL) {
        free(self);
    }
}

// Decode incoming data (virtual function replacement)
uint16_t frsky_sport_sensor_decode_data(FrSkySportSensor* self, uint8_t id, uint16_t app_id, uint32_t data)
{
    if (self == NULL) {
        return SENSOR_NO_DATA_ID;
    }
    
    // Base implementation returns no data
    // This should be overridden by specific sensor implementations
    return SENSOR_NO_DATA_ID;
}

// Send sensor data
uint16_t frsky_sport_sensor_send(FrSkySportSensor* self, FrSkySportSingleWireSerial* serial, uint8_t id, uint32_t now)
{
    if (self == NULL || serial == NULL) {
        return SENSOR_NO_DATA_ID;
    }
    
    uint16_t data_id = SENSOR_NO_DATA_ID;
    
    // We only send the data if we are the one that's supposed to send it
    if (self->sensor_id == id) {
        // We iterate through the elements in the collection
        // Time etc. is handled in the send method
        if (self->single_data_count > 0 && self->single_datas[self->sensor_data_idx] != NULL) {
            frsky_sport_single_data_send(self->single_datas[self->sensor_data_idx], serial, &data_id, now);
            
            self->sensor_data_idx++;
            if (self->sensor_data_idx >= self->single_data_count) {
                self->sensor_data_idx = 0;
            }
        }
    }
    
    return data_id;
}

// Add single data to the sensor
void frsky_sport_sensor_add_single_data(FrSkySportSensor* self, FrskySportSingleData* s_data)
{
    if (self == NULL || s_data == NULL) {
        return;
    }
    
    if (self->single_data_count < MAX_SINGLE_DATA_COUNT) {
        self->single_datas[self->single_data_count] = s_data;
        self->single_data_count++;
    }
}

// Calculate sensor ID with CRC
uint8_t frsky_sport_calc_sensor_id(const uint8_t physical_id)
{
    uint8_t result = physical_id;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 1) ^ BIT(physical_id, 2)) << 5;
    result += (BIT(physical_id, 2) ^ BIT(physical_id, 3) ^ BIT(physical_id, 4)) << 6;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 2) ^ BIT(physical_id, 4)) << 7;
    return result;
}
