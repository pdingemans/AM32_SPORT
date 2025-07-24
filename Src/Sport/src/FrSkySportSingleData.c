/*
  FrSky Sport Single Data for AM32 ESC firmware
  Converted from C++ to C implementation
  (c) Pawelsky 20150725
  Not for commercial use
*/

#include "FrSkySportSingleData.h"
#include <stdlib.h>

// Create a new FrskySportSingleData instance
FrskySportSingleData* frsky_sport_single_data_create(uint32_t id, uint32_t period)
{
    FrskySportSingleData* self = (FrskySportSingleData*)malloc(sizeof(FrskySportSingleData));
    if (self != NULL) {
        // Use a cast to initialize const members
        *(uint16_t*)&self->id = id;
        *(uint32_t*)&self->period = period;
        self->value = 0;
        self->next_time_to_send = 0;
    }
    return self;
}

// Destroy a FrskySportSingleData instance
void frsky_sport_single_data_destroy(FrskySportSingleData* self)
{
    if (self != NULL) {
        free(self);
    }
}

// Send data via serial connection
void frsky_sport_single_data_send(FrskySportSingleData* self, FrSkySportSingleWireSerial* serial, uint16_t* data_id_ref, uint32_t now)
{
    if (self == NULL || serial == NULL || data_id_ref == NULL) {
        return;
    }
    
    *data_id_ref = self->id;
    if (now > self->next_time_to_send) {
        self->next_time_to_send = now + self->period;
        frsky_sport_single_wire_serial_send_data(serial, self->id, self->value);
    } else {
        frsky_sport_single_wire_serial_send_empty(serial, self->id);
        *data_id_ref = SENSOR_EMPTY_DATA_ID;
    }
}

// Set the data value
void frsky_sport_single_data_set_value(FrskySportSingleData* self, uint32_t val)
{
    if (self != NULL) {
        self->value = val;
    }
}

// Get the data value
uint32_t frsky_sport_single_data_get_value(const FrskySportSingleData* self)
{
    if (self != NULL) {
        return self->value;
    }
    return 0;
}
