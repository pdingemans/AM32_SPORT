#ifndef _FRSKY_SPORT_SINGLE_DATA_H_
#define _FRSKY_SPORT_SINGLE_DATA_H_

#include <stdint.h>
#include "FrSkySportSingleWireSerial.h" 

#define SENSOR_EMPTY_DATA_ID 0xFFFF // Data ID of an empty data frame

// FrskySportSingleData ADT structure
typedef struct {
    const uint16_t  id;
    const uint32_t  period;
    uint32_t        value;
    uint32_t        next_time_to_send;
} FrskySportSingleData;

// Function prototypes
FrskySportSingleData* frsky_sport_single_data_create(uint32_t id, uint32_t period);
void frsky_sport_single_data_destroy(FrskySportSingleData* self);
void frsky_sport_single_data_send(FrskySportSingleData* self, FrSkySportSingleWireSerial* serial, uint16_t* data_id_ref, uint32_t now);
void frsky_sport_single_data_set_value(FrskySportSingleData* self, uint32_t val);
uint32_t frsky_sport_single_data_get_value(const FrskySportSingleData* self);

#endif // _FRSKY_SPORT_SINGLE_DATA_H_
