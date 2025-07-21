#include "main.h"

#ifndef SPORT_TELEMETRY_H_
#define SPORT_TELEMETRY_H_

// FrSky SPORT telemetry protocol constants
#define SPORT_START_FRAME       0x7E
#define SPORT_SENSOR_ID         0x00    // ESC sensor ID
#define SPORT_FRAME_ID          0x10    // Data frame ID

// SPORT data IDs for ESC telemetry
#define SPORT_ESC_POWER_ID      0x0B50  // ESC Power (voltage/current)
#define SPORT_ESC_RPM_ID        0x0500  // ESC RPM
#define SPORT_ESC_TEMP_ID       0x0B70  // ESC Temperature
#define SPORT_CURR_ID           0x0200  // Current
#define SPORT_VFAS_ID           0x0210  // Battery voltage

typedef struct __attribute__((packed))
{
    uint8_t start_frame;    // 0x7E
    uint8_t sensor_id;      // Sensor ID
    uint8_t frame_id;       // Frame ID
    uint16_t data_id;       // Data type ID
    uint32_t data_value;    // 32-bit data value
    uint8_t crc;            // CRC checksum
} sport_telem_frame_t;

extern uint8_t aTxBuffer[49] __attribute__((aligned(4)));
extern uint8_t sport_response_pending;

void makeTelemPackage(uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm);
void makeInfoPacket(void);
void makeSportFrame(uint16_t data_id, uint32_t value);
void sport_handle_poll_request(uint8_t sensor_id);
void sport_send_response(void);

#endif
