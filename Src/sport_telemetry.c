/*
  FrSky Sport Telemetry for AM32 ESC firmware
  Author : pdingemans
  date: 1/10/2025
*/

#include "sport_telemetry.h"
#include "singlewire_uart.h"
#include "eeprom.h"
#include <stdlib.h>
#include <stdio.h>
#include "functions.h"


uint8_t sport_calc_sensor_id(uint8_t physical_id);
// Sensor ID enumeration
typedef enum
{
    FRSKY_SENSOR_ID1 = 0x00, // 0000 0000
    FRSKY_SENSOR_ID2 = 0xA1, // 1010 0001
    FRSKY_SENSOR_ID3 = 0x22, // 0010 0010
    FRSKY_SENSOR_ID4 = 0x83, // 1000 0011
    FRSKY_SENSOR_ID5 = 0xE4, // 1110 0100
    FRSKY_SENSOR_ID6 = 0x45,
    FRSKY_SENSOR_ID7 = 0xC6, // 1100 0110
    FRSKY_SENSOR_ID8 = 0x67,
    FRSKY_SENSOR_ID9 = 0x48,
    FRSKY_SENSOR_ID10 = 0xE9,
    FRSKY_SENSOR_ID11 = 0x6A,
    FRSKY_SENSOR_ID12 = 0xCB,
    FRSKY_SENSOR_ID13 = 0xAC,
    FRSKY_SENSOR_ID14 = 0x0D,
    FRSKY_SENSOR_ID15 = 0x8E,
    FRSKY_SENSOR_ID16 = 0x2F, // 0010 1111
    FRSKY_SENSOR_ID17 = 0xD0, // ESC1
    FRSKY_SENSOR_ID18 = 0x71, // esc2
    FRSKY_SENSOR_ID19 = 0xF2,
    FRSKY_SENSOR_ID20 = 0x53, // 0101 0011
    FRSKY_SENSOR_ID21 = 0x34, // 0011 0100
    FRSKY_SENSOR_ID22 = 0x95, // 1001 0101
    FRSKY_SENSOR_ID23 = 0x16, // 0001 0110
    FRSKY_SENSOR_ID24 = 0xB7,
    FRSKY_SENSOR_ID25 = 0x98,
    FRSKY_SENSOR_ID26 = 0x39,
    FRSKY_SENSOR_ID27 = 0xBA,
    FRSKY_SENSOR_ID28 = 0x1B,
    FRSKY_SENSOR_ID_IGNORE = 0xFF
} FrSkySportSensorId;

#define ESC_DEFAULT_ID FRSKY_SENSOR_ID10
#define ESC_DATA_COUNT 4

#define ESC_POWER_DATA_ID 0x0B50
#define ESC_RPM_CONS_DATA_ID 0x0B60
#define ESC_TEMP_DATA_ID 0x0B70
#define ESC_SBEC_DATA_ID 0x0E50
#define FRSKY_TELEMETRY_START_FRAME 0x7E
#define FRSKY_TELEMETRY_STUFFING 0x7D

#define FRSKY_RESPONSE_CHAR 0x10
#define ESC_DATA_FIELDS 4

static uint8_t frame_buffer[16];

// type definitions
typedef struct
{
    uint16_t id;
    uint32_t data;
} FrSkySportTelemetryType;
;
typedef struct
{
    FrSkySportSensorId sensor_id;
    uint8_t data_field_count;
    uint8_t ndx_to_send;
    uint8_t previous_char;
    FrSkySportTelemetryType power_value;
    FrSkySportTelemetryType rpm_value;
    FrSkySportTelemetryType temp_value;
    FrSkySportTelemetryType sbec_value;

    FrSkySportTelemetryType *data_fields[ESC_DATA_COUNT];
} FrSkySportESCSensor;

FrSkySportESCSensor esc_sensor = {
    .sensor_id = ESC_DEFAULT_ID,
    .ndx_to_send = 0,
    .previous_char = 0,
    .power_value = {ESC_POWER_DATA_ID, 0},
    .rpm_value = {ESC_RPM_CONS_DATA_ID, 0},
    .temp_value = {ESC_TEMP_DATA_ID, 0},
    .sbec_value = {ESC_SBEC_DATA_ID, 0},
    .data_fields = {
        &esc_sensor.power_value,
        &esc_sensor.rpm_value,
        &esc_sensor.temp_value,
        &esc_sensor.sbec_value}};

// Set ESC sensor data

void set_esc_sensor_data(uint16_t voltage,
                         uint16_t current,
                         uint16_t e_rpm,
                         uint16_t consumption,
                         uint16_t temp,
                         uint16_t sbec_v,
                         uint16_t sbec_a)
{

    // Update single data values
    esc_sensor.power_value.data = (((uint32_t)current << 16) + (uint32_t)voltage);
    esc_sensor.rpm_value.data = (((uint32_t)consumption) << 16) + e_rpm;
    esc_sensor.temp_value.data = (uint32_t)temp;
    esc_sensor.sbec_value.data = (((uint32_t)sbec_a << 16) + (uint32_t)sbec_v);
}

static void sportbyte(int *crc, int *pos, char b)
{
    *crc += b;
    *crc += *crc >> 8;
    *crc &= 0xff;
    if (b == FRSKY_TELEMETRY_STUFFING || b == FRSKY_TELEMETRY_START_FRAME)
    { // Byte stuffing
        b &= ~0x20;
        frame_buffer[(*pos)++] = FRSKY_TELEMETRY_STUFFING;
    }
    frame_buffer[(*pos)++] = b;
}

static int sportresp(const FrSkySportTelemetryType *data)
{
    int crc = 0, pos = 0;
    sportbyte(&crc, &pos, FRSKY_RESPONSE_CHAR);
    sportbyte(&crc, &pos, data->id & 0XFF);
    sportbyte(&crc, &pos, data->id >> 8);
    sportbyte(&crc, &pos, data->data & 0xFF);
    sportbyte(&crc, &pos, (data->data >> 8) & 0XFF);
    sportbyte(&crc, &pos, (data->data >> 16) & 0XFF);
    sportbyte(&crc, &pos, (data->data >> 24) & 0XFF);
    sportbyte(&crc, &pos, 0xff - crc);

    return pos;
}

// Receive and process telemetry data
void sport_telemetry_handle_poll(serial_telemetry_class *self)
{

    uint8_t data;
  
    //     // testing stuff
    //     static uint32_t oldmillis=0;
    //     static uint8_t senddata=0;
    //     if( millis() - oldmillis > 1000){
//         oldmillis=millis();
//         senddata++;
//         singlewire_uart_send_frame(&senddata,1);
//     }   
//     // end test stuff

    static uint8_t previous_char = 0;

    while (singlewire_uart_read_byte(&data))
    {
      
        
        uint8_t polled_id = FRSKY_SENSOR_ID_IGNORE;
        if (previous_char == FRSKY_TELEMETRY_START_FRAME)
        {
            polled_id = data;
        }
        previous_char = data;
   
        
        if (polled_id == esc_sensor.sensor_id)
        {

            singlewire_uart_send_frame(frame_buffer, sportresp(esc_sensor.data_fields[esc_sensor.ndx_to_send]));
            esc_sensor.ndx_to_send++;
            esc_sensor.ndx_to_send %= ESC_DATA_FIELDS;
        }
    }
}

void sportmakeTelemPackage(serial_telemetry_class *self, uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm)
{

   set_esc_sensor_data (voltage,
                        current,
                        e_rpm,
                        consumption,
                        temp,
                        0,  // SBEC voltage (not available)
                        0); // SBEC current (not available)
}

// Calculate sensor ID with CRC
#define BIT(x, index) (((x) >> index) & 0x01)
uint8_t sport_calc_sensor_id(uint8_t physical_id)
{
    volatile uint8_t result = physical_id;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 1) ^ BIT(physical_id, 2)) << 5;
    result += (BIT(physical_id, 2) ^ BIT(physical_id, 3) ^ BIT(physical_id, 4)) << 6;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 2) ^ BIT(physical_id, 4)) << 7;
    return result;
}


void sport_sensor_set_id(serial_telemetry_class *self, uint8_t id)
{

    esc_sensor.sensor_id = sport_calc_sensor_id(id-1);
}


serial_telemetry_class sport_telemetry = {
    .id = 0,
    .set_id = sport_sensor_set_id,
    .handle_esc_telemetry = sport_telemetry_handle_poll,
    .makeTelemPackage = sportmakeTelemPackage,
    .handle_TX_DMA_complete = sw_uart_dma_complete_handler
};
serial_telemetry_class *init_sport_telemetry(void)
{
    // here we initialize also the singlewire_uart
     singlewire_uart_init();

    
    return &sport_telemetry;
}