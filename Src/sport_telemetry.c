#include "sport_telemetry.h"
#include "eeprom.h"

extern uint8_t get_crc8(uint8_t* Buf, uint8_t BufLen);
extern void send_telem_DMA(uint8_t length);
extern uint8_t degrees_celsius;
extern uint16_t battery_voltage;
extern uint16_t actual_current;
extern uint32_t consumed_current;
extern uint16_t e_rpm;

uint8_t aTxBuffer[49] __attribute__((aligned(4)));
static uint8_t sport_data_index = 0;
uint8_t sport_response_pending = 0;
static uint16_t sport_response_delay_us = 500; // 500µs delay

// Calculate SPORT CRC
uint8_t sport_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Byte stuffing for SPORT protocol (escape 0x7E and 0x7D)
uint8_t sport_stuff_byte(uint8_t byte, uint8_t *buffer, uint8_t *index)
{
    if (byte == 0x7E) {
        buffer[(*index)++] = 0x7D;
        buffer[(*index)++] = 0x5E;
        return 2;
    } else if (byte == 0x7D) {
        buffer[(*index)++] = 0x7D;
        buffer[(*index)++] = 0x5D;
        return 2;
    } else {
        buffer[(*index)++] = byte;
        return 1;
    }
}

void makeSportFrame(uint16_t data_id, uint32_t value)
{
    uint8_t frame[8];
    uint8_t buffer_index = 0;
    
    // Build raw frame
    frame[0] = SPORT_SENSOR_ID;
    frame[1] = SPORT_FRAME_ID;
    frame[2] = data_id & 0xFF;
    frame[3] = (data_id >> 8) & 0xFF;
    frame[4] = value & 0xFF;
    frame[5] = (value >> 8) & 0xFF;
    frame[6] = (value >> 16) & 0xFF;
    frame[7] = (value >> 24) & 0xFF;
    
    // Calculate CRC on frame data (excluding start byte)
    uint8_t crc = sport_crc(frame, 8);
    
    // Start frame
    aTxBuffer[buffer_index++] = SPORT_START_FRAME;
    
    // Stuff frame bytes
    for (uint8_t i = 0; i < 8; i++) {
        sport_stuff_byte(frame[i], aTxBuffer, &buffer_index);
    }
    
    // Stuff CRC
    sport_stuff_byte(crc, aTxBuffer, &buffer_index);
}

void makeTelemPackage(uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm)
{
    // Cycle through different telemetry data types on each call
    switch(sport_data_index) {
        case 0:
            // Send voltage (in centivolts)
            makeSportFrame(SPORT_VFAS_ID, voltage);
            break;
        case 1:
            // Send current (in centiamps)
            makeSportFrame(SPORT_CURR_ID, current);
            break;
        case 2:
            // Send RPM (eRPM)
            makeSportFrame(SPORT_ESC_RPM_ID, e_rpm);
            break;
        case 3:
            // Send temperature (in Celsius)
            makeSportFrame(SPORT_ESC_TEMP_ID, temp);
            break;
    }
    
    // Cycle to next data type
    sport_data_index = (sport_data_index + 1) % 4;
}

void makeInfoPacket()
{
    // Keep the same info packet format for compatibility
    for(int i = 0; i < 48; i++) {
        aTxBuffer[i] = eepromBuffer.buffer[i];
    }
    aTxBuffer[48] = get_crc8(aTxBuffer, 48);
}

// Handle SPORT polling request from receiver
void sport_handle_poll_request(uint8_t sensor_id)
{
    if (sensor_id == SPORT_SENSOR_ID) {
        // Start timer for 500µs response delay
        TMR10->ctrl1_bit.tmren = FALSE; // Stop timer first
        TMR10->cval = 0; // Reset counter
        TMR10->pr = (144 * sport_response_delay_us) / 1000; // 144MHz / 1000 = 144kHz, so 144 * 500 / 1000 = ~72 for 500µs
        sport_response_pending = 1;
        TMR10->ctrl1_bit.tmren = TRUE; // Start timer
    }
}

// Send SPORT response with current telemetry data
void sport_send_response(void)
{
    // Create telemetry frame with current sensor data
    makeTelemPackage(degrees_celsius, battery_voltage, actual_current, 
                    (uint16_t)(consumed_current >> 16), e_rpm);
    
    // Send the frame via DMA
    // Frame length varies due to byte stuffing, but max is about 12 bytes
    send_telem_DMA(12);
}
