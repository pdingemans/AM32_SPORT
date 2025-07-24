/*
  FrSky Sport Single Wire Serial for AM32 ESC firmware
  Converted from C++ to C implementation
  Based on the work of Pawelsky version: 2021050
  Not for commercial use
*/

#include "FrSkySportSingleWireSerial.h"
#include <stdlib.h>

// For AT32F421 software UART support
#include "singlewire_sw_uart.h"

#define FRSKY_SENSOR_DATA_FRAME 0x10
#define FRSKY_STUFFING 0x7D

// Forward declarations
void frsky_sport_single_wire_serial_set_mode(FrSkySportSingleWireSerial* self, FrSkySportSerialMode mode);

// Create a new FrSkySportSingleWireSerial instance
FrSkySportSingleWireSerial* frsky_sport_single_wire_serial_create(void)
{
    FrSkySportSingleWireSerial* self = (FrSkySportSingleWireSerial*)malloc(sizeof(FrSkySportSingleWireSerial));
    if (self != NULL) {
        self->crc = 0;
    }
    return self;
}

// Destroy a FrSkySportSingleWireSerial instance
void frsky_sport_single_wire_serial_destroy(FrSkySportSingleWireSerial* self)
{
    if (self != NULL) {
        free(self);
    }
}

// Initialize the serial interface
void frsky_sport_single_wire_serial_begin(FrSkySportSingleWireSerial* self)
{
    if (self == NULL) {
        return;
    }

    // AT32F421 software UART initialization - should be done by main application
    // Start in RX mode
    frsky_sport_single_wire_serial_set_mode(self, FRSKY_SERIAL_MODE_RX);

    self->crc = 0;
}

// Set serial mode (TX or RX)
void frsky_sport_single_wire_serial_set_mode(FrSkySportSingleWireSerial* self, FrSkySportSerialMode mode)
{
    if (self == NULL) {
        return;
    }

    // AT32F421 software UART mode switching
    if (mode == FRSKY_SERIAL_MODE_TX) {
        sw_uart_disable_rx();
        sw_uart_enable_tx();
    } else {
        sw_uart_disable_tx();
        sw_uart_enable_rx();
    }
}

// Send a single byte with stuffing
void frsky_sport_single_wire_serial_send_byte(FrSkySportSingleWireSerial* self, uint8_t byte)
{
    if (self == NULL) {
        return;
    }

    // AT32F421 software UART implementation
    if (byte == 0x7E) {
        sw_uart_send_byte(FRSKY_STUFFING);
        sw_uart_send_byte(0x5E); // 0x7E xor 0x20
    } else if (byte == 0x7D) {
        sw_uart_send_byte(FRSKY_STUFFING);
        sw_uart_send_byte(0x5D); // 0x7D xor 0x20
    } else {
        sw_uart_send_byte(byte);
    }

    // Calculate CRC
    self->crc += byte;
    self->crc += self->crc >> 8;
    self->crc &= 0x00ff;
    self->crc += self->crc >> 8;
    self->crc &= 0x00ff;
}

// Send CRC byte
void frsky_sport_single_wire_serial_send_crc(FrSkySportSingleWireSerial* self)
{
    if (self == NULL) {
        return;
    }

    // AT32F421 software UART implementation
    sw_uart_send_byte(0xFF - self->crc);
    self->crc = 0;
}

// Send telemetry data
void frsky_sport_single_wire_serial_send_data(FrSkySportSingleWireSerial* self, uint16_t data_type_id, uint32_t data)
{
    if (self == NULL) {
        return;
    }

    frsky_sport_single_wire_serial_set_mode(self, FRSKY_SERIAL_MODE_TX);
    frsky_sport_single_wire_serial_send_byte(self, FRSKY_SENSOR_DATA_FRAME);
    
    uint8_t* bytes = (uint8_t*)&data_type_id;
    frsky_sport_single_wire_serial_send_byte(self, bytes[0]);
    frsky_sport_single_wire_serial_send_byte(self, bytes[1]);
    
    bytes = (uint8_t*)&data;
    frsky_sport_single_wire_serial_send_byte(self, bytes[0]);
    frsky_sport_single_wire_serial_send_byte(self, bytes[1]);
    frsky_sport_single_wire_serial_send_byte(self, bytes[2]);
    frsky_sport_single_wire_serial_send_byte(self, bytes[3]);
    
    frsky_sport_single_wire_serial_send_crc(self);

    // Wait for transmission to complete
    while (!sw_uart_tx_complete()) {
        // Wait for TX complete
    }

    frsky_sport_single_wire_serial_set_mode(self, FRSKY_SERIAL_MODE_RX);
}

// Send empty frame
void frsky_sport_single_wire_serial_send_empty(FrSkySportSingleWireSerial* self, uint16_t data_type_id)
{
    if (self == NULL) {
        return;
    }

    frsky_sport_single_wire_serial_set_mode(self, FRSKY_SERIAL_MODE_TX);
    frsky_sport_single_wire_serial_send_byte(self, 0x00);
    
    uint8_t* bytes = (uint8_t*)&data_type_id;
    frsky_sport_single_wire_serial_send_byte(self, bytes[0]);
    frsky_sport_single_wire_serial_send_byte(self, bytes[1]);
    
    for (uint8_t i = 0; i < 4; i++) {
        frsky_sport_single_wire_serial_send_byte(self, 0x00);
    }
    
    frsky_sport_single_wire_serial_send_crc(self);

    // Wait for transmission to complete
    while (!sw_uart_tx_complete()) {
        // Wait for TX complete
    }

    frsky_sport_single_wire_serial_set_mode(self, FRSKY_SERIAL_MODE_RX);
}

// Check if data is available
uint8_t frsky_sport_single_wire_serial_available(FrSkySportSingleWireSerial* self)
{
    if (self == NULL) {
        return 0;
    }

    // For AT32F421 software UART, data availability is handled by callbacks
    // This would need to be implemented with a receive buffer/queue
    return 0; // Placeholder - implement with RX buffer when needed
}

// Read a byte from serial
uint8_t frsky_sport_single_wire_serial_read(FrSkySportSingleWireSerial* self)
{
    if (self == NULL) {
        return 0;
    }

    // For AT32F421 software UART, data reading is handled by callbacks
    // This would need to be implemented with a receive buffer/queue
    return 0; // Placeholder - implement with RX buffer when needed
}
