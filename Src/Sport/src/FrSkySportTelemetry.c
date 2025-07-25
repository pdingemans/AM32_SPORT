/*
  FrSky Sport Telemetry for AM32 ESC firmware
  Converted from C++ to C implementation
  (c) Pawelsky 20150725
  Not for commercial use
*/

#include "../FrSkySportTelemetry.h"
#include <stdlib.h>
#include <stdio.h>

#ifdef ENABLE_FRSKY_SPORT_TELEMETRY
#include "../../../Mcu/f421/Inc/singlewire_sw_uart.h"
#include "at32f421.h"
#endif

// Create a new FrSkySportTelemetry instance
FrSkySportTelemetry* frsky_sport_telemetry_create(void)
{
    FrSkySportTelemetry* self = (FrSkySportTelemetry*)malloc(sizeof(FrSkySportTelemetry));
    if (self != NULL) {
        self->sensor_count = 0;
        self->prev_data = FRSKY_SENSOR_ID_IGNORE;
        
        // Initialize the sensor array to NULL
        for (uint8_t i = 0; i < FRSKY_TELEMETRY_MAX_SENSORS; i++) {
            self->sensors[i] = NULL;
        }
        
#ifdef ENABLE_FRSKY_SPORT_TELEMETRY
        // Initialize software UART configuration for Sport telemetry
        // PB6, 57600 baud, inverted logic, Timer 10
        self->uart_config.gpio_port = GPIOB;
        self->uart_config.gpio_pin = GPIO_PINS_6;
        self->uart_config.baud_rate = 57600;
        self->uart_config.inverted = true;        // Sport uses inverted logic
        self->uart_config.idle_state = 0;         // Inverted idle = LOW
        
        // EXTI configuration for PB6
        self->uart_config.exti_line = EXINT_LINE_6;
        self->uart_config.exti_irq = EXINT15_4_IRQn;
        self->uart_config.scfg_source = SCFG_PINS_SOURCE6;
        
        // Timer configuration (using TMR17 for both TX and RX)
        self->uart_config.timer = TMR17;
        self->uart_config.timer_clk = CRM_TMR17_PERIPH_CLOCK;
        self->uart_config.timer_irq = TMR17_GLOBAL_IRQn;
        
        // Initialize software UART
        sw_uart_init(&self->uart_config);
#endif
        
        // Initialize the serial structure
        // Note: This assumes the serial structure is embedded, not a pointer
        // If it needs to be a pointer, this would need to be modified
    }
    return self;
}

// Destroy a FrSkySportTelemetry instance
void frsky_sport_telemetry_destroy(FrSkySportTelemetry* self)
{
    if (self != NULL) {
#ifdef ENABLE_FRSKY_SPORT_TELEMETRY
        sw_uart_deinit();
#endif
        free(self);
    }
}

// Initialize the telemetry system
void frsky_sport_telemetry_begin(FrSkySportTelemetry* self)
{
    if (self == NULL) {
        return;
    }
    
    frsky_sport_single_wire_serial_begin(&self->serial);
}

// Send telemetry data
uint16_t frsky_sport_telemetry_send(FrSkySportTelemetry* self)
{
    if (self == NULL) {
        return SENSOR_NO_DATA_ID;
    }
    
    uint16_t data_id = SENSOR_NO_DATA_ID;
    uint8_t polled_id = FRSKY_SENSOR_ID_IGNORE;
    
#ifdef RASPBERRY_PI_PICO
    uint32_t now = time_us_32();
#else
    uint32_t now = 0; // Platform-specific time implementation needed
#endif

    if (frsky_sport_single_wire_serial_available(&self->serial)) {
        uint8_t data = frsky_sport_single_wire_serial_read(&self->serial);
        
        if (self->prev_data == FRSKY_TELEMETRY_START_FRAME) {
            polled_id = data;
        }
        self->prev_data = data;
    }

    if (polled_id != FRSKY_SENSOR_ID_IGNORE) {
        // Send the actual data
        for (uint8_t i = 0; i < self->sensor_count; i++) {
            if (self->sensors[i] != NULL) {
                // The sensor will check if it's the correct id, otherwise it will return
                data_id = frsky_sport_sensor_send(self->sensors[i], &self->serial, polled_id, now);
            }
        }
    }

    if (data_id == SENSOR_EMPTY_DATA_ID) {
        data_id = SENSOR_NO_DATA_ID; // If empty frame was sent we return SENSOR_NO_DATA_ID as no actual data has been sent
    }
    
    return data_id;
}

// Add a sensor to the telemetry system
void frsky_sport_telemetry_add_sensor(FrSkySportTelemetry* self, FrSkySportSensor* sensor)
{
    if (self == NULL || sensor == NULL) {
        return;
    }
    
    if (self->sensor_count < FRSKY_TELEMETRY_MAX_SENSORS) {
        self->sensors[self->sensor_count] = sensor;
        self->sensor_count++;
    }
}
