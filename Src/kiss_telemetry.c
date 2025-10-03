#include "stddef.h"
#include "kiss_telemetry.h"
#include "serial_telemetry.h"
#include "functions.h"
#include "eeprom.h"
#include "common.h"

typedef struct __attribute__((packed))
{
    uint8_t temperature; // temperature in Celcius
    uint8_t voltage_h;   // voltage in centivolts
    uint8_t voltage_l;
    uint8_t current_h; // current in centiamps
    uint8_t current_l;
    uint8_t consumption_h; // accumulated current consumption in mAH
    uint8_t consumption_l;
    uint8_t erpm_h; // eRPM * 100, so 1 in the packet means 100 eRPM
    uint8_t erpm_l;
    uint8_t crc;
} kiss_telem_pkt_t; // sizeof(kiss_telem_pkt_t) = 10

extern uint8_t aTxBuffer[49] __attribute__((aligned(4)));

extern uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);

uint8_t aTxBuffer[49] __attribute__((aligned(4)));
uint8_t esc_info_buffer[10] __attribute__((aligned(4)));

// this function copies the data to a local buffer
// so we can send it when requested
void makeInfoPacket()
{
    for(int i = 0; i < 48; i++) {
        aTxBuffer[i] = eepromBuffer.buffer[i];
    }
    aTxBuffer[48] = get_crc8(aTxBuffer, 48);
}
void makeTelemPackage(serial_telemetry_class *self, uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm)
{
    kiss_telem_pkt_t *telem_pkt = (kiss_telem_pkt_t *)esc_info_buffer;

    telem_pkt->temperature = temp; // temperature in Celcius

    // voltage in centivolts
    telem_pkt->voltage_h = (voltage >> 8) & 0xFF;
    telem_pkt->voltage_l = voltage & 0xFF;

    // current in centiamps
    telem_pkt->current_h = (current >> 8) & 0xFF;
    telem_pkt->current_l = current & 0xFF;

    // accumulated current consumption in mAH
    telem_pkt->consumption_h = (consumption >> 8) & 0xFF;
    telem_pkt->consumption_l = consumption & 0xFF;

    // eRPM * 100, so 1 in the packet means 100 eRPM
    telem_pkt->erpm_h = (e_rpm >> 8) & 0xFF;
    telem_pkt->erpm_l = e_rpm & 0xFF;

    telem_pkt->crc = get_crc8((uint8_t *)telem_pkt, sizeof(kiss_telem_pkt_t) - 1);
}

void send_telem_data(void)
{
    // we send the esc_info_buffer which is 10 bytes long
    // or we send the aTxBuffer which is 49 bytes long
    // depending on the length passed to this function
    for (uint8_t i = 0; i < 10; i++)
    {
        aTxBuffer[i] = esc_info_buffer[i];
    }
    send_telem_DMA(10);
}
void set_kiss_telemetry_id(serial_telemetry_class *self, uint8_t new_id)
{
    self->id = new_id;
    if (new_id > 0)
    {
        self->update_on_interval = 29 + new_id; // send every 30 + id msec
    }
}

void handle_kiss_esc_telemetry(serial_telemetry_class *self)
{
    static uint32_t old_millis = 0;
    // this send_telemetry is set from dshot input
    if (send_telemetry)
    {
        send_telem_data();
        send_telemetry = 0;
    }
    if (send_esc_info_flag)
    {
        makeInfoPacket();
        send_telem_DMA(49);
        send_esc_info_flag = 0;
    }
    if (self->update_on_interval == 0)
        return;

    // if update_on_interval is 0, no automatic telemetry is send
    // here we send it every 30 msec + the esc id...

    if (millis() - old_millis >= (uint32_t)self->update_on_interval)
    {
        send_telem_data();
        old_millis = millis();
    }
}

serial_telemetry_class kiss_telemetry = {
    .id = 0,
    .update_on_interval = 0,
    .set_id = set_kiss_telemetry_id,
    .handle_esc_telemetry = handle_kiss_esc_telemetry,
    .makeTelemPackage = makeTelemPackage,
};

serial_telemetry_class *init_kiss_telemetry()
{
    telem_UART_Init();
    return &kiss_telemetry;
}
