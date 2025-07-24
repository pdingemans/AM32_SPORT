/*
  FrSky halfduplex serial class for RP2040 based boards (e.g. RP2040 zero)
  based on the work of  Pawelsky version : 2021050

*/
#include "hardware/gpio.h"
#include "FrSkySportSingleWireSerial.h"
#include <stdio.h>

#define FRSKY_SENSOR_DATA_FRAME 0x10
#define FRSKY_STUFFING 0x7D

FrSkySportSingleWireSerial::FrSkySportSingleWireSerial()
{

  port = nullptr;
}

void FrSkySportSingleWireSerial::begin()
{

  port = UART_ID;
  uart_init(UART_ID, BAUDRATE);
  uart_set_format(port, 8, 1, UART_PARITY_NONE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT);
  gpio_set_oeover(UART_TX_PIN,GPIO_OVERRIDE_LOW);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  setMode(RX);

  crc = 0;
}

void FrSkySportSingleWireSerial::setMode(SerialMode mode)
{

  if (mode == TX)
  {
    //gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    //gpio_set_oeover(UART_TX_PIN, GPIO_OVERRIDE_HIGH);
   //gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT);
    gpio_set_oeover(UART_TX_PIN,GPIO_OVERRIDE_HIGH);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_NULL); // no function at all
  }
  else
  {
    //gpio_set_oeover(UART_TX_PIN, GPIO_OVERRIDE_LOW);
    //gpio_set_function(UART_TX_PIN, GPIO_FUNC_NULL);
    gpio_set_oeover(UART_TX_PIN,GPIO_OVERRIDE_LOW);
    
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_inover(UART_RX_PIN, GPIO_OVERRIDE_INVERT);
  }
}

void FrSkySportSingleWireSerial::sendByte(uint8_t byte)
{
  //printf("%X ", byte);
  if (byte == 0x7E)
  {
    uart_putc_raw(port, FRSKY_STUFFING);
    uart_putc_raw(port, 0x5E); // 0x7E xor 0x20
  }
  else if (byte == 0x7D)
  {
    uart_putc_raw(port, FRSKY_STUFFING);
    uart_putc_raw(port, 0x5D); // 0x7D xor 0x20
  }
  else
  {
    uart_putc_raw(port, byte);
  }
  // crc = ((crc+byte) + ((crc+byte) >> 8)) & 0x00ff;
  crc += byte;
  crc += crc >> 8;
  crc &= 0x00ff;
  crc += crc >> 8; // these lines do nothing, can be removed
  crc &= 0x00ff;   // these lines do nothing, can be removed
  //     ldrh	r3, [r0, #4]
  // 10004df0:	185b      	adds	r3, r3, r1
  // 10004df2:	b29b      	uxth	r3, r3
  // 10004df4:	8083      	strh	r3, [r0, #4]
  // 10004df6:	0a1a      	lsrs	r2, r3, #8
  // 10004df8:	189b      	adds	r3, r3, r2
  // 10004dfa:	b29b      	uxth	r3, r3
  // 10004dfc:	8083      	strh	r3, [r0, #4]
  // 10004dfe:	22ff      	movs	r2, #255	; 0xff
  // 10004e00:	4013      	ands	r3, r2
  // 10004e02:	8083      	strh	r3, [r0, #4]
}

void FrSkySportSingleWireSerial::sendCrc()
{

  uart_putc_raw(port, 0xFF - crc);
  crc = 0;
}

void FrSkySportSingleWireSerial::sendData(uint16_t dataTypeId, uint32_t data)
{

  setMode(TX);
  sendByte(FRSKY_SENSOR_DATA_FRAME);
  uint8_t *bytes = (uint8_t *)&dataTypeId;
  sendByte(bytes[0]);
  sendByte(bytes[1]);
  bytes = (uint8_t *)&data;
  sendByte(bytes[0]);
  sendByte(bytes[1]);
  sendByte(bytes[2]);
  sendByte(bytes[3]);
  sendCrc();
  uart_tx_wait_blocking(port);// lets wait till data has left the buffer...
  //sleep_us(1500); 
  setMode(RX);
}

void FrSkySportSingleWireSerial::sendEmpty(uint16_t dataTypeId)
{

  setMode(TX);
  sendByte(0x00);
  uint8_t *bytes = (uint8_t *)&dataTypeId;
  sendByte(bytes[0]);
  sendByte(bytes[1]);
  for (uint8_t i = 0; i < 4; i++)
  {
    sendByte(0x00);
  }
  sendCrc();
  uart_tx_wait_blocking(port);// lets wait till data has left the buffer...
  
  setMode(RX);
}

uint8_t FrSkySportSingleWireSerial::available()
{

  return uart_is_readable(port);
}
uint8_t FrSkySportSingleWireSerial::read()
{
  return uart_getc(port);
}