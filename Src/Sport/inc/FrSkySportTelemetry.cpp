/*
  FrSky telemetry class for Teensy 3.x and 328P based boards (e.g. Pro Mini,
  Nano, Uno)
  (c) Pawelsky 20150725
  Not for commercial use
*/

#include "FrSkySportTelemetry.h"
#include "hardware/timer.h"
#include "stdio.h"

FrSkySportTelemetry::FrSkySportTelemetry()
{

  prevData = FrSkySportSensor::ID_IGNORE;
}

void FrSkySportTelemetry::begin()
{
  serial.begin();
}

uint16_t FrSkySportTelemetry::send()
{
  uint16_t dataId = SENSOR_NO_DATA_ID;

  //  serial cant be 0 if we are here, so we do not check it

  uint8_t polledId = FrSkySportSensor::ID_IGNORE;
  uint32_t now = time_us_32();

  if (serial.available())
  {
    uint8_t data = 0;

    data = serial.read();
    //  printf("%X ", data);
    // // return 0;

    if (prevData == FRSKY_TELEMETRY_START_FRAME)
    {
      polledId = data;
    }
    prevData = data;
  }

  if (polledId != FrSkySportSensor::ID_IGNORE)
  {

    // Send the actual data

    //printf("pollde id : %X \n", polledId);
    for (auto sensor : sensors)
    {
      // the sensor will check if it;s the correct id, otherwise it will return
      // that's a bit silly, isn't it?
      dataId = sensor->send(serial, polledId, now);
    }
  }

  if (dataId == SENSOR_EMPTY_DATA_ID)
  {
    dataId = SENSOR_NO_DATA_ID; // If empty frame was sent we return SENSOR_NO_DATA_ID as no actual data has been sent
  }
  return dataId;
}

void FrSkySportTelemetry::addSensor(FrSkySportSensor *sensor)
{
  sensors.push_back(sensor);
}
