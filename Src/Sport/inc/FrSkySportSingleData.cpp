#include "FrSkySportSingleData.h"

 
FrskySportSingleData::FrskySportSingleData(uint32_t id,uint32_t period )
: id(id), period(period),value(0),nexttimetosend(0)
{

}
void FrskySportSingleData::send(FrSkySportSingleWireSerial &serial,  uint16_t &dataIdRef,  uint32_t now)
{
  dataIdRef = id;
  if (now > nexttimetosend)
  {
    nexttimetosend = now + period;
    serial.sendData(id, value);
  }
  else
  {
    serial.sendEmpty(id);
    dataIdRef = SENSOR_EMPTY_DATA_ID;
  }
}
void FrskySportSingleData::setvalue(uint32_t val)
{
    value=val;
}

uint32_t FrskySportSingleData::getvalue()
{
    return value;
}