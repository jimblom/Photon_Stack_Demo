#include "SparkFunMAX17043.h"
#include "MAX17043_Definitions.h"

MAX17043::MAX17043()
{

}

uint8_t MAX17043::begin()
{
  Wire.begin();
  return 1;
}

float MAX17043::getVoltage()
{
  uint16_t vCell;
  vCell = read16(MAX17043_VCELL);
  // vCell is a 12-bit register where each bit represents 1.25mV
  vCell = (vCell) >> 4;

  return ((float) vCell / 800.0);
}

float MAX17043::getSOC()
{
  uint16_t soc;
  float percent;
  soc = read16(MAX17043_SOC);
  percent = (soc & 0xFF00) >> 8;
  percent += (float) (((uint8_t) soc) / 256.0);

  return percent;
}

uint16_t MAX17043::getVersion()
{
  return read16(MAX17043_VERSION);
}

uint8_t MAX17043::write8(uint8_t data, uint8_t address)
{

}

uint8_t MAX17043::write16(uint16_t data, uint8_t address)
{


}

uint8_t MAX17043::read8(uint8_t address)
{

}

uint16_t MAX17043::read16(uint8_t address)
{
  uint8_t msb, lsb;
  int16_t timeout = 1000;

  Wire.beginTransmission(MAX17043_ADDRESS);
  Wire.write(address);
  Wire.endTransmission(false);

  Wire.requestFrom(MAX17043_ADDRESS, 2);
  while ((Wire.available() < 2) && (timeout-- > 0))
    delay(1);
  msb = Wire.read();
  lsb = Wire.read();

  return ((uint16_t) msb << 8) | lsb;
}

MAX17043 gauge;
