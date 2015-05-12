#ifndef __SparkFunMAX17043_H__
#define __SparkFunMAX17043_H__

#include "application.h"

class MAX17043
{
public:
  MAX17043();

  uint8_t begin();

  float getVoltage();
  float getSOC();
  uint16_t getVersion();
  uint16_t getCompensation();

  uint8_t setCompensation();

  uint8_t quickStart();

  uint8_t sleep();
  uint8_t wake();

  uint8_t setThreshold(uint8_t percent);
  uint8_t enableAlert(bool enable = true);

  uint8_t powerOnReset();

private:
  uint8_t write8(uint8_t data, uint8_t address);
  uint8_t write16(uint16_t data, uint8_t address);
  uint8_t read8(uint8_t address);
  uint16_t read16(uint8_t address);
};

extern MAX17043 gauge;

#endif
