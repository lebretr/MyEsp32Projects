#pragma once

#include "Arduino.h"

#define DHTLIB_OK                         0
#define DHTLIB_ERROR_CHECKSUM            -1
#define DHTLIB_ERROR_TIMEOUT_A           -2
#define DHTLIB_ERROR_TIMEOUT_B           -3
#define DHTLIB_ERROR_TIMEOUT_C           -4
#define DHTLIB_ERROR_TIMEOUT_D           -5
#define DHTLIB_ERROR_TIMEOUT_E           -6
// #define DHTLIB_ERROR_BIT_SHIFT           -7
// #define DHTLIB_ERROR_SENSOR_NOT_READY    -8
// #define DHTLIB_WAITING_FOR_READ          -9

class DHTRL
{
public:

  DHTRL(uint8_t pin);
  
  int read();

  float temperature=0;
  float humidity=0;
  bool isValid=false;

private:
  uint8_t  _dataPin           = 0;
};
