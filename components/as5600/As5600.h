#ifndef _AS5600_H_
#define _AS5600_H_
#include <Hardware.h>
#include <Register.h>
#include <errno.h>
#include <limero.h>

#define I2C_AS5600 0x36

#define AS_FIRST 0x00
#define AS_ZMCO 0x00
#define AS_ZPOS 0x01
#define AS_MPOS 0x03
#define AS_MANG 0x05
#define AS_CONF 0x07
#define AS_RAW_ANGLE 0x0C
#define AS_ANGLE 0x0E
#define AS_STATUS 0x0B
#define AS_AGC 0x1A
#define AS_MAGNITUDE 0x1B
#define AS_LAST 0x1C
#define AS_BURN 0xFF

#define AS_INVALID 32767

class As5600 : public Driver {
  I2C& _i2c;
  uint8_t readReg8(uint8_t address);
  uint16_t readReg16(uint8_t address);

  void writeReg8(uint8_t address, uint8_t value);
  void writeReg16(uint8_t address, uint16_t value);

 public:
  As5600(I2C&);
  As5600(Uext&);
  int init();
  uint16_t angle();
  uint16_t rawAngle();
  uint8_t status();
  uint16_t zmco();
  uint16_t zpos();
  uint16_t mpos();
  uint16_t mang();
  uint16_t conf();
  uint16_t magnitude();
  uint8_t agc();
  void zmco(uint8_t);
  void zpos(uint16_t);
  void mpos(uint16_t);
  void mang(uint16_t);
  void conf(uint16_t);
  void burn(uint8_t);

  int degrees();
};
#endif