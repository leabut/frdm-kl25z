/*
 * Servo2.h
 *
 *  Created on: 05.09.2020
 *      Author: spk
 */

#ifndef SERVO2_H
#define SERVO2_H

#include "driver/mcpwm.h"

class Servo2 {
public:
  Servo2(uint8_t pin);
  void setAngle(uint32_t angle);
  uint32_t getAngle() const;

private:
  uint32_t _angle = 0u;
};

#endif /* SERVO2_H */
