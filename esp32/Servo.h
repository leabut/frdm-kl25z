/*
 * Servo.h
 *
 *  Created on: 05.09.2020
 *      Author: spk
 */

#ifndef SERVO_H
#define SERVO_H

#include "driver/mcpwm.h"

class Servo {
public:
  Servo(uint8_t pin);
  void setAngle(uint32_t angle);
  uint32_t getAngle() const;

private:
  uint32_t _angle = 0u;
};

#endif /* PWM_H */
