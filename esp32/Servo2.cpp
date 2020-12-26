/*
 * Servo2.cpp
 *
 *  Created on: 05.09.2020
 *      Author: spk
 */

#include <Servo2.h>

// You can get these value from the datasheet of servo you use, in general pulse
// width varies between 1000 to 2000 microseconds
#define SERVO_MIN_PULSEWIDTH 1000  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000  // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180.0f    // Maximum angle in degree upto which servo can rotate

Servo2::Servo2(uint8_t pin) {
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, pin);

  // initial mcpwm configuration
  mcpwm_config_t pwm_config = {
      .frequency = 50,  // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
      .cmpr_a = 0.0f,   // duty cycle of PWMxA = 0
      .cmpr_b = 0.0f,   // duty cycle of PWMxb = 0
      .duty_mode = MCPWM_DUTY_MODE_0,
      .counter_mode = MCPWM_UP_COUNTER};

  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);  // Configure PWM0A & PWM0B with above settings
}

void Servo2::setAngle(uint32_t angle) {
  _angle = angle;
  uint32_t pulsewidth = 0;
  pulsewidth = SERVO_MIN_PULSEWIDTH + (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (angle / SERVO_MAX_DEGREE);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, pulsewidth);
}

uint32_t Servo2::getAngle() const {
  return _angle;
}
