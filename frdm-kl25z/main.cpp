/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "MMA8451Q.h"

#include "main.h"
#include "Visualizer.h"

// configurable values
#define CONTROL_RATE 50ms

double Kp = 0.50;
double ServoXOffset = 0.0;
double ServoYOffset = 0.0;

#define SERVO_PWM_LENGTH 0.020f   // 20ms pwm base frequency
#define SERVO_MAX_POS    0.00195f // 1.95ms duty cycle (171 degrees)
#define SERVO_MID_POS    0.0015f  // 1.95ms duty cycle (9 degrees)
#define SERVO_MIN_POS    0.0015f  // 1.5ms duty cycle (90 degrees)

#define RAD_TO_DEG (360.0f/(2*3.141f))

// hardware multiplexer settings
PwmOut servoX(PTB0);
PwmOut servoY(PTB1);

float AccX = 0.0f, AccY = 0.0f, AccZ = 0.0f;
double Pitch = 0.0, Yaw = 0.0;
double ServoXDeg = 0.0, ServoYDeg = 0.0;
double AngleXZAxis = 0.0, AngleYZAxis = 0.0;

int main() {
    servoX.period(SERVO_PWM_LENGTH);
    servoX.pulsewidth(SERVO_MID_POS);
    servoY.period(SERVO_PWM_LENGTH);
    servoY.pulsewidth(SERVO_MID_POS);

    visualizer::startupBlink();

    getAccData();
    getConstructionAngles();

    while (true) {
        getAccData();
        float data[3] = {AccX, AccY, AccZ};
        visualizer::visualizeAcc(data);

        transformAccData();

        data[0] = AccX;
        data[1] = AccY;
        data[2] = AccZ;
        visualizer::printfAcc(data);

        accel_degrees();
        callPidController();
        updateServoPos(ServoXDeg, ServoYDeg);

        ThisThread::sleep_for(CONTROL_RATE);
    }
}

void getConstructionAngles() {
  double vec[3] = {AccX, AccY, AccZ};
  AngleXZAxis = atan(vec[1]/vec[2]);
  rotateXAxis(vec, AngleXZAxis);
  AngleYZAxis = atan(-vec[0]/vec[2]);
}

void transformAccData() {
  double vec[3] = {AccX, AccY, AccZ};
  rotateXAxis(vec, AngleXZAxis);
  rotateYAxis(vec, AngleYZAxis);
  AccX = vec[0];
  AccY = vec[1];
  AccZ = vec[2];
}

void getAccData() {
    AccX = mma8451q.getAccX();
    AccY = mma8451q.getAccY();
    AccZ = mma8451q.getAccZ();
}

void accel_degrees () {
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  Yaw   = atan2(AccY, AccZ) * RAD_TO_DEG;
  Pitch = atan(-AccX / sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  Yaw   = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG;
  Pitch = atan2(-AccX, AccZ) * RAD_TO_DEG;
#endif
}

void callPidController() {
    ServoXDeg = 90 + Kp*Pitch + ServoXOffset;
    ServoYDeg = 90 + Kp*Yaw + ServoYOffset;
}

void updateServoPos(double servoXAngle, double servoYAngle) {
    auto servoXPwm = SERVO_MID_POS;
    auto servoYPwm = SERVO_MID_POS;

    servoXPwm = (servoXAngle / 180.0f) * 0.001f + 0.001f;
    servoYPwm = (servoYAngle / 180.0f) * 0.001f + 0.001f;

    if(servoXAngle >= 172.0) {
        servoXPwm = SERVO_MAX_POS;
    }
    if(servoXAngle <= 10.0) {
        servoXPwm = SERVO_MIN_POS;
    }

    if(servoYAngle >= 172.0) {
        servoYPwm = SERVO_MAX_POS;
    }
    if(servoYAngle <= 10.0) {
        servoYPwm = SERVO_MIN_POS;
    }

    servoX.pulsewidth(servoXPwm);
    servoY.pulsewidth(servoYPwm);
}

void rotateXAxis(double vec[3], double alpha) {
  double copyVec[3] = {0.0};
  std::memcpy(copyVec, vec, sizeof(double) * 3u);

  vec[0] = copyVec[0];
  vec[1] = cos(alpha) * copyVec[1] - sin(alpha) * copyVec[2];
  vec[2] = sin(alpha) * copyVec[1] + cos(alpha) * copyVec[2];
}

void rotateYAxis(double vec[3], double alpha) {
  double copyVec[3] = {0.0};
  std::memcpy(copyVec, vec, sizeof(double) * 3u);

  vec[0] = cos(alpha) * copyVec[0] + sin(alpha) * copyVec[2];
  vec[1] = copyVec[1];
  vec[2] = -sin(alpha) * copyVec[0] + cos(alpha) * copyVec[2];
}
