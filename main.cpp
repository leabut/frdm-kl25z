/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "MMA8451Q.h"

PinName ACC_SDA = PTE25;
PinName ACC_SCL = PTE24;

#define MMA8451_I2C_ADDRESS (0x1d<<1)

#define SLEEP_RATE 50ms

#define RAD_TO_DEG (360.0f/(2*3.141f))

MMA8451Q acc(ACC_SDA, ACC_SCL, MMA8451_I2C_ADDRESS);
PwmOut rled(LED1);
PwmOut gled(LED2);
PwmOut bled(LED3);
PwmOut servoX(PTB0);
PwmOut servoY(PTB1);

// specify period first
float pwmLength = 0.020f; // 20ms

float AccX = 0.0f, AccY = 0.0f, AccZ = 0.0f;
double dt = 0.0, currentTime = 0.0, previousTime = 0.0;
double pitch = 0.0, yaw = 0.0;
double errorX = 0.0, errorY = 0.0;
double pidX_p = 0.0, pidY_p = 0.0;
double PIDX = 0.0, PIDY = 0.0;
double servoXDeg = 0.0, servoYDeg = 0.0;

double AngleXZAxis = 0.0, AngleYZAxis = 0.0;

double kp = 0.50;
double kd = 0.0;
double ki = 0.0;

double desired_angleX = 0.0, desired_angleY = 0.0;
double servoX_offset = 0.0;
double servoY_offset = 0.0;

void getAccData();
void visualizeAcc();
void printfAcc();
void accel_degrees();
void pidcompute();
void updateServoPos();
void getConstructionAngles();
void correctAccData();

double angleVecToVec(double vec1[3], double vec2[3]);
void rotateXAxis(double vec[3], double alpha);
void rotateYAxis(double vec[3], double alpha);
void rotateZAxis(double vec[3], double alpha);

int main() {
    servoX.period(pwmLength);
    servoX.pulsewidth(0.0015f);
    servoY.period(pwmLength);
    servoY.pulsewidth(0.0015f);

    rled = 1.0f;
    bled = 1.0f;
    for(auto i = 0u; i < 10u; i++) {
      if(i % 2u == 0u) {
        gled = 0.0f;
      } else {
        gled = 1.0f;
      }
      ThisThread::sleep_for(50ms);
    }

    getAccData();
    getConstructionAngles();

    while (true) {
        previousTime = currentTime;
        //currentTime = us_ticker_read() / 1000L;
        dt = (currentTime - previousTime) / 1000;

        getAccData();
        visualizeAcc();
        //printfAcc();

        correctAccData();
        printfAcc();

        accel_degrees();
        pidcompute();
        updateServoPos();

        ThisThread::sleep_for(SLEEP_RATE);
    }
}

void getConstructionAngles() {
  double vec[3] = {AccX, AccY, AccZ};
  AngleXZAxis = atan(vec[1]/vec[2]);
  rotateXAxis(vec, AngleXZAxis);
  AngleYZAxis = atan(-vec[0]/vec[2]);
}

void correctAccData() {
  double vec[3] = {AccX, AccY, AccZ};
  rotateXAxis(vec, AngleXZAxis);
  rotateYAxis(vec, AngleYZAxis);
  AccX = vec[0];
  AccY = vec[1];
  AccZ = vec[2];
}

void getAccData() {
    AccX = acc.getAccX();
    AccY = acc.getAccY();
    AccZ = acc.getAccZ();
}

void printfAcc() {
    int x = AccX * 1000u;
    int y = AccY * 1000u;
    int z = AccZ * 1000u;
    printf("X: %d, Y: %d, Z: %d\n", x, y, z);
}

void visualizeAcc() {
    rled = 1.0f - AccX;
    gled = 1.0f - AccY;
    bled = 1.0f - AccZ;
}

void accel_degrees () {
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  yaw   = atan2(AccY, AccZ) * RAD_TO_DEG;
  pitch = atan(-AccX / sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  yaw   = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG;
  pitch = atan2(-AccX, AccZ) * RAD_TO_DEG;
#endif
}

void pidcompute() {
    auto previous_errorX = errorX;
    auto previous_errorY = errorY;

    errorX = pitch - desired_angleX;
    errorY = yaw - desired_angleY;

    //Defining "P"
    pidX_p = kp*errorX;
    pidY_p = kp*errorY;

    //Defining "D"
    /*
    pidX_d = kd*((errorX - previous_errorX)/dt);
    pidY_d = kd*((errorY - previous_errorY)/dt);

    //Defining "I"
    pidX_i = ki * (pidX_i + errorX * dt);
    pidY_i = ki * (pidY_i + errorY * dt);
    */

    PIDX = pidX_p; // + pidX_i + pidX_d;
    PIDY = pidY_p; // + pidY_i + pidY_d;

    servoXDeg = 90 + PIDX;
    servoYDeg = 90 + PIDY;

    int a = servoXDeg * 10;
    int b = servoYDeg * 10;

    //printf("servoXDeg: %d - ", a);
    //printf("servoYDeg: %d\n", b);
}

void updateServoPos() {
    auto servoXPwm = 0.015f;
    auto servoYPwm = 0.015f;

    servoXPwm = (servoXDeg / 180.0f) * 0.001f + 0.001f;
    servoYPwm = (servoYDeg / 180.0f) * 0.001f + 0.001f;

    if(servoXDeg >= 175.0) {
        servoXPwm = 0.00195f;
    }
    if(servoXDeg <= 5.0) {
        servoXPwm = 0.00105f;
    }

    if(servoYDeg >= 175.0) {
        servoYPwm = 0.00195f;
    }
    if(servoYDeg <= 5.0) {
        servoYPwm = 0.00105f;
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
