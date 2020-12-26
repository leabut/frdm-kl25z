/*
 * main.cpp
 *
 *  Created on: 05.09.2020
 *      Author: spk
 */
#include <cmath>
#include <cstring>

#include "esp_log.h"
#include "sdkconfig.h"

#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Servo2.h>

// configurable values
#define CONTROL_RATE 50

double Kp = 0.50;
double ServoXOffset = 0.0;
double ServoYOffset = 0.0;

#include "main.h"

#define RAD_TO_DEG (360.0f / (2 * 3.141f))

float AccX = 0.0f, AccY = 0.0f, AccZ = 0.0f;
double Pitch = 0.0, Yaw = 0.0;
double ServoXDeg = 0.0, ServoYDeg = 0.0;
double AngleXZAxis = 0.0, AngleYZAxis = 0.0;

MPU6050* mpu6050 = nullptr;
Servo* servoX = nullptr;
Servo2* servoY = nullptr;

SemaphoreHandle_t print_mux = NULL;

static void main_task(void* arg) {
  uint32_t task_idx = (uint32_t)arg;

  static MPU6050 _mpu6050{MPU6050_DEFAULT_ADDRESS};
  mpu6050 = &_mpu6050;
  static Servo _servoX{18};
  servoX = &_servoX;
  static Servo2 _servoY{19};
  servoY = &_servoY;

  getAccData();
  getConstructionAngles();

  while (1) {
    getAccData();

    transformAccData();

    accel_degrees();
    callPidController();
    updateServoPos();

    printf("*******************\n");
    printf("TASK[%d]  MASTER READ SENSOR( MPU6050 )\n", task_idx);
    printf("*******************\n");
    int accX = AccX;
    int accY = AccY;
    int accZ = AccZ;
    printf("accX: %d - accY: %d - accZ: %d\n", accX, accY, accZ);
    vTaskDelay(CONTROL_RATE / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

extern "C" {
void app_main(void) {
  print_mux = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(i2c_init());
  xTaskCreate(main_task, "i2c_test_task_0", 1024 * 2, (void*)0, 10, NULL);
}
}

void getConstructionAngles() {
  double vec[3] = {AccX, AccY, AccZ};
  AngleXZAxis = atan(vec[1] / vec[2]);
  rotateXAxis(vec, AngleXZAxis);
  AngleYZAxis = atan(-vec[0] / vec[2]);
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
  AccX = (mpu6050->getAccelerationX() * 1000.0f) / 2048.0f;
  AccY = (mpu6050->getAccelerationY() * 1000.0f) / 2048.0f;
  AccZ = (mpu6050->getAccelerationZ() * 1000.0f) / 2048.0f;
}

void accel_degrees() {
  Yaw = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG;
  Pitch = atan(-AccX / sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;
}

void callPidController() {
  ServoXDeg = 90 + Kp * Pitch + ServoXOffset;
  ServoYDeg = 90 + Kp * Yaw + ServoYOffset;
}

void updateServoPos() {
  uint32_t servoXAngleInt = ServoXDeg;
  __attribute__((unused)) uint32_t servoYAngleInt = ServoYDeg;

  if (ServoXDeg >= 110.0) {
    servoXAngleInt = 110;
  }
  if (ServoXDeg <= 40.0) {
    servoXAngleInt = 40;
  }

  if (ServoYDeg >= 110.0) {
    servoYAngleInt = 110;
  }
  if (ServoYDeg <= 40.0) {
    servoYAngleInt = 40;
  }

  servoX->setAngle(servoXAngleInt);
  servoY->setAngle(servoYAngleInt);
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
