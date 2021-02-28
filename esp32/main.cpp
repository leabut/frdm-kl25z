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

double ServoXOffset = -20.0;
double ServoYOffset = -30.0;

double ServoXMinAngle = 40.0;
double ServoXMaxAngle = 165.0;
double ServoYMinAngle = 70.0;
double ServoYMaxAngle = 170.0;

#include "main.h"

#define RAD_TO_DEG (360.0f / (2 * 3.141f))

float AccX = 0.0f, AccY = 0.0f, AccZ = 0.0f;
double Pitch = 0.0, Yaw = 0.0;
double ServoXDeg = 90.0, ServoYDeg = 90.0;
double AngleXZAxis = 0.0, AngleYZAxis = 0.0;

MPU6050* mpu6050 = nullptr;
Servo* servoX = nullptr;
Servo2* servoY = nullptr;

SemaphoreHandle_t print_mux = NULL;

auto isFreeFall = false;
auto isParachuteOpen = false;

void fireParachute() {
  constexpr TickType_t timeout = 3000u; // 3000ms = 3sec
  static TickType_t timer = portMAX_DELAY;

  if(isFreeFall == false || isParachuteOpen) {
	  return;
  }

  if(isFreeFall && timer == portMAX_DELAY) {
    timer = xTaskGetTickCount() / pdMS_TO_TICKS(1u);
  }

  if((timer + timeout) < (xTaskGetTickCount() / pdMS_TO_TICKS(1u))) {
	printf("Fire parachute!\n");
	gpio_set_level(GPIO_NUM_15, 1);
	isParachuteOpen = true;
  }
}

void detectFreeFall() {
  constexpr TickType_t timeout = 300u; // 300ms
  static TickType_t timer = portMAX_DELAY;
  static auto positiveCounter = 0u;
  static auto negativeCounter = 0u;

  if(isFreeFall) {
	  fireParachute();
	  return;
  }

  if((std::abs(AccX) < 200u) && (std::abs(AccY) < 200u) && (std::abs(AccZ) < 200u)) {
	if(timer == portMAX_DELAY) {
		timer = xTaskGetTickCount() / pdMS_TO_TICKS(1u);
	}
	positiveCounter++;
  } else {
	negativeCounter++;
  }

  if(timer != portMAX_DELAY) {
    if((timer + timeout) < (xTaskGetTickCount() / pdMS_TO_TICKS(1u))) {
    double trustFactor = (double) positiveCounter / (positiveCounter + negativeCounter);
	  printf("trustFactor: %f\n", trustFactor);
	  if(trustFactor > 0.8) {
		isFreeFall = true;
		printf("Detected freeFall! %d / %d\n", positiveCounter, negativeCounter);
	  } else {
        positiveCounter = 0u;
	    negativeCounter = 1u;
	  }
	  timer = portMAX_DELAY;
    }
  }
}

static void main_task(void* arg) {

  gpio_reset_pin(GPIO_NUM_15);
  gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);

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

    int accX = AccX;
    int accY = AccY;
    int accZ = AccZ;
    //printf("%d,%d,%d\n", accX, accY, accZ);
    //vTaskDelay(CONTROL_RATE / portTICK_RATE_MS);

    detectFreeFall();
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
  Yaw *= -1;
}

void callPidController() {
  constexpr auto gainY = 1.4f;
  constexpr auto gainX = 1.4f;
  constexpr auto averageDerivate = 0.15;
  constexpr auto average = 0.15;

  auto derivateX = (90 + Pitch + ServoXOffset) * averageDerivate - (ServoXDeg/gainX) * (1.0f - averageDerivate);
  auto derivateY = (90 + Yaw + ServoYOffset) * averageDerivate - (ServoYDeg/gainY)* (1.0f - averageDerivate);

  ServoXDeg = (90 + Pitch + ServoXOffset) * average + (ServoXDeg/gainX) * (1.0f-average) + derivateX * 0.1;
  ServoXDeg *= gainX;
  ServoYDeg = (90 + Yaw + ServoYOffset) * average + (ServoYDeg/gainY) * (1.0f-average) + derivateY * 0.1;
  ServoYDeg *= gainY;
}

void updateServoPos() {
  uint32_t servoXAngleInt = ServoXDeg;
  uint32_t servoYAngleInt = ServoYDeg;

  auto upperLimitX = ServoXMaxAngle + ServoXOffset;
  auto lowerLimitX = ServoXMinAngle + ServoXOffset;
  auto upperLimitY = ServoYMaxAngle + ServoYOffset;
  auto lowerLimitY = ServoYMinAngle + ServoYOffset;

  if(upperLimitX >= 180.0) {
	  upperLimitX = 180.0;
  }
  if(lowerLimitX <= 0.0) {
	  lowerLimitX = 0.0;
  }
  if(upperLimitY >= 180.0) {
	  upperLimitY = 180.0;
  }
  if(lowerLimitY <= 0.0) {
	  lowerLimitY = 0.0;
  }

  if (ServoXDeg >= upperLimitX) {
    servoXAngleInt = upperLimitX;
  }
  if (ServoXDeg <= lowerLimitX) {
    servoXAngleInt = lowerLimitX;
  }

  if (ServoYDeg >= upperLimitY) {
    servoYAngleInt = upperLimitY;
  }
  if (ServoYDeg <= lowerLimitY) {
    servoYAngleInt = lowerLimitY;
  }

  servoX->setAngle(servoXAngleInt);
  servoY->setAngle(servoYAngleInt);

  servoXAngleInt -= ServoXOffset;
  servoYAngleInt -= ServoYOffset;

  //printf("servoXAngleInt: %d servoYAngleInt: %d\n", servoXAngleInt, servoYAngleInt);
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
