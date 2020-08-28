/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "MMA8451Q.h"

#include "RCSwitch.h"

DigitalOut sendData(PTB8);

// Blinking rate in milliseconds
#define BLINKING_RATE 500ms
#define SLEEP_RATE 100ms
 
PinName ACC_SDA = PTE25;
PinName ACC_SCL = PTE24;
 
#define MMA8451_I2C_ADDRESS (0x1d<<1)
 
int main(void)
{
    //MMA8451Q acc(ACC_SDA, ACC_SCL, MMA8451_I2C_ADDRESS);
    PwmOut rled(LED1);
    PwmOut gled(LED2);
    PwmOut bled(LED3);
 
    //printf("MMA8451 ID: %d\n", acc.getWhoAmI());
 
    while (true) {
        float x, y, z;
        /*
        x = abs(acc.getAccX());
        y = abs(acc.getAccY());
        z = abs(acc.getAccZ());
        */
        rled = 1.0f - x;
        gled = 1.0f - y;
        bled = 1.0f - z;
        ThisThread::sleep_for(SLEEP_RATE);
        //printf("X: %1.2f, Y: %1.2f, Z: %1.2f\n", x, y, z);
    }
}
