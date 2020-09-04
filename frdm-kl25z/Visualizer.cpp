#include "Visualizer.h"

#include "mbed.h"

namespace visualizer {

PwmOut rled(LED1);
PwmOut gled(LED2);
PwmOut bled(LED3);

void startupBlink() {
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
}

void printfAcc(float vec[3]) {
    int x = vec[0] * 1000u;
    int y = vec[1] * 1000u;
    int z = vec[2] * 1000u;
    printf("X: %d, Y: %d, Z: %d\n", x, y, z);
}

void visualizeAcc(float vec[3]) {
    rled = 1.0f - vec[0];
    gled = 1.0f - vec[1];
    bled = 1.0f - vec[2];
}

} /* namespace visualizer */