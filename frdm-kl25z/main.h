#ifndef MAIN_H
#define MAIN_H

void getAccData();
void visualizeAcc();
void printfAcc();
void accel_degrees();
void callPidController();
void updateServoPos(double servoXAngle, double servoYAngle);
void getConstructionAngles();
void transformAccData();

void rotateXAxis(double vec[3], double alpha);
void rotateYAxis(double vec[3], double alpha);

#endif /* MAIN_H */