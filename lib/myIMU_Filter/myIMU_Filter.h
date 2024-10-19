#ifndef MYIMU_FILTER_H
#define MYIMU_FILTER_H

#include <Arduino.h>
#include <MPU6050.h>
#include "StandardKalmanFilter.h"
#include "LowPassFilter.h"
#include "HighPassFilter.h"
#include "simpson.h"
#include <cmath>

class myIMU_Filter {
public:
    myIMU_Filter(uint8_t addr=MPU6050_DEFAULT_ADDRESS);
    void init(void);
    void calibrate(void);
    void loop(void);

    double ax, ay, az;
    double gx, gy, gz;
    double roll, pitch, yaw;
    double vx, vy, vz;
    double posX, posY, posZ;
    double q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

private:
    MPU6050 accelgyro; 
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    double abiasx = 0.f, abiasy = 0.f, abiasz = 0.f;
    double gbiasx = 0.f, gbiasy=0.f, gbiasz=0.f;
    double dt, last_tick;

    bool in_calib = 0;

    void updateIMU(bool);
    void gyro_loop();
    void acc_loop();
    void getQuaternion();

    LowPassFilter lpfAx;
    LowPassFilter lpfAy;
    LowPassFilter lpfAz;

    KalmanFilter kalmanGx;
    KalmanFilter kalmanGy;
    KalmanFilter kalmanGz;

    SimpsonIntegrator AxToVx;
    SimpsonIntegrator AyToVy;
    SimpsonIntegrator AzToVz;
    SimpsonIntegrator VxToPx;
    SimpsonIntegrator VyToPy;
    SimpsonIntegrator VzToPz;
    SimpsonIntegrator GxToRoll;
    SimpsonIntegrator GyToPitch;
    SimpsonIntegrator GzToYaw;
};

#endif