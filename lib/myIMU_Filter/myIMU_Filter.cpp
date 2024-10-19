#include "myIMU_Filter.h"
#ifndef PI
#define PI 3.1415926535897932384626
#endif
#ifndef g
#define g 9.7803
#endif

double normalize(double angle){
    if(angle > PI){
        return angle - 2 * PI;
    }
    if(angle < -PI){
        return angle + 2 * PI;
    }
    return angle;
}

void myIMU_Filter::getQuaternion() {
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

myIMU_Filter::myIMU_Filter(uint8_t addr)
    : lpfAx(0.1),
      lpfAy(0.1),
      lpfAz(0.1),
      kalmanGx(0.01, 5.0, 0.5),
      kalmanGy(0.01, 5.0, 0.5),
      kalmanGz(0.01, 5.0, 0.5),
      accelgyro(addr) {
    // Constructor
}

void myIMU_Filter::updateIMU(bool useBias) {
    // Read raw data from the MPU6050
    accelgyro.getMotion6(&ay_raw, &ax_raw, &az_raw, &gy_raw, &gx_raw, &gz_raw); // Reference to Snailbot

    // Convert raw data to float
    ax = ax_raw / 16384.0f; // Assuming ±2g     // In m/s^2
    ay = ay_raw / 16384.0f;
    az = az_raw / 16384.0f;

    az += g;

    gx = gx_raw / 131.0f / 180.0f;    // Assuming ±250°/s   // In rad/s
    gy = gy_raw / 131.0f / 180.0f;
    gz = gz_raw / 131.0f / 180.0f;

    // Apply bias correction
    if(useBias){
        ax -= abiasx;
        ay -= abiasy;
        az -= abiasz;
        gx -= gbiasx;
        gy -= gbiasy;
        gz -= gbiasz;
    }

    // Use filter to get filtered values
    ax = lpfAx.update(ax);
    ay = lpfAy.update(ay);
    az = lpfAz.update(az);
    gx = kalmanGx.update(gx);
    gy = kalmanGy.update(gy);
    gz = kalmanGz.update(gz);

    // Serial.println(">ax:" + String(ax));
    // Serial.println(">ay:" + String(ay));
    // Serial.println(">az:" + String(az));
}

void myIMU_Filter::init() {
    accelgyro.initialize();
    accelgyro.setRate(7); // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    accelgyro.setExternalFrameSync(0); // Disable FSYNC
    accelgyro.setDLPFMode(0); // Set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // Set Gyro Full Scale Range to ±250deg/s
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // Set Accelerometer Full Scale Range to ±2g
    calibrate();
}

void myIMU_Filter::calibrate() {
    in_calib = 1;   // Stop integration while calibration
#define CALIB_ROUND 1000
    for(int i = 0; i < CALIB_ROUND; i++){   // 1000 round avg
        updateIMU(0);
        abiasx += ax / CALIB_ROUND;
        abiasy += ay / CALIB_ROUND;
        abiasz += az / CALIB_ROUND;
        gbiasx += gx / CALIB_ROUND;
        gbiasy += gy / CALIB_ROUND;
        gbiasz += gz / CALIB_ROUND;
    }
    gbiasz += g;    // Add gravity to the bias
    in_calib = 0;
}

void myIMU_Filter::gyro_loop(){
    // Integrate gyro speed
    if(!in_calib){
        roll += gx * dt;
        pitch += gy * dt;
        yaw += gz * dt;
        // roll = GxToRoll.update(gx, dt);
        // pitch = GyToPitch.update(gy, dt);
        // yaw = GzToYaw.update(gz, dt);
        Serial.println("loop");
        Serial.println(">roll:" + String(roll));
        Serial.println(">pitch:" + String(pitch));
        Serial.println(">yaw:" + String(yaw));
    }
    
    roll = normalize(roll);
    pitch = normalize(pitch);
    yaw = normalize(yaw);

    getQuaternion();
}

void myIMU_Filter::acc_loop(){
    // Calculate the gravity vector in the body frame
    float ax_gravity = g * sin(pitch);
    float ay_gravity = -g * sin(roll) * cos(pitch);
    float az_gravity = -g * cos(roll) * cos(pitch);

    // Subtract gravity from acceleration
    ax -= ax_gravity;
    ay -= ay_gravity;
    az -= az_gravity;

    // Integrate acceleration to get velocity and position
    if(!in_calib){
        // vx += ax * dt;
        // vy += ay * dt;
        // vz += az * dt;
        vx = AxToVx.update(ax, dt);
        vz = AyToVy.update(ay, dt);
        vz = AzToVz.update(az, dt);

        // Serial.println(">ax:" + String(ax));
        // Serial.println(">ay:" + String(ay));
        // Serial.println(">az:" + String(az));
        // Serial.println(">vx:" + String(vx));
        // Serial.println(">vy:" + String(vy));
        // Serial.println(">vz:" + String(vz));

        // posX += vx * dt;
        // posY += vy * dt;
        // posZ += vz * dt;
        posX = VxToPx.update(vx, dt);
        posY = VyToPy.update(vy, dt);
        posZ = VzToPz.update(vz, dt);
    }
}

void myIMU_Filter::loop() {
    // Main loop function
    updateIMU(1);

    dt = (micros() - last_tick) / 1000000;

    gyro_loop();
    acc_loop();

    last_tick = micros();
}