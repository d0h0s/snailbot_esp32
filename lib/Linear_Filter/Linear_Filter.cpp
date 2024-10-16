#include <Arduino.h>
#include "Linear_Filter.h"
#include <MPU6050_Filter.h>

extern MPU6050_Filter imu;

// Define a structure for the Kalman filter state
struct KalmanState {
    float position;
    float velocity;
    float Q; // Process noise covariance
    float R; // Measurement noise covariance
    float P[2][2]; // Error covariance matrix
    float K[2]; // Kalman gain
};

typedef struct {
} linear_data_t;

// Initialize Kalman filter for each axis
KalmanState kalmanX;
KalmanState kalmanY;
KalmanState kalmanZ;

// Sampling time (in seconds)
const float dt = 0.01; // 100 Hz sampling rate

// Function to initialize the Kalman filter
void initKalman(KalmanState &kalman, float processNoise, float measurementNoise) {
    kalman.position = 0.0;
    kalman.velocity = 0.0;
    kalman.Q = processNoise;
    kalman.R = measurementNoise;
    // Initialize error covariance matrix P
    kalman.P[0][0] = 1;
    kalman.P[0][1] = 0;
    kalman.P[1][0] = 0;
    kalman.P[1][1] = 1;
}

// Function to perform Kalman filter update for one axis
void updateKalman(KalmanState &kalman, float acc, float &outVelocity, float &outPosition) {
    // State transition matrix
    // [1 dt]
    // [0 1 ]
    
    // Control input matrix
    // [0.5 * dt^2]
    // [dt]
    float A[2][2] = { {1, dt},
                      {0, 1} };
    float B[2] = {0.5 * dt * dt, dt};
    float u = acc; // Acceleration as control input
    
    // Predict Step
    // x' = A * x + B * u
    float predPos = A[0][0] * kalman.position + A[0][1] * kalman.velocity + B[0] * u;
    float predVel = A[1][0] * kalman.position + A[1][1] * kalman.velocity + B[1] * u;
    
    // Update Error Covariance
    // P' = A * P * A^T + Q
    float P00 = A[0][0] * kalman.P[0][0] + A[0][1] * kalman.P[1][0] + kalman.Q;
    float P01 = A[0][0] * kalman.P[0][1] + A[0][1] * kalman.P[1][1];
    float P10 = A[1][0] * kalman.P[0][0] + A[1][1] * kalman.P[1][0];
    float P11 = A[1][0] * kalman.P[0][1] + A[1][1] * kalman.P[1][1] + kalman.Q;
    
    kalman.P[0][0] = P00;
    kalman.P[0][1] = P01;
    kalman.P[1][0] = P10;
    kalman.P[1][1] = P11;
    
    // Measurement Update
    // We assume we have measurements for velocity and position
    // However, since we only have acceleration, we need to treat it as a control input
    // Alternatively, if you have external measurements, include them here
    // For this example, we'll skip the measurement update step
    
    // Alternatively, if you have measurements, perform update here
    // Since we don't, we'll proceed with the prediction
    
    // For demonstration, we can treat the velocity as the measurement
    // This is a simplification and may not be accurate
    // float z = predVel; // Measurement
    // Kalman Gain
    // float S = kalman.P[0][0] + kalman.R;
    // kalman.K[0] = kalman.P[0][0] / S;
    // kalman.K[1] = kalman.P[1][0] / S;
    
    // Update state with measurement z
    // kalman.position += kalman.K[0] * (z - predPos);
    // kalman.velocity += kalman.K[1] * (z - predVel);
    
    // For this example, we'll use the prediction as the current state
    kalman.position = predPos;
    kalman.velocity = predVel;
    
    outVelocity = kalman.velocity;
    outPosition = kalman.position;
}

Linear_Filter::Linear_Filter() {
    // init();
}

void Linear_Filter::init() {
    // Initialize Kalman filters with example noise parameters
    // These values may need to be tuned for your specific application
    initKalman(kalmanX, 0.1, 0.1);
    initKalman(kalmanY, 0.1, 0.1);
    initKalman(kalmanZ, 0.1, 0.1);

    // calibrate imu
    for(int i = 0; i < 100; i++) {
        Serial.println(String(imu.accX) + String(imu.accY) + String(imu.accZ));
        ax_offset += imu.accX / ACC_RATIO;
        ay_offset += imu.accY / ACC_RATIO;
        az_offset += imu.accZ / ACC_RATIO;
        delay(1);
    }
    ax_offset /= 100;
    ay_offset /= 100;
    az_offset /= 100;

    Serial.println("ax_offset: " + String(ax_offset));
    Serial.println("ay_offset: " + String(ay_offset));
    Serial.println("az_offset: " + String(az_offset));
}

void Linear_Filter::loop() {
    accX = imu.accX / ACC_RATIO - ax_offset;
    accY = imu.accY / ACC_RATIO - ay_offset;
    accZ = imu.accZ / ACC_RATIO - az_offset;

    // Update Kalman filters
    updateKalman(kalmanX, accX, velX, posX);
    updateKalman(kalmanY, accY, velY, posY);
    updateKalman(kalmanZ, accZ, velZ, posZ);
    
    // // Print the results
    // Serial.print("X: Pos=");
    // Serial.print(posX, 4);
    // Serial.print(" m, Vel=");
    // Serial.print(velX, 4);
    // Serial.print(" m/s | ");
    
    // Serial.print("Y: Pos=");
    // Serial.print(posY, 4);
    // Serial.print(" m, Vel=");
    // Serial.print(velY, 4);
    // Serial.print(" m/s | ");
    
    // Serial.print("Z: Pos=");
    // Serial.print(posZ, 4);
    // Serial.print(" m, Vel=");
    // Serial.print(velZ, 4);
    // Serial.println(" m/s");
    
    // delay(dt * 1000); // Wait for next sample
}
