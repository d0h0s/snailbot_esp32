#ifndef STANDARDKALMANFILTER_H
#define STANDARDKALMANFILTER_H

class KalmanFilter {
public:
    KalmanFilter(double processNoise, double measurementNoise, double estimatedError);
    double update(double measurement);
private:
    double q; // Process noise covariance
    double r; // Measurement noise covariance
    double p; // Estimated error covariance
    double x; // Value
    double k; // Kalman gain
};

#endif