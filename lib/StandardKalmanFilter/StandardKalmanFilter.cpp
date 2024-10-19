#include "StandardKalmanFilter.h"
KalmanFilter::KalmanFilter(double processNoise, double measurementNoise, double estimatedError)
    : q(processNoise), r(measurementNoise), p(estimatedError), x(0) {}
double KalmanFilter::update(double measurement)
{
    // Prediction update
    p += q;

    // Measurement update
    k = p / (p + r);
    x += k * (measurement - x);
    p *= (1 - k);
    
    return x;
}