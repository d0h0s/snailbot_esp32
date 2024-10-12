/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "Kalman.h"
#include <Arduino.h>

void limit_angle_2pi(float* angle)
{
    if (*angle >  PI) *angle -= 2.f * PI;
    if (*angle < -PI) *angle += 2.f * PI;
}

Kalman::Kalman() {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = 0.0001f;

    angle = 0.0f; // Reset the angle
    P= 0.0f; 

};

float Kalman::predict(float newDelta, float dt) 
{
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    angle += newDelta;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P += dt * Q_angle;

    // make angle belongs to [-PI, PI]
    limit_angle_2pi(&angle);

    return angle;
}

float Kalman::update(float newAngle, float R_measure) 
{

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P + R_measure; // Estimate error
    /* Step 5 */
    float K; // Kalman gain - This is a 2x1 vector
    K = P / S;
    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference

    limit_angle_2pi(&y);

    /* Step 6 */
    angle += K * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */

    P *= 1 - K;

    // make angle belongs to [-PI, PI]
    limit_angle_2pi(&angle);
    
    return angle;
}

void Kalman::setAngle(float angle) { this->angle = angle; P=0.0f; }; // Used to set angle, this should be set as the starting angle

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getAngle() { return angle; };

/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };

float Kalman::getQangle() { return this->Q_angle; };
