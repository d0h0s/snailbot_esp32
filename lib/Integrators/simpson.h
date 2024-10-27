#ifndef SIMPSON_INTEGRATOR_H
#define SIMPSON_INTEGRATOR_H

#include <vector>

class SimpsonIntegrator {
public:
    SimpsonIntegrator() : velocity(0.0f), prevAcc(0.0f), currentAcc(0.0f), dt(0.0f) {}

    float update(float newAcc, float dt) {
        // Update current acceleration
        currentAcc = newAcc;

        // Apply Simpson's rule for integration
        if (dt > 0.0f) {
            velocity += (prevAcc + 4.0f * currentAcc) / 6.0f * dt;
        }

        // Update previous acceleration
        prevAcc = currentAcc;

        return velocity;
    }

    void reset() {
        velocity = 0.0f;
        prevAcc = 0.0f;
        currentAcc = 0.0f;
    }

private:
    float velocity;      // Current integrated velocity
    float prevAcc;      // Previous acceleration reading
    float currentAcc;   // Current acceleration reading
    float dt;           // Time step
};

#endif // SIMPSON_INTEGRATOR_H
