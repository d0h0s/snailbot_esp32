#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter {
public:
    LowPassFilter(float alpha) : alpha(alpha), filteredValue(0.0f) {}

    float update(float rawValue) {
        filteredValue = alpha * filteredValue + (1.0f - alpha) * rawValue;
        return filteredValue;
    }

    void reset(float initialValue) {
        filteredValue = initialValue;
    }

private:
    float alpha;          // Smoothing factor (0 < alpha < 1)
    float filteredValue;  // Current filtered value
};

#endif // LOWPASSFILTER_H
