#ifndef HIGHPASSFILTER_H
#define HIGHPASSFILTER_H

class HighPassFilter {
public:
    HighPassFilter(float alpha) : alpha(alpha), previousValue(0.0f), filteredValue(0.0f) {}

    float update(float rawValue) {
        filteredValue = alpha * (filteredValue + rawValue - previousValue);
        previousValue = rawValue;
        return filteredValue;
    }

    void reset(float initialValue) {
        filteredValue = initialValue;
        previousValue = initialValue;
    }

private:
    float alpha;          // Smoothing factor (0 < alpha < 1)
    float previousValue;  // Previous raw value
    float filteredValue;  // Current filtered value
};

#endif // HIGHPASSFILTER_H
