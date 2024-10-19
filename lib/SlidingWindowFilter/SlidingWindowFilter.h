#include <Arduino.h>
#include <cstddef>

class SlidingWindowFilter
{
public:
    SlidingWindowFilter();
    SlidingWindowFilter(int windowSize);
    void setWindowSize(int windowSize);
    float getAverage();
    void addValue(float value);
    int getWindowSize();
    float* getWindow();
private:
    int windowSize;
    float* window;
    int windowIndex;
    float windowSum;
};