#include "SlidingWindowFilter.h"
SlidingWindowFilter::SlidingWindowFilter()
{
    windowSize = 0;
    window = nullptr;
    windowIndex = 0;
    windowSum = 0;
}
SlidingWindowFilter::SlidingWindowFilter(int windowSize)
{
    if(windowSize <= 0)
    {
        windowSize = 1;
    }
    this->windowSize = windowSize;
    window = (float*)calloc(windowSize, sizeof(float));
    windowIndex = 0;
    windowSum = 0;
}
void SlidingWindowFilter::setWindowSize(int windowSize)
{
    if (window != nullptr)
    {
        delete[] window;
    }
    this->windowSize = windowSize;
    window = (float*)calloc(windowSize, sizeof(float));
    windowIndex = 0;
    windowSum = 0;
}
float SlidingWindowFilter::getAverage()
{
    return windowSum / windowSize;
}
void SlidingWindowFilter::addValue(float value)
{
    windowSum -= window[windowIndex];
    window[windowIndex] = value;
    windowSum += value;
    windowIndex = (windowIndex + 1) % windowSize;
}
int SlidingWindowFilter::getWindowSize()
{
    return windowSize;
}
float* SlidingWindowFilter::getWindow()
{
    return window;
}