#ifndef _LINEAR_FILTER_H
#define _LINEAR_FILTER_H

#define ACC_RATIO 208.973214

class Linear_Filter {
public:
    Linear_Filter();
    void init();
    void loop();

    float posX, posY, posZ;
    float velX, velY, velZ;
    float accX, accY, accZ;

private:
    float ax_offset, ay_offset, az_offset;
};

#endif