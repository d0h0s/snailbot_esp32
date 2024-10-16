#ifndef MPU6050_FILTER_H
#define MPU6050_FILTER_H

#include <MPU6050.h>
#include <EEPROM.h>
#include <ESP32Encoder.h>
#include <deque>
#include "Kalman.h"
#include <InterruptEncoder.h>

#define IMU_CALIBRATE_OK 67

#define CALIB_SERIAL_DEBUG
#define CALIB_BUFFER_SIZE 1000    //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
#define CALIB_ACEl_DEADZONE 8     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
#define CALIB_GIRO_DEADZONE 3     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

#define SENSOR_ERROR_FROM_DEG(angle) (sqrt(3.0f / 4.0f) * PI * (angle / 180.0f))

#define ENCODER_ALPHA 0.17151f
#define ENCODER_BETA 0.36727f
#define MOTOR_RATIO_RAD (2.f * PI / 1260.f / 14.f)

#define ALPHA_ESTIMATE_MZ_TOLERANCE_HARD 0.95
#define ALPHA_ESTIMATE_MZ_TOLERANCE_SOFT 0.9
#define ALPHA_ESTIMATE_MZ_TOLERANCE_REGION (1.f / (ALPHA_ESTIMATE_MZ_TOLERANCE_HARD - ALPHA_ESTIMATE_MZ_TOLERANCE_SOFT))
#define ALPHA_ESTIMATE_MZ_TOLERANCE_NORM_RATIO (ALPHA_ESTIMATE_MZ_TOLERANCE_REGION * ALPHA_ESTIMATE_MZ_TOLERANCE_REGION)

#define OPT_CHANGE_RATE_QUEUE_SIZE 20

enum FilterMode {FREE, LIFTER1, LIFTER2, BOTH};

struct GyrodomFrameCache {
    float gyro_integral = 0.f;
    float odom_integral = 0.f;
    float approx_integral = 0.f;
    float integral_t = 0.f;
    unsigned long gyro_end_time = 0;
    uint32_t gyro_frame_id = 0;
};

struct GyrodomCache {
    bool gyro_first_revieved = false;
    unsigned long gyro_start_time = 0;
    GyrodomFrameCache current;
    GyrodomFrameCache next;
    float gyro_revise = 0.f;
};

class MPU6050_Filter {
public:
    MPU6050_Filter(ESP32Encoder*, ESP32Encoder*, uint32_t, uint8_t addr=MPU6050_DEFAULT_ADDRESS);
    void init(void);
    void meansensors(int, int*, int*, int*, int*, int*, int*);
    void calibrate(void);
    void loop(void);
    void MadgwickQuaternionUpdate6(float ax, float ay, float az, float gx, float gy, float gz, float deltat);
    void MadgwickQuaternionUpdate9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float decay, float delta_ey, float delta_ez, float deltat, GyrodomCache &gyro_cache);
    //void set_base1_mag(float mx, float my, float mz);
    void set_base2_mag(float mx, float my, float mz);

    void set_node1_quat(float qw, float qx, float qy, float qz) {node1_q0=qw; node1_q1=qx; node1_q2=qy; node1_q3=qz;}
    void set_node2_quat(float qw, float qx, float qy, float qz) {node2_q0=qw; node2_q1=qx; node2_q2=qy; node2_q3=qz;}
    void get_node1_quat(float &qw, float &qx, float &qy, float &qz) {qw=node1_q0; qx=node1_q1; qy=node1_q2; qz=node1_q3;}
    void get_node2_quat(float &qw, float &qx, float &qy, float &qz) {qw=node2_q0; qx=node2_q1; qy=node2_q2; qz=node2_q3;}
    
    //void set_base1_gyro(uint32_t frame_id, float angle, float duration, int64_t t);
    void set_base2_gyro(uint32_t frame_id, float angle, float duration, int64_t t);
    void set_opt_result(float alpha, float vx, float vy, float vz);
    void update_mode(int, int);
    enum FilterMode get_mode(void) {return _mode;}
    float get_yaw_angle(void);
    void reset_timer(void);

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float accX, accY, accZ, dt;
    float gyroXrate, gyroYrate, gyroZrate;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gbiasx = 0.f, gbiasy=0.f, gbiasz=0.f;

private:
    MPU6050 accelgyro; 
    ESP32Encoder* _encoder_2l, * _encoder_2r;
    uint32_t eeprom_offset;

    //void update_base1_encoder(void);
    void update_base2_encoder(void);

    void estimate_alpha(void);
    void integrate_alpha(float delta_ey, float delta_ez, float dt, GyrodomCache &gyro_cache);
    void initial_yaw_guess(float mx, float my, float mz);
    void fast_gravity_align(float ax, float ay, float az);
    float gyro_revise_estimate(uint32_t frame_id,float angle, float duration, int64_t t, GyrodomCache &gyro_cache);
    void update_gyro_values(uint32_t frame_id, float duration, int64_t t, GyrodomCache &gyro_cache);
    void init_gyro_frame(uint32_t frame_id, float duration, int64_t t, GyrodomCache &gyro_cache);
    float gyrodom_cache_integral(float delta_ey, float delta_ez, float deltat, GyrodomCache &gyro_cache);
    float gyrodometry(float gz_, float gz_e);
    void clear_opt_buffers(void);
    
    unsigned long timer;
  
    float beta_static = SENSOR_ERROR_FROM_DEG(0.5f);
    float beta_dynamic = SENSOR_ERROR_FROM_DEG(5.0f); //0.0165
    float beta_init = SENSOR_ERROR_FROM_DEG(60.0f); //0.033
    float beta = beta_init;  // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3   
    float zeta = SENSOR_ERROR_FROM_DEG(0.2f);  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    
    unsigned long firstUpdate;
    bool stabled = false;
    bool fast_estimated = true;

    // strut-node gyro synchronize and gyrodometry
    //bool gyro_first_revieved = false;
    
    //float gyro_revise = 0.f;
   
    GyrodomCache base1_cache;
    GyrodomCache base2_cache;
    float gyroz_world;
    float gyrodom_world;

    // node-node yaw angle kalman filter
    Kalman yaw_anglle_filter;
    bool angle_filter_initialized = false;

    // orientation filter variables
    enum FilterMode _mode = FREE;
    float _m1x = 0.f, _m1y = 0.f, _m1z = 1.f;
    float _m2x = 0.f, _m2y = 0.f, _m2z = 1.f;
    float _optx, _opty, _optz, _opt_alpha;
    unsigned long _m1_t, _m2_t, _opt_t;

    float node1_q0, node1_q1, node1_q2, node1_q3;
    float node2_q0, node2_q1, node2_q2, node2_q3;

    std::deque<float> opt_change_rate;
    float opt_rate_sum = 0.f;
    float opt_rate_variance = 0.f;
    float opt_confidence = 0.f;

    int64_t base1_l_last_count, base1_r_last_count;
    int64_t base1_l_count, base1_r_count;
    int64_t base2_l_last_count, base2_r_last_count;
    int64_t base2_l_count, base2_r_count;
    
    float _delta_e1y=0.f, _delta_e1z=0.f;
    float _delta_e2y=0.f, _delta_e2z=0.f;

    float R11, R12, R13, R21, R22, R23, R31, R32, R33; // R^E_S rotation matrix
    
};

#endif