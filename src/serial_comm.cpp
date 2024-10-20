#include <Arduino.h>
// #include <AsyncTCP.h>
// #include <AsyncUDP.h>
// #include <HTTPClient.h>
// #include <ESPAsyncWebServer.h>
// #include <ElegantOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "freertos/semphr.h"
// #include "esp_sntp.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <MPU6050_Filter.h>
#include "Magnet_Lifter.h"
#include "Differential_Driver.h"
#include "common.h"
#include "gpio.h"

#include "HardwareSerial.h"
#include "Magnet_Lifter.h"
#include "common.h"
#include "eeprom_offset.h"
#include "Strut_Controller.h"

#include "DC_Motor.h"
#include "SlidingWindowFilter.h"

typedef struct{
    uint8_t header;
    uint8_t flag_stop;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t wx;
    int16_t wy;
    int16_t wz;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t posX;
    int16_t posY;
    int16_t th;
    uint8_t checksum;
} __attribute__((packed)) send_t;

typedef struct{
    uint8_t header;
    int16_t vx;
    int16_t vy;
    int16_t wz;
    uint8_t checksum;
} __attribute__((packed)) recv_t;

typedef struct{
    float x;
    float y;
    float z;

    float vx;
    float vy;
    float vz;

    float ax;
    float ay;
    float az;

    float roll;
    float pitch;
    float yaw;

    float q[4];

    float wx;
    float wy;
    float wz;

    float s_left;
    float s_right;

    float v_left;
    float v_right;

    float v_left_filtered;
    float v_right_filtered;

    float posX;
    float posY;
    float th;

    float last_tick;
} chassis_t;

// extern byte MAJOR_CODE, MINOR_CODE;
// extern int robot_id;
// extern int hardware_code;
// uint64_t robot_mac;
// uint8_t robot_mac_[14];

// extern HardwareSerial mySerial;

extern int lifter1_node_id, lifter2_node_id;
extern bool lifter1_node_connected, lifter2_node_connected;
extern unsigned long lifter1_connected_time, lifter2_connected_time;

extern MagnetLifter lifter1;
extern MagnetLifter lifter2;

extern DifferentialDriver base1_driver; 
extern DifferentialDriver base2_driver;

//extern MPU6050_Filter imu;
extern myIMU_Filter imu;

extern QueueHandle_t xSendKeyQueue;

// unsigned long ota_progress_millis = 0;

extern bool start_calibrate_imu;

extern StrutController strut_controller;

extern bool cmd_parse(char *cmd);

float dt;

// send_t* send_packet = nullptr;
// recv_t* recv_packet = nullptr;
chassis_t* chassis = nullptr;

extern DC_Motor motor_C;
extern DC_Motor motor_F;

char serial_cmd_copy[120];

#define GEAR_RATIO 1600
#define WHEEL_RADIUS 0.02
#define WHEEL_DISTANCE 0.08075

unsigned char check_sum(unsigned char Mode, uint8_t* data, size_t len) {
    unsigned char sum = 0, k;

    if (Mode == 1)  // Send
    for (k = 0; k < len; k++) {
        sum = sum ^ data[k];
    }

    if (Mode == 0)  // Recv
    for (k = 0; k < len; k++) {
        sum = sum ^ data[k];
    }
    return sum;
}

void quaternionToRPY(const float q[4], float &roll, float &pitch, float &yaw) {
    // Normalize the quaternion
    double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    double q0 = q[0] / norm;
    double q1 = q[1] / norm;
    double q2 = q[2] / norm;
    double q3 = q[3] / norm;

    // Compute roll (x-axis rotation)
    roll = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

    // Compute pitch (y-axis rotation)
    pitch = std::asin(2.0 * (q0 * q2 - q3 * q1));

    // Compute yaw (z-axis rotation)
    yaw = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
}

// SlidingWindowFilter roll_filter(20);
// SlidingWindowFilter pitch_filter(20);
// SlidingWindowFilter yaw_filter(20);

void imu_solve(chassis_t* chassis, myIMU_Filter imu) {
    chassis->ax = imu.ax;
    chassis->ay = imu.ay;
    chassis->az = imu.az;

    chassis->wx = imu.gx;
    chassis->wy = imu.gy;
    chassis->wz = imu.gz;

    chassis->roll = imu.roll;
    chassis->pitch = imu.pitch;
    chassis->yaw = imu.yaw;

    chassis->vx += chassis->ax * dt;
    chassis->vy += chassis->ay * dt;
    chassis->vz += chassis->az * dt;

    // chassis->x += chassis->vx * dt;
    // chassis->y += chassis->vy * dt;
    // chassis->z += chassis->vz * dt;
}

void fd_kinematic(float v, float wz, float* v_left, float* v_right) {
    *v_left = (v - (wz * WHEEL_DISTANCE) / 2) * GEAR_RATIO / WHEEL_RADIUS / 1000;
    *v_right = (v + (wz * WHEEL_DISTANCE) / 2) * GEAR_RATIO / WHEEL_RADIUS / 1000;
}

void bk_kinematic(float v_left, float v_right, float* v, float* wz) {
    *v = (v_left + v_right) / 2;
    *wz = (v_right - v_left) / WHEEL_DISTANCE;
}

SlidingWindowFilter v_left_filter(20);
SlidingWindowFilter v_right_filter(20);

void encoder_solve(chassis_t* chassis, double motor_l_real_speed, double motor_r_real_speed) {
    double dt = (micros() - chassis->last_tick) / 1000000.0f;

    chassis->v_left = 2.236f * motor_l_real_speed;
    chassis->v_right = 2.236f * motor_r_real_speed;

    v_left_filter.addValue(chassis->v_left);
    v_right_filter.addValue(chassis->v_right);

    chassis->v_left_filtered = v_left_filter.getAverage();
    chassis->v_right_filtered = v_right_filter.getAverage();

    chassis->s_left += chassis->v_left_filtered * dt;
    chassis->s_right += chassis->v_right_filtered * dt;

    bk_kinematic(chassis->v_left_filtered, chassis->v_right_filtered, &chassis->vx, &chassis->wz);
    // Serial.println(">v_left_filtered:" + String(chassis->v_left_filtered));
    // Serial.println(">v_right_filtered:" + String(chassis->v_right_filtered));
    // Serial.println(">solved_vx:" + String(chassis->vx));
    // Serial.println(">solved_wz:" + String(chassis->wz));
    chassis->posX += chassis->vx * cos(chassis->yaw) * dt;
    chassis->posY += chassis->vx * sin(chassis->yaw) * dt;
    chassis->th += chassis->wz * dt;

    chassis->last_tick = micros();
}

// void cmd_control(chassis_t* chassis, recv_t* recv_packet) {
//     float expect_v_left, expect_v_right;
//     fd_kinematic(recv_packet->vx, recv_packet->wz, &expect_v_left, &expect_v_right);
//     Serial.println("Try set vel");
//     strut_controller.set_target_velocity(true, expect_v_left, expect_v_right);
// }

void send() {
    // Serial.println("Start send");
    send_t send_packet;
    send_packet.header = 0x7B;
    send_packet.flag_stop = 0;
    send_packet.vx = imu.vx * 10000;  //chassis->vx;
    send_packet.vy = imu.vy * 10000;
    send_packet.vz = imu.vz * 10000;
    send_packet.ax = imu.ax * 10000;
    send_packet.ay = imu.ay * 10000;
    send_packet.az = imu.az * 10000;
    send_packet.wx = imu.gx * 10000;
    send_packet.wy = imu.gy * 10000;
    send_packet.wz = imu.gz * 10000;
    send_packet.roll = imu.roll * 10000;
    send_packet.pitch = -imu.pitch * 10000;
    send_packet.yaw = imu.yaw * 10000;

    Serial.println("yaw: " + String(imu.yaw));

    send_packet.posX = chassis->x * 10000;
    send_packet.posY = chassis->y * 10000;
    send_packet.th = chassis->th * 10000;
    // Serial.println("Initialized send_packet");

    send_packet.checksum = check_sum(1, (uint8_t*)&send_packet, sizeof(send_t) - 1);

    Serial2.write((uint8_t*)&send_packet, sizeof(send_t));
}

void test_i2c() {
    ;
}

void print_debug() {
    // imu
    // Serial.println("Acc: " + String(imu.accX) + ", " + String(imu.accY) + ", " + String(imu.accZ));
    // Serial.println("Gyro: " + String(imu.gyroXrate) + ", " + String(imu.gyroYrate) + ", " + String(imu.gyroZrate));

    // Serial.println("loop");
    // Serial.println(">v_left:" + String(chassis->v_left));
    // Serial.println(">v_right:" + String(chassis->v_right));
    // Serial.println(">v_left_filtered:" + String(chassis->v_left_filtered));
    // Serial.println(">v_right_filtered:" + String(chassis->v_right_filtered));
    // Serial.println(">posX:" + String(chassis->posX));
    // Serial.println(">posY:" + String(chassis->posY));
    // Serial.println(">roll:" + String(imu.roll));
    // Serial.println(">pitch:" + String(imu.pitch));
    // Serial.println(">yaw:" + String(imu.yaw));
    // Serial.println(">wx:" + String(imu.gx));
    // Serial.println(">wy:" + String(imu.gy));
    // Serial.println(">wz:" + String(imu.gz));
    // Serial.println(">vx:" + String(imu.vx));
    // Serial.println(">vy:" + String(imu.vy));
    // Serial.println(">vz:" + String(imu.vz));
    // Serial.println(">ax:" + String(imu.ax));
    // Serial.println(">ay:" + String(imu.ay));
    // Serial.println(">az:" + String(imu.az));
}

void serial_setup(void)
{
    Serial.println("setup");
    // start_calibrate_imu = true;
    // chassis
    chassis = (chassis_t*)calloc(1, sizeof(chassis_t));
    chassis->last_tick = micros();
    strut_controller.base2_driver->set_speed_pid(2, 0.0, 0.0);
}
void serial_loop(void)
{
    static uint8_t buffer[sizeof(recv_t)];  // 定义静态缓冲区用于存储接收到的数据
    static int buffer_index = 0;            // 缓冲区索引
    static int16_t last_vx = 0;  // 保存上次的 vx 值
    static int16_t last_wz = 0;  // 保存上次的 wz 值
    recv_t recv_packet;

    // 如果串口有可用数据
    if (Serial2.available() > 0) {
        // 逐字节接收数据
        while (Serial2.available() > 0) {
            uint8_t byte = Serial2.read();

            // 找到帧头 0x7A
            if (buffer_index == 0 && byte == 0x7A) {
                buffer[buffer_index++] = byte;  // 将帧头存入缓冲区
            }
            else if (buffer_index > 0) {
                buffer[buffer_index++] = byte;  // 将后续数据存入缓冲区

                // 如果接收到了完整的数据包
                if (buffer_index == sizeof(recv_t)) {
                    memcpy(&recv_packet, buffer, sizeof(recv_t));  // 复制数据包

                    // 校验和校验
                    uint8_t calculated_checksum = check_sum(0, (uint8_t*)&recv_packet, sizeof(recv_t) - 1);
                    // Serial.print("Calculated checksum: 0x");
                    // Serial.println(calculated_checksum, HEX);

                    // Serial.print("recv_packet.checksum: 0x");
                    // Serial.println(recv_packet.checksum, HEX);

                    if (recv_packet.checksum == calculated_checksum) {
                        // 如果校验和正确，打印出接收到的数据
                        Serial.println(">vx:" + String(recv_packet.vx));
                        Serial.println(">wz:" + String(recv_packet.wz));
                        // Serial.println("Checksum is correct");
                        // Serial.print("recv_packet.vx:");
                        // Serial.println(recv_packet.vx);
                        // Serial.print("recv_packet.vy:");
                        // Serial.println(recv_packet.vy);
                        // Serial.print("recv_packet.wz:");
                        // Serial.println(recv_packet.wz);        

                        if (recv_packet.vx != last_vx || recv_packet.wz != last_wz) 
                        {
                            // 调用函数处理接收到的指令
                            float expect_v_left, expect_v_right;
                            // fd_kinematic(recv_packet.vx, recv_packet.wz, &expect_v_left, &expect_v_right);
                            strut_controller.set_target_velocity(true, recv_packet.vx/=1.5, recv_packet.wz/2.0, millis());
                            // 更新上次的速度指令
                            last_vx = recv_packet.vx;
                            last_wz = recv_packet.wz;
                        }

                    } else {
                        // 校验和失败
                        Serial.println("Checksum is incorrect, skipping this packet");
                    }

                    // 重置缓冲区索引，准备接收下一帧数据
                    buffer_index = 0;
                }
            }
        }
    }
    encoder_solve(chassis, strut_controller.base2_driver->motor_l_real_speed, strut_controller.base2_driver->motor_l_real_speed);
    imu_solve(chassis, imu);
    print_debug();
    send();
} 

