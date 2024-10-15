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

    float wx;
    float wy;
    float wz;

    float s_left;
    float s_right;

    float v_left;
    float v_right;

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

extern MPU6050_Filter imu;

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

void imu_solve(chassis_t* chassis, MPU6050_Filter imu) {
    chassis->ax = imu.accX;
    chassis->ay = imu.accY;
    chassis->az = imu.accZ;

    chassis->wx = imu.gyroXrate;
    chassis->wy = imu.gyroYrate;
    chassis->wz = imu.gyroZrate;

    float w = imu.q[0];
    float x = imu.q[1];
    float y = imu.q[2];
    float z = imu.q[3];
    chassis->roll = atan2(2.0f * (w * y + x * z), 1.0f - 2.0f * (y * y + x * x));
    chassis->pitch = asin(2.0f * (w * z - y * x));
    chassis->yaw = atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (z * z + x * x));

    chassis->vx += chassis->ax * dt;
    chassis->vy += chassis->ay * dt;
    chassis->vz += chassis->az * dt;

    chassis->x += chassis->vx * dt;
    chassis->y += chassis->vy * dt;
    chassis->z += chassis->vz * dt;
}

void encoder_solve(chassis_t* chassis, DC_Motor l_motor, DC_Motor r_motor) {
    chassis->v_left = l_motor.get_speed();
    chassis->v_right = r_motor.get_speed();

    chassis->s_left += chassis->v_left * dt;
    chassis->s_right += chassis->v_right * dt;
}

void fd_kinematic(float v, float wz, float* v_left, float* v_right) {
    *v_left = (v - (wz * WHEEL_DISTANCE) / 2) * GEAR_RATIO / WHEEL_RADIUS / 1000;
    *v_right = (v + (wz * WHEEL_DISTANCE) / 2) * GEAR_RATIO / WHEEL_RADIUS / 1000;
}

void bk_kinematic(float v_left, float v_right, float* v, float* wz) {
    *v = (v_left + v_right) / 2;
    *wz = (v_right - v_left) / WHEEL_DISTANCE;
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
    send_packet.vx = (int16_t)chassis->vx;
    send_packet.vy = (int16_t)chassis->vy;
    send_packet.vz = (int16_t)chassis->vz;
    send_packet.ax = (int16_t)chassis->ax;
    send_packet.ay = (int16_t)chassis->ay;
    send_packet.az = (int16_t)chassis->az;
    send_packet.wx = (int16_t)chassis->wx;
    send_packet.wy = (int16_t)chassis->wy;
    send_packet.wz = (int16_t)chassis->wz;
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
}

void serial_setup(void)
{
    Serial.println("Start serial setup");
    // start_calibrate_imu = true;

    // chassis
    chassis = (chassis_t*)calloc(1, sizeof(chassis_t));
    chassis->last_tick = millis();
    strut_controller.base2_driver->set_speed_pid(0.1, 0.0, 0.0);
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
                            strut_controller.set_target_velocity(true, recv_packet.vx, recv_packet.wz, millis());
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
} 

