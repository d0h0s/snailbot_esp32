#include "gpio.h"
#include "common.h"
#include "eeprom_offset.h"
#include <Arduino.h>
#include "FreeRTOSConfig.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"
#include <freertos/queue.h>
#include <ESPAsyncWebServer.h>

#include "DC_Motor.h"
#include <ESP32Encoder.h>
#include <InterruptEncoder.h>
#include "Differential_Driver.h"
#include "Strut_Controller.h"

#include <EEPROM.h>

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_Filter.h>
#include "Magnet_Lifter.h"

#include "soc/uart_periph.h"
#include "driver/uart.h"
#include "soc/gpio_sig_map.h"
#include "soc/uart_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "driver/gpio.h"

// #include <HardwareSerial.h>
// #include <SoftwareSerial.h>
// HardwareSerial mySerial(2);

// #include <FastLED.h>

#include <PID_v1.h>

//#define OLD_STRUT

//#define DEBUG_HEAP_SIZE
//#define DEBUG_CPU_USAGE

//version code, first verison with code 1.5
byte MAJOR_CODE = 1, MINOR_CODE = 6;
int robot_id = -1;
int hardware_code = HARDWARE_CODE_STRUT_PLUS;
extern uint8_t robot_mac_[14];

bool start_sleep_mode = false;
bool terminate_sleep_mode = false;
uint16_t sleep_ready_mask = 0;

extern AsyncWebSocket ws;

bool lifter1_node_connected = false, lifter2_node_connected = false;
int lifter1_node_id = -1, lifter2_node_id = -1;
double lifter1_event_time = 0., lifter2_event_time = 0.;

uint16_t lifter1_force, lifter2_force;
unsigned long lifter1_connected_time = 0, lifter2_connected_time = 0;

SemaphoreHandle_t xMutexLifter;
SemaphoreHandle_t xSerialSemaphore;
QueueHandle_t xSendKeyQueue;

DC_Motor motor_A(MCPWM_UNIT_1, MCPWM_TIMER_2, AIN1, AIN2);
DC_Motor motor_B(MCPWM_UNIT_0, MCPWM_TIMER_2, BIN1, BIN2);
DC_Motor motor_C(MCPWM_UNIT_1, MCPWM_TIMER_0, CIN2, CIN1);
DC_Motor motor_D(MCPWM_UNIT_0, MCPWM_TIMER_1, DIN2, DIN1);
DC_Motor motor_E(MCPWM_UNIT_0, MCPWM_TIMER_0, EIN2, EIN1);
DC_Motor motor_F(MCPWM_UNIT_1, MCPWM_TIMER_1, FIN2, FIN1);

// lifter motor use interrupt encoder since ESP32 contains only four PCNT units and it is more rarely used
ESP32Encoder encoder_A;
ESP32Encoder encoder_B; 
ESP32Encoder encoder_C;
// InterruptEncoder encoder_D;
// InterruptEncoder encoder_E;
ESP32Encoder encoder_F;

// MagnetLifter lifter1(&motor_B, &encoder_B, BLIM, BFRC, LIFTER1_EEPROM_OFFSET, FLAG_SLEEP_LIFTER1_READY); // upper
// MagnetLifter lifter2(&motor_A, &encoder_A, ALIM, AFRC, LIFTER2_EEPROM_OFFSET, FLAG_SLEEP_LIFTER2_READY); // lower
MagnetLifter lifter1(&motor_B, &encoder_B, BLIM, LIFTER1_EEPROM_OFFSET, FLAG_SLEEP_LIFTER1_READY); // upper
MagnetLifter lifter2(&motor_A, &encoder_A, ALIM, LIFTER2_EEPROM_OFFSET, FLAG_SLEEP_LIFTER2_READY); // lower

MPU6050_Filter imu(&encoder_C, &encoder_F, IMU_EEPROM_OFFSET, 0x69); 
bool start_calibrate_imu = false;

//DifferentialDriver base1_driver(&motor_E, &encoder_E, &motor_D, &encoder_D, BASE1_EEPROM_OFFSET); // upper
DifferentialDriver base2_driver(&motor_C, &encoder_C, &motor_F, &encoder_F, BASE2_EEPROM_OFFSET); // lower

StrutController strut_controller(&base2_driver, &imu, &xSendKeyQueue);


extern bool cmd_parse(char *cmd);
extern void network_setup(void);
extern void network_loop(void);
extern void serial_setup(void);
extern void serial_loop(void);

static void task_serial_comm(void *arg)
{
  serial_setup();
  Serial.println("Start serial comm task");

  while(1){
    serial_loop();
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

static void task_wifi_communication(void *arg)
{
  network_setup();

  while(1){
    network_loop();
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

static void task_kalman_filter(void *arg)
{
  vTaskDelay(2000 / portTICK_RATE_MS);

  imu.init();

#ifdef DEBUG_HEAP_SIZE
  unsigned long last_debug_heap_time = micros();
#endif

  while(1){

    if (start_calibrate_imu){
      imu.calibrate();
      start_calibrate_imu = false;

      // notify monitor 
      ws.textAll("notify imu_calib_done!\r\n");
    }
    vTaskDelay(1 / portTICK_RATE_MS);
    
    imu.loop();
  }
}  


void task_base_control(TimerHandle_t xTimer)
{
  while(1)
  {
    unsigned long start_time = micros();
    strut_controller.loop();

    unsigned long time_ellapsed = (unsigned long)(micros() - start_time) / 1000;
    if (time_ellapsed > 4) vTaskDelay(1/portTICK_RATE_MS);
    else vTaskDelay((4 - time_ellapsed) / portTICK_RATE_MS);
  }
}

static void task_lifter_control(void *arg)
{
  MagnetLifter* lifter = (MagnetLifter*) arg;

#ifdef DEBUG_HEAP_SIZE
  unsigned long last_debug_heap_time = micros();
#endif

  vTaskDelay(1000/portTICK_RATE_MS);

  lifter->initialize();

  uint32_t start_time;

  //lifter->set_target_position(-12.f);
  while (1)
  {
    if (start_sleep_mode) {
      if (terminate_sleep_mode) {
        sleep_ready_mask &= ~lifter->sleep_ready_flag;
      }
      else {
        if (!(sleep_ready_mask & lifter->sleep_ready_flag)){
          lifter->disable_magnetic_send(5000); 
          lifter->speed(0);
          sleep_ready_mask |= lifter->sleep_ready_flag;
        }
      }
      vTaskDelay(100/portTICK_RATE_MS);
      continue;
    }

    if (lifter->flag_start_calibrate)
    {
    //   lifter->force_calibrate();
      lifter->set_target_position(-12.f);
    }

    if (!lifter->is_initialized())
    {
      vTaskDelay(100/portTICK_RATE_MS);
      continue;
    }

    if (!lifter->reach_target) 
    {
      while(!lifter->reach_target) {
        if (!lifter->is_initialized()) break;
        lifter->loop();
        vTaskDelay(5/portTICK_RATE_MS);
      }
      lifter->speed(0);
      uint32_t key = lifter->clear_task_key();
      if (key != 0) {
        lifter->disable_magnetic_send(5000); // TODO make it varible
        ws.textAll("task_key " + String(key));
      }
    }

    // if (lifter->flag_find_max_force_pos)
    // {
    //   lifter->find_max_force_pos();
    // }

    vTaskDelay(50/portTICK_RATE_MS);
    lifter->check_stop(); // check if the lifter stopped and store to EEPROM
#ifdef DEBUG_HEAP_SIZE
    if (micros() - last_debug_heap_time > 1e6){
      xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);
      Serial.println(" [lifter " + String(xPortGetCoreID()) + "] the min free stack size is " + String((int32_t)uxTaskGetStackHighWaterMark(NULL)) + "  " + String(xPortGetFreeHeapSize()));
      xSemaphoreGive(xSerialSemaphore);
      last_debug_heap_time = micros();
    }
#endif
  }
}

void hardware_version_setup(int code)
{
  hardware_code = code;
  robot_mac_[8] = (hardware_code << 4) | (MAJOR_CODE & 0x0F);
  if (code == HARDWARE_CODE_STRUT) {
    // STRUT version, no force sensor, different lifter motor reduce ratio
    lifter1.set_reduce_ratio(LIFTER_REDUCTION_RATIO);
    lifter2.set_reduce_ratio(LIFTER_REDUCTION_RATIO);
  }
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Start setup");

  Serial2.begin(115200, SERIAL_8N1, PI_RX, PI_TX);  // 19,20:256,18  20,19:256:18
  // uart_driver_install(UART_NUM_2, 2048, 1024, 0, NULL, 0); // 2048字节接收缓冲区，1024字节发送缓冲区
  Serial.println("Serial2 begin");
  // while(!Serial);
  // while(!Serial2);

  EEPROM.begin(80);

  uint16_t rid = EEPROM.readShort(ROBOT_ID_EEPROM_OFFSET);
  if ((rid & 0xF000) == ROBOT_ID_VALID_CODE) {
    robot_id = (int) (rid & 0x0FFF);
    robot_mac_[1] = 0xFF & robot_id;
    Serial.print("Load cached robot id from EEPROM: ");
    Serial.println(robot_id);
  }
  else {
    robot_id = -1;
    robot_mac_[1] = 255;
    Serial.println("No valid robot id from EEPROM");
  }

  uint16_t code = EEPROM.readShort(ROBOT_HARDWARE_CODE_EPPROM_OFFSET);
  if ((code & 0xF000) == ROBOT_HARDWARE_CODE_VALID_CODE) {
    hardware_version_setup((int) (code & 0x0FFF));
    
    Serial.print("Load cached robot hardware code from EEPROM: ");
    Serial.println(hardware_code);
  }

  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(100000);

  // // 初始化 MPU6050
  // Wire.beginTransmission(0x68);
  // Wire.write(0x6B); // PWR_MGMT_1 寄存器
  // Wire.write(0);    // 设置为 0，唤醒 MPU6050
  // if (Wire.endTransmission() != 0) {
  //   Serial.println("Cannot init MPU6050");
  // } else {
  //   Serial.println("MPU6050 initialized");
  // }

  pinMode(V_SENSE, INPUT);
  //pinMode(V_SENSE, OUTPUT);
  //digitalWrite(V_SENSE, HIGH);

  encoder_A.attachFullQuad(AWE1, AWE2);
  encoder_B.attachFullQuad(BWE1, BWE2);

  ESP32Encoder::useInternalWeakPullResistors=NONE;
  encoder_C.attachHalfQuad(CWE1, CWE2);
  // encoder_D.attach(DWE1, DWE2);
  // encoder_E.attach(EWE1, EWE2);
  encoder_F.attachHalfQuad(FWE1, FWE2);

  // base1_driver.initialize();
  base2_driver.initialize();

  // int sdaPin = 19; // Example SDA pin
  // int sclPin = 20; // Example SCL pin
  // Wire.begin(sdaPin, sclPin); // Initialize I2C as master with custom pins

  xMutexLifter = xSemaphoreCreateMutex();
  xSerialSemaphore = xSemaphoreCreateMutex();
  xSendKeyQueue = xQueueCreate( 5, sizeof( uint32_t ) );

  xTaskCreatePinnedToCore(task_serial_comm, "task_serial_comm", 60*1024, NULL, 2, NULL, 0);
  //xTaskCreatePinnedToCore(task_wifi_communication, "task_wifi_communication", 60*1024, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(task_kalman_filter, "task_kalman_filter", 16*1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(task_lifter_control, "lifter1_control_loop", 16*1024, (void *) &lifter1, 3, NULL, 1);
  xTaskCreatePinnedToCore(task_lifter_control, "lifter2_control_loop", 16*1024, (void *) &lifter2, 3, NULL, 1);
  xTaskCreatePinnedToCore(task_base_control, "task_base_control", 30*1024,  NULL, 3, NULL, 0);
#ifdef DEBUG_CPU_USAGE
  xTaskCreate(task_cpu_debug, "task_cpu_debug", 2*1024, NULL, 3, NULL);
#endif
}

void loop(void)
{
  while(1);
}
