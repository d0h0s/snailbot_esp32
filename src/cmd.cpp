#include "HardwareSerial.h"
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Strut_Controller.h"
#include "Differential_Driver.h"
#include <MPU6050_Filter.h>
#include "Magnet_Lifter.h"
#include "common.h"
#include "eeprom_offset.h"

extern DC_Motor motor_A;
extern DC_Motor motor_B;
extern DC_Motor motor_C;
// extern DC_Motor motor_D;
// extern DC_Motor motor_E;
extern DC_Motor motor_F;

extern ESP32Encoder encoder_A; 
extern ESP32Encoder encoder_B; 
extern ESP32Encoder encoder_C;
// extern ESP32Encoder encoder_D;
// extern ESP32Encoder encoder_E;
extern ESP32Encoder encoder_F;

extern MagnetLifter lifter1;
extern MagnetLifter lifter2;

// extern DifferentialDriver base1_driver;
extern DifferentialDriver base2_driver;

extern StrutController strut_controller;

extern MPU6050_Filter imu;
extern bool start_calibrate_imu;

extern byte MAJOR_CODE, MINOR_CODE;
extern int robot_id;
extern int hardware_code;
extern uint8_t robot_mac_[14];
extern IPAddress *monitor_address;

extern IPAddress *lifter1_node_address, lifter2_node_address;
extern int lifter1_node_id, lifter2_node_id;
extern double lifter1_event_time, lifter2_event_time;

extern SemaphoreHandle_t xSerialSemaphore;
// extern AsyncWebSocket ws;

extern void hardware_version_setup(int);

bool cmd_parse(char *cmd)
{
  uint8_t argc = 0;
  char *argv[15];
  char *p;

  argv[argc] = strtok_r(cmd, " ", &p);
  if (argv[argc] == NULL)
  {
    return true;
  }

  do
  {
    argv[++argc] = strtok_r(NULL, " ", &p);
  } while ((argc < 15) && (argv[argc] != NULL));

  argc--;

  if (strcmp(argv[0], "base2") == 0)
  {
    if (argc >= 1)
    {
      if (strcmp(argv[1], "speed") == 0 && argc >= 3)
      {
        float linear = String(argv[2]).toFloat();
        float angular = String(argv[3]).toFloat();
        strut_controller.pwm(true, linear, angular);
      }
      else if (strcmp(argv[1], "vel") == 0 && argc >= 3)
      {
        if (argc >= 4)
        {
          uint32_t key = ((uint32_t)String(argv[4]).toInt());
          strut_controller.set_target_velocity(true, String(argv[2]).toFloat(), String(argv[3]).toFloat(), key);
        }
        else
          strut_controller.set_target_velocity(true, String(argv[2]).toFloat(), String(argv[3]).toFloat());
      }
      else if (strcmp(argv[1], "position") == 0 && argc >= 9)
      {
        float x = String(argv[2]).toFloat();
        float y = String(argv[3]).toFloat();
        float z = String(argv[4]).toFloat();
        bool relative_to_node = ((bool)String(argv[5]).toInt());
        bool orientation_only = ((bool)String(argv[6]).toInt());
        bool absolute_orientation = ((bool)String(argv[7]).toInt());
        bool hold = ((bool)String(argv[8]).toInt());
        float error_tolerance = String(argv[9]).toFloat();
        if (argc >= 10)
        {
          uint32_t key = ((uint32_t)String(argv[10]).toInt());
          strut_controller.set_target_position(true, CLOSE_POS_CONTROL, x, y, z, relative_to_node, orientation_only, absolute_orientation, false, hold, error_tolerance, key);
        }
        else
          strut_controller.set_target_position(true, CLOSE_POS_CONTROL, x, y, z, relative_to_node, orientation_only, absolute_orientation, false, hold, error_tolerance);
      }
    }
  }
  // else if (strcmp(argv[0], "base1") == 0)
  // {
  //   if (argc >= 1)
  //   {
  //     if (strcmp(argv[1], "speed") == 0 && argc >= 3)
  //     {
  //       float linear = String(argv[2]).toFloat();
  //       float angular = String(argv[3]).toFloat();
  //       strut_controller.pwm(false, linear, angular);
  //     }
  //     else if (strcmp(argv[1], "vel") == 0 && argc >= 3)
  //     {
  //       if (argc >= 4)
  //       {
  //         uint32_t key = ((uint32_t)String(argv[4]).toInt());
  //         strut_controller.set_target_velocity(false, String(argv[2]).toFloat(), String(argv[3]).toFloat(), key);
  //       }
  //       else
  //         strut_controller.set_target_velocity(false, String(argv[2]).toFloat(), String(argv[3]).toFloat());
  //     }
  //     else if (strcmp(argv[1], "position") == 0 && argc >= 9)
  //     {
  //       float x = String(argv[2]).toFloat();
  //       float y = String(argv[3]).toFloat();
  //       float z = String(argv[4]).toFloat();
  //       bool relative_to_node = ((bool)String(argv[5]).toInt());
  //       bool orientation_only = ((bool)String(argv[6]).toInt());
  //       bool absolute_orientation = ((bool)String(argv[7]).toInt());
  //       bool hold = ((bool)String(argv[8]).toInt());
  //       float error_tolerance = String(argv[9]).toFloat();
  //       if (argc >= 10)
  //       {
  //         uint32_t key = ((uint32_t)String(argv[10]).toInt());
  //         strut_controller.set_target_position(false, CLOSE_POS_CONTROL, x, y, z, relative_to_node, orientation_only, absolute_orientation, false, hold, error_tolerance, key);
  //       }
  //       else
  //         strut_controller.set_target_position(false, CLOSE_POS_CONTROL, x, y, z, relative_to_node, orientation_only, absolute_orientation, false, hold, error_tolerance);
  //     }
  //   }
  // }
  if (strcmp(argv[0], "lifter2") == 0)
  {
    if (argc == 0)
      return false;

    if (strcmp(argv[1], "speed") == 0)
    {
      if (argc >= 2)
      {
        double speed = String(argv[2]).toDouble();
        lifter2.disable_magnetic_send(5000);
        lifter2.speed(speed);
      }
    }
    else if (strcmp(argv[1], "pos") == 0)
    {
      if (argc >= 2)
      {
        double target = String(argv[2]).toDouble();
        if (argc >= 3)
        {
          uint32_t key = ((uint32_t)String(argv[3]).toInt());
          lifter2.set_target_position(target, key);
        }
        else
          lifter2.set_target_position(target);
      }
    }
    else if (strcmp(argv[1], "stop") == 0)
    {
      lifter2.reach_target = true;
    }
    // else if (strcmp(argv[1], "max_force") == 0)
    // {
    //   lifter2.flag_find_max_force_pos = true;
    // }
    else if (strcmp(argv[1], "comm") == 0)
    {
      if (argc >= 2)
      {
        int id = String(argv[2]).toInt();
        if (id == -1)
          id = robot_id;
        if (id == -1)
          return true;
        uint16_t mag_send_message = (0x3F & id) | 0x80 | 0x40;
        uint16_t parity = 0;
        for (int i = 0; i < 9; i++)
        {
          if (mag_send_message & (1 << i))
            parity += 1;
        }
        lifter2.ready_send_data(mag_send_message << 1 | (parity & 0x1), 10, 1); // even parity
      }
    }
    else if (strcmp(argv[1], "reset") == 0)
    {
      lifter2.flag_start_calibrate = true;
    }
    else if (strcmp(argv[1], "set_max_friction_pos") == 0)
    {
      if (argc >= 2)
      {
        lifter2.set_current_position(String(argv[2]).toFloat());
        // ws.textAll("notify set_maxfriction_pos2->" + String(lifter2.get_current_position()));
      }
    }
    else if (strcmp(argv[1], "inspect") == 0)
    {
      // ws.textAll("notify lifter2_inspect:" + String(!lifter2.inspect()));
    }
    else if (strcmp(argv[1], "raw_speed") == 0)
    {
      if (argc >= 2)
      {
        double speed = String(argv[2]).toDouble();
        lifter2.raw_speed(speed);
      }
    }
  }
  else if (strcmp(argv[0], "lifter1") == 0)
  {
    if (argc == 0)
      return false;

    if (strcmp(argv[1], "speed") == 0)
    {
      if (argc >= 2)
      {
        double speed = String(argv[2]).toDouble();
        lifter1.disable_magnetic_send(5000);
        lifter1.speed(speed);
      }
    }
    else if (strcmp(argv[1], "pos") == 0)
    {
      if (argc >= 2)
      {
        double target = String(argv[2]).toDouble();
        if (argc >= 3)
        {
          uint32_t key = ((uint32_t)String(argv[3]).toInt());
          lifter1.set_target_position(target, key);
        }
        else
          lifter1.set_target_position(target);
      }
    }
    else if (strcmp(argv[1], "stop") == 0)
    {
      lifter1.reach_target = true;
    }
    // else if (strcmp(argv[1], "max_force") == 0)
    // {
    //   lifter1.flag_find_max_force_pos = true;
    // }
    else if (strcmp(argv[1], "comm") == 0)
    {
      if (argc >= 2)
      {
        int id = String(argv[2]).toInt();
        if (id == -1)
          id = robot_id;
        if (id == -1)
          return true;
        uint16_t mag_send_message = (0x3F & id) | 0x80;
        uint16_t parity = 0;
        for (int i = 0; i < 9; i++)
        {
          if (mag_send_message & (1 << i))
            parity += 1;
        }
        lifter1.ready_send_data(mag_send_message << 1 | (parity & 0x1), 10, 1); // even parity
      }
    }
    else if (strcmp(argv[1], "reset") == 0)
    {
      lifter1.flag_start_calibrate = true;
    }
    else if (strcmp(argv[1], "set_max_friction_pos") == 0)
    {
      if (argc >= 2)
      {
        lifter1.set_current_position(String(argv[2]).toFloat());
        // ws.textAll("notify set_maxfriction_pos1->" + String(lifter1.get_current_position()));
      }
    }
    else if (strcmp(argv[1], "inspect") == 0)
    {
      // ws.textAll("notify lifter1_inspect:" + String(!lifter1.inspect()));
    }
    else if (strcmp(argv[1], "raw_speed") == 0)
    {
      if (argc >= 2)
      {
        double speed = String(argv[2]).toDouble();
        lifter1.raw_speed(speed);
      }
    }
  }
  
  else if (strcmp(argv[0], "id") == 0)
  {
    if (argc >= 1)
    {
      int rid = String(argv[1]).toInt();
      if ((rid >= 0) && (rid < 0x0FFF) && (rid != robot_id)) // valid robot id assigned
      {
        if (robot_id != -1)
          Serial.printf("Error: robot id changed %d -> %d\n", robot_id, rid);
        robot_id = rid;
        robot_mac_[1] = 0xFF & robot_id;
        EEPROM.writeShort(ROBOT_ID_EEPROM_OFFSET, (((uint16_t)robot_id) & 0x0FFF) | ROBOT_ID_VALID_CODE);
        EEPROM.commit();
        Serial.print("Cache robot id: ");
        Serial.println(robot_id);
      }
    }
  }
  else if (strcmp(argv[0], "hardware_code") == 0)
  {
    if (argc >= 1)
    {
      int code = String(argv[1]).toInt();
      if ((code >= HARDWARE_CODE_UNKNOWN) && (code <= HARDWARE_CODE_STRUT_PLUS) && (code != hardware_code)) // valid robot id assigned
      {
        hardware_version_setup(code);
        EEPROM.writeShort(ROBOT_HARDWARE_CODE_EPPROM_OFFSET, (((uint16_t)hardware_code) & 0x0FFF) | ROBOT_HARDWARE_CODE_VALID_CODE);
        EEPROM.commit();
        Serial.print("Cache hardware code: ");
        Serial.println(hardware_code);
      }
    }
  }
  else if (strcmp(argv[0], "monitor_ip") == 0)
  {
    if (monitor_address == &INADDR_NONE)
    {
      IPAddress *addr = new IPAddress();
      if (addr->fromString(String(argv[1])))
      {
        monitor_address = addr;
        Serial.print("Monitor address: ");
        Serial.println(monitor_address->toString());
      }
      else
        delete addr;
    }
    else
    {
      monitor_address->fromString(String(argv[1]));
      Serial.print("Monitor address changed: ");
      Serial.println(monitor_address->toString());
    }
  }

  else if (strcmp(argv[0], "stop") == 0)
  {
  }
  else if (strcmp(argv[0], "imu_calib") == 0)
  {
    start_calibrate_imu = true;
  }
  else if (strcmp(argv[0], "lifterPID") == 0)
  {
    if (argc >= 3)
    {
      double Kp, Ki, Kd;
      Kp = String(argv[1]).toFloat();
      Ki = String(argv[2]).toFloat();
      Kd = String(argv[3]).toFloat();
      lifter1.set_speed_pid(Kp, Ki, Kd);
      lifter2.set_speed_pid(Kp, Ki, Kd);
    }
  }
  else if (strcmp(argv[0], "PID") == 0)
  { // wheel speed PID
    if (argc >= 3)
    {
      double Kp, Ki, Kd;
      Kp = String(argv[1]).toFloat();
      Ki = String(argv[2]).toFloat();
      Kd = String(argv[3]).toFloat();
      //base1_driver.set_speed_pid(Kp, Ki, Kd);
      base2_driver.set_speed_pid(Kp, Ki, Kd);
    }
  }
  else if (strcmp(argv[0], "orientationPID") == 0)
  {
    if (argc >= 3)
    {
      double Kp, Ki, Kd;
      Kp = String(argv[1]).toFloat();
      Ki = String(argv[2]).toFloat();
      Kd = String(argv[3]).toFloat();
      //base1_driver.set_orientation_pid(Kp, Ki, Kd);
      base2_driver.set_orientation_pid(Kp, Ki, Kd);
    }
  }
  else if (strcmp(argv[0], "positionPID") == 0)
  {
    if (argc >= 3)
    {
      double Kp, Ki, Kd;
      Kp = String(argv[1]).toFloat();
      Ki = String(argv[2]).toFloat();
      Kd = String(argv[3]).toFloat();
      //base1_driver.set_position_pid(Kp, Ki, Kd);
      base2_driver.set_position_pid(Kp, Ki, Kd);
    }
  }
  else if (strcmp(argv[0], "base_inspect") == 0)
  {
    //uint8_t base1_ok = base1_driver.inspect();
    uint8_t base2_ok = base2_driver.inspect();
    // ws.textAll("base2:" + String(base2_ok) + "\r\n");
  }
  else
    return false;
  return true;
}