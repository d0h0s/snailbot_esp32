/* DC motor library for Arduino
created by Ashutosh M Bhatt (+91-9998476150) - Gujarat (INDIA) 
ver 1.0 - 12/10/2016

this library is used to control DC motors
it can control its speed from 0 to 100% and its direction
it can rotate motor forward and reverse at set speed 
it can start or stop the motor as well as provides DC BREAK for instant STOP 
just one has to select arduino pins for motor,then then start rotating motor using given library functions
*/
#ifndef Magnet_Lifter_h
#define Magnet_Lifter_h

#include "Arduino.h"
#include "DC_Motor.h"
#include <ESP32Encoder.h>
#include <InterruptEncoder.h>
#include <PID_v1.h>

#define LIFTER_REDUCTION_RATIO_PLUS 210. * 22. / 17. * 2. // 1200 turns per mm  // * 42. / 22. previously different reduce ratio
#define LIFTER_REDUCTION_RATIO 210. * 42. / 22. * 2.
#define DEFAULT_ENCODER_POLES 14 // now: since use software counter, 14 poles

#define LIFTER_INVALID_SPEED_AMP 0.1
#define LIFTER_POSITION_DEAD_ZONE 0.1

//#define LIMITER_TIMER_DEBOUNCE
#define LIMITER_STABLE_US 20 * 1000 // 20 ms

#define LIFTER_CALIB_UNKNOWN 0
#define LIFTER_CALIB_AUTO 1
#define LIFTER_CALIB_MANUAL 2

#define MAX_V 4096 // only used to decide step size
#define DEFAULT_MAX_FORCE_V 2400 // only used to decide step size

//#define USE_INTERRUPT_ENCODER 1

class MagnetLifter
{ 
  private:
    DC_Motor* motor;
#ifdef USE_INTERRUPT_ENCODER
    InterruptEncoder* encoder;
#else
    ESP32Encoder* encoder;
#endif
    uint8_t limiter_pin;
    uint8_t force_pin;
    PID position_pid;
    bool initialized=false;
    bool in_motion=false;
    bool start_stop = false;
    uint32_t task_key = 0; // 0 means no key
    int64_t last_encoder_count_eeprom; // used for EEPROM restore check
    uint32_t eeprom_offset;
    unsigned long disable_magnetic_send_time = 0;
    uint16_t calibrated_max_force_v = DEFAULT_MAX_FORCE_V;
    double lifter_count_ratio;

    float last_motor_pwm = 0.;
    unsigned long last_motor_pwm_time = 0;
    int64_t last_motor_pwm_encoder_count = 0;

  public:
#ifdef USE_INTERRUPT_ENCODER
    // MagnetLifter(DC_Motor*, InterruptEncoder*, uint8_t, uint8_t, uint32_t, uint16_t);
    MagnetLifter(DC_Motor*, InterruptEncoder*, uint8_t, uint32_t, uint16_t);
#else
    // MagnetLifter(DC_Motor*, ESP32Encoder*, uint8_t, uint8_t, uint32_t, uint16_t);
    MagnetLifter(DC_Motor*, ESP32Encoder*, uint8_t, uint32_t, uint16_t);
#endif
  void set_reduce_ratio(double reduce_ratio);
  
  void speed(double speed); 
  void raw_speed(double speed){motor->speed(speed);};
  
  void loop(void);
  void set_target_position(double position); 
  void set_target_position(double position, uint32_t key); 
  double update_position(void);
  bool is_initialized(void) {return initialized;}
  bool is_in_motion(void) {return in_motion;}
  void initialize(void);
//   void force_calibrate(void);
  bool inspect(void);
  void ready_send_data(uint16_t, int, int);
  void brake(void) { motor->brake(); }
  void check_stop(void);
  void check_stalled(void);
  void set_speed_pid(double, double, double);
  void reset(void);
  uint32_t clear_task_key(void);
  void disable_magnetic_send(int duration);
  void magnetic_send_finished(void);
  double get_current_position(void);
  void set_current_position(double);
  void IRAM_ATTR isr();
  void limiter_triggered(void);
//   uint16_t read_force_v(void);
//   void find_max_force_pos(void);
//   void find_max_force_pos_dir(double, uint16_t*, float*);
  bool is_touch_limiter(void) {return digitalRead(limiter_pin) == LOW;}
  double motor_speed, expected_speed, current_position, target_position = 0.;
  int64_t last_encoder_count; uint32_t last_encoder_changed_timer;
  bool reach_target = true;
  bool flag_resend = false;
  bool flag_magnetic_send = false;
//   bool flag_find_max_force_pos = false;
  bool flag_start_calibrate = false;
  uint16_t sleep_ready_flag;
  uint16_t mag_send_message;
  int message_length;
  int current_message_priority = 0;
  byte calibration_approach = LIFTER_CALIB_UNKNOWN;
#ifdef LIMITER_TIMER_DEBOUNCE
  hw_timer_t * limiter_debounce_timer = NULL;
  void attach_timer_isr(hw_timer_t*, void(*fn)());
#endif
};
  
#endif
