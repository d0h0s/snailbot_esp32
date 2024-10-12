#include "Arduino.h"
#include "Magnet_Lifter.h"
#include <FunctionalInterrupt.h>
#include <EEPROM.h>

#define EEPROM_CACHE

#define IN_MOTION_CODE 139
#define NOT_IN_MOTION_CODE 11

#define AUTO_CALIB_CODE 121
#define MANUAL_CALIB_CODE 208

#define FORCE_V_CALIBRATED_CODE 158

#ifdef USE_INTERRUPT_ENCODER
// MagnetLifter::MagnetLifter(DC_Motor* motor_, InterruptEncoder* encoder_, uint8_t limiter_pin_, uint8_t force_pin_, uint32_t eeprom_offset_, uint16_t sleep_ready_flag_)
// :position_pid(&current_position, &expected_speed, &target_position, 5., 15., 0.1, DIRECT)
MagnetLifter::MagnetLifter(DC_Motor* motor_, InterruptEncoder* encoder_, uint8_t limiter_pin_, uint32_t eeprom_offset_, uint16_t sleep_ready_flag_)
:position_pid(&current_position, &expected_speed, &target_position, 5., 15., 0.1, DIRECT)
{
  motor = motor_;
  encoder = encoder_;
  limiter_pin = limiter_pin_;
//   force_pin = force_pin_;
  eeprom_offset = eeprom_offset_;
  sleep_ready_flag = sleep_ready_flag_;
  set_reduce_ratio((double) LIFTER_REDUCTION_RATIO_PLUS);

  pinMode(limiter_pin, INPUT_PULLUP);
  pinMode(limiter_pin, INPUT);
  position_pid.SetMode(AUTOMATIC);
  position_pid.SetOutputLimits(-1.0, 1.0);
  position_pid.SetSampleTime(10); 
  
}
#else
// MagnetLifter::MagnetLifter(DC_Motor* motor_, ESP32Encoder* encoder_, uint8_t limiter_pin_, uint8_t force_pin_, uint32_t eeprom_offset_, uint16_t sleep_ready_flag_)
// :position_pid(&current_position, &expected_speed, &target_position, 20., 3., 0, DIRECT)
MagnetLifter::MagnetLifter(DC_Motor* motor_, ESP32Encoder* encoder_, uint8_t limiter_pin_, uint32_t eeprom_offset_, uint16_t sleep_ready_flag_)
:position_pid(&current_position, &expected_speed, &target_position, 20., 3., 0, DIRECT)
{
  motor = motor_;
  encoder = encoder_;
  limiter_pin = limiter_pin_;
//   force_pin = force_pin_;
  eeprom_offset = eeprom_offset_;
  sleep_ready_flag = sleep_ready_flag_;
  set_reduce_ratio((double) LIFTER_REDUCTION_RATIO_PLUS);

  pinMode(limiter_pin, INPUT_PULLUP);
  pinMode(limiter_pin, INPUT);
  position_pid.SetMode(AUTOMATIC);
  position_pid.SetOutputLimits(-1.0, 1.0);
  position_pid.SetSampleTime(10); 
}
#endif

void MagnetLifter::set_reduce_ratio(double reduce_ratio)
{
#ifdef USE_INTERRUPT_ENCODER
  lifter_count_ratio = reduce_ratio * ((double) DEFAULT_ENCODER_POLES) * 2.;
#else
  lifter_count_ratio = reduce_ratio * ((double) DEFAULT_ENCODER_POLES);
#endif
}

// uint16_t MagnetLifter::read_force_v(void)
// {
//   uint16_t force_v = 0;
//   for (int i=0; i<16; i++) {
//     force_v += analogRead(force_pin);
//   }
//   force_v = force_v / 16;
//   return force_v;
// }

// void MagnetLifter::find_max_force_pos_dir(double raw_speed, uint16_t *_max_force_v, float *_max_force_pos)
// {
//   double max_force_pos = get_current_position();
//   uint16_t max_force_v = read_force_v();
//   while(1)
//   {
//     speed(raw_speed);
//     vTaskDelay(1 / portTICK_RATE_MS);
//     double current_pos = get_current_position();
//     //double step_size = ((float)(MAX_V - analogRead(force_pin)))/ (MAX_V-calibrated_max_force_v);
//     //if (step_size > 0.95) step_size = 0.1; // fine tune
//     //else step_size = (1 - step_size) * 3;
//     double step_size = 0.2;
//     if (abs(max_force_pos-current_pos) > step_size) {
//       speed(0);
//       vTaskDelay(500 / portTICK_RATE_MS);
//       uint16_t force_v = read_force_v();
//       Serial.println(force_v);
//       Serial.println(current_pos);
//       if (force_v < max_force_v) {
//         max_force_v = force_v;
//         max_force_pos = current_pos;
//       }
//       else {
//         speed(0);
//         *_max_force_v = max_force_v;
//         *_max_force_pos = max_force_pos;
//         return;
//       }
//     }
//   }
// }

// void MagnetLifter::find_max_force_pos(void)
// {
//   flag_find_max_force_pos = false;
  
//   // TODO decide whether change target by current position
//   //if (current_position < -14.);

//   set_target_position(-14.); // a upper bound max force position
//   while (!reach_target)
//   {
//     loop();
//     vTaskDelay(5/portTICK_RATE_MS);
//   }
//   speed(0);
//   vTaskDelay(500 / portTICK_RATE_MS);

//   uint16_t max_force_v = read_force_v();
//   Serial.println(max_force_v);
//   if (max_force_v > 4000) // not connect to node
//     return;
  
//   float max_force_pos;
//   //uint16_t max_force_v;
//   find_max_force_pos_dir(-1., &max_force_v, &max_force_pos);
//   find_max_force_pos_dir(1., &max_force_v, &max_force_pos);
//   set_target_position(max_force_pos);
//   calibrated_max_force_v = max_force_v;
//   // EEPROM.writeUShort(eeprom_offset + 11, calibrated_max_force_v);
//   // EEPROM.writeUShort(eeprom_offset + 10, FORCE_V_CALIBRATED_CODE);
//   // EEPROM.commit();
// }

void MagnetLifter::speed(double speed)                       
{
  if (!initialized) return;
  if ((speed > 0) & (encoder->getCount() > -1000)) speed = 0.; // keep distance with 0
  if ((speed > 0) & (digitalRead(limiter_pin) == LOW)) speed = 0.; // limiter triggered

  if (abs(speed) < LIFTER_INVALID_SPEED_AMP) motor_speed = 0.;
  else motor_speed = speed;
  motor->speed(motor_speed); 
}

void MagnetLifter::check_stalled(void)
{
  float motor_pwm = motor->get_speed();
  unsigned long current_time = micros();
  int64_t encoder_count = encoder->getCount();
  if (last_motor_pwm_time > 0)
  {
    if ((abs(last_motor_pwm) > 0.8) && (last_motor_pwm_encoder_count == encoder_count)) // motor stalled or encoder failed
    {
      if ((unsigned long)(current_time - last_motor_pwm_time) > 1000 * 1000) // stalled for 400 ms, stop and uninitialzie
      {
        initialized = false;
        reach_target = true;
        motor->brake();
        EEPROM.writeByte(eeprom_offset, IN_MOTION_CODE);
        EEPROM.commit();
        last_motor_pwm_time = 0;
      }
    }
    else 
    {
      last_motor_pwm_time = current_time;
    }
  }
  else last_motor_pwm_time = current_time;

  last_motor_pwm = motor_pwm;
  last_motor_pwm_encoder_count = encoder_count;
}

void MagnetLifter::check_stop(void)
{
  float speed = motor->loop();

  if (abs(speed) < LIFTER_INVALID_SPEED_AMP) 
  {
    if (in_motion | start_stop) {
      in_motion = false;
#ifdef EEPROM_CACHE
      if (start_stop) {  // after may stopped
        if (encoder->getCount() == last_encoder_count_eeprom) { // really stopped
          EEPROM.writeByte(eeprom_offset, NOT_IN_MOTION_CODE);
          EEPROM.writeLong64(eeprom_offset+1, encoder->getCount());
          EEPROM.commit();
          start_stop = false;
        }
        else last_encoder_count_eeprom = encoder->getCount(); // update counter and detect stop next time
      }
      else { // may stopped
        last_encoder_count_eeprom = encoder->getCount();
        start_stop = true;
      }
#endif
    }
  }
  else
  {
    if (!in_motion) // start to move
    {
      in_motion = true;
#ifdef EEPROM_CACHE // if start moving resore the info to EEPROM
      start_stop = false;
      EEPROM.writeByte(eeprom_offset, IN_MOTION_CODE);
      EEPROM.commit();
#endif
    }
  }
}

void MagnetLifter::loop(void)
{
  if (!initialized) return;
  update_position();
  position_pid.Compute();
  speed(expected_speed);
  check_stop();
  check_stalled(); // check stalled by pwm and encoder count
}

void MagnetLifter::set_target_position(double position)
{
  if (position > -0.03) return;
  target_position = position;
  last_encoder_count = encoder->getCount();
  update_position();
}

void MagnetLifter::set_target_position(double position, uint32_t key)
{
  task_key = key;
  set_target_position(position);
  reach_target = false; // trigger a state change
}

double MagnetLifter::update_position(void)
{
  int64_t encoder_count = encoder->getCount();
  current_position = ((double) encoder_count) / lifter_count_ratio;
  double error = abs(current_position - target_position);
  if (encoder_count != last_encoder_count) last_encoder_changed_timer = micros();
  if ( (error < LIFTER_POSITION_DEAD_ZONE) ) { // stop for 0.3s
    reach_target = true;
  }
  else reach_target = false;
  last_encoder_count = encoder_count;
  return error;
}

double MagnetLifter::get_current_position(void)
{
  return ((double)encoder->getCount()) / lifter_count_ratio;
}

void MagnetLifter::set_current_position(double position) // call when stopped for manual calibration 
{
  
  encoder->setCount((int64_t) (position * lifter_count_ratio));
  int64_t count = encoder->getCount();
  calibration_approach = LIFTER_CALIB_AUTO;
  initialized = true;

  EEPROM.writeByte(eeprom_offset, NOT_IN_MOTION_CODE);
  EEPROM.writeLong64(eeprom_offset+1, count);
  EEPROM.writeByte(eeprom_offset+9, MANUAL_CALIB_CODE);
  EEPROM.commit();
  last_encoder_count_eeprom = count;
  last_encoder_count = count;
  // better reboot after calibration
}

uint32_t MagnetLifter::clear_task_key(void)
{
  uint32_t temp = task_key;
  task_key = 0;
  return temp;
}

void MagnetLifter::reset(void)
{
  reach_target = true;
  speed(0.);
}

void MagnetLifter::initialize(void)
{
#ifdef EEPROM_CACHE
  if (EEPROM.readByte(eeprom_offset) == NOT_IN_MOTION_CODE) {
    encoder->setCount(EEPROM.readLong64(eeprom_offset+1));
    initialized = true;
    byte calib_code = EEPROM.readByte(eeprom_offset+9);
    if (calib_code == MANUAL_CALIB_CODE) calibration_approach = LIFTER_CALIB_MANUAL;
    else if (calib_code == AUTO_CALIB_CODE) calibration_approach = LIFTER_CALIB_AUTO;
  }
  //if (EEPROM.readByte(eeprom_offset + 10) == FORCE_V_CALIBRATED_CODE) calibrated_max_force_v = EEPROM.readUShort(eeprom_offset + 11);
#endif
  detachInterrupt(digitalPinToInterrupt(limiter_pin));
  attachInterrupt(digitalPinToInterrupt(limiter_pin), std::bind(&MagnetLifter::isr,this), FALLING);
}

// void MagnetLifter::force_calibrate(void)
// {
//   initialized = false;
//   if (digitalRead(limiter_pin) == HIGH) // initialize by move until reach the limiter
//     motor->speed(1);
//   else // already touch the limiter
//     isr();

//   while(!initialized) vTaskDelay(500/portTICK_RATE_MS);
  
//   encoder->clearCount();  // only calibrate when initializing
//   calibration_approach = LIFTER_CALIB_AUTO;  
//   EEPROM.writeByte(eeprom_offset+9, AUTO_CALIB_CODE);
//   EEPROM.commit();
//   flag_start_calibrate = false;
// }

void MagnetLifter::ready_send_data(uint16_t message, int length, int priority)
{
  if (micros() < disable_magnetic_send_time) return;

  if (flag_magnetic_send) {
    if (priority <= current_message_priority) return;
    flag_resend = true;
  }
  current_message_priority = priority;
  mag_send_message = message;
  message_length = length;
  flag_magnetic_send = true;
}

void MagnetLifter::magnetic_send_finished(void)
{
  if (flag_resend) flag_resend = false;
  else {
    current_message_priority = 0;
    flag_magnetic_send = false;
  }
}

void MagnetLifter::disable_magnetic_send(int duration) // ms
{
  disable_magnetic_send_time = micros() + duration * 1000;
  flag_magnetic_send = false;
}

void MagnetLifter::set_speed_pid(double Kp, double Ki, double Kd)
{
  position_pid.SetTunings(Kp, Ki, Kd);
}

void IRAM_ATTR MagnetLifter::isr()
{
#ifdef LIMITER_TIMER_DEBOUNCE
  if (!initialized) encoder->clearCount(); 
  
  if (timerRead(limiter_debounce_timer) < LIMITER_STABLE_US) {
    timerWrite(limiter_debounce_timer, 0);
    
  }
  else{
    timerAlarmWrite(limiter_debounce_timer, LIMITER_STABLE_US, false); //set time in us
    timerWrite(limiter_debounce_timer, 0);
    timerAlarmEnable(limiter_debounce_timer); 
  }
#else
  motor->brake();
  if (!initialized) initialized = true;
#endif
}

void MagnetLifter::limiter_triggered(void)
{
  motor->brake();
  if (flag_start_calibrate) initialized = true;
}

#ifdef LIMITER_TIMER_DEBOUNCE
void MagnetLifter::attach_timer_isr(hw_timer_t* timer, void(*fn)())
{
  limiter_debounce_timer = timer;
  timerAttachInterrupt(limiter_debounce_timer, fn, true);  //attach callback
}
#endif

bool MagnetLifter::inspect(void)
{
  int64_t count = encoder->getCount();
  motor->speed(-1);
  delay(100);
  motor->speed(0);
  return (abs(encoder->getCount() - count) > 0);
}
