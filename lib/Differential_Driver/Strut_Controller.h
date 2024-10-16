#ifndef Strut_Controller_h
#define Strut_Controller_h

#include "Arduino.h"
#include "MPU6050_Filter.h"
#include "Differential_Driver.h"
#include <freertos/queue.h>
#include <queue>

#define TASK_RESOURCE_NONE  0b00
#define TASK_RESOURCE_BASE1 0b01
#define TASK_RESOURCE_BASE2 0b10
#define TASK_RESOURCE_BOTH  0b11


enum ControllerMode {MANUALLY, CLOSE_POS_CONTROL, CO_OPERATE_TRI, CLOSE_VEL_CONTROL};

struct ControlParam{
  enum ControllerMode mode = MANUALLY;
  uint8_t resource = TASK_RESOURCE_NONE;
  uint32_t task_key = 0;
  float target_position_x = 0.f;
  float target_position_y = 0.f;
  float target_position_z = 1.f;
  bool relative_to_node = false;
  bool orientation_only = false;
  bool absolute_orientation = false;
  bool straight_mainly = false;
  bool hold = false;
  float error_tolerance = 2.f * PI / 180.;
};

struct ControlState {
  bool is_base2 = false;
  bool reach_target = true;
  enum ControllerMode mode = MANUALLY;
  uint8_t resource = TASK_RESOURCE_NONE;
  bool first_approach_target = false;
  unsigned long approach_time_cache = 0;
  unsigned long approach_time = 0;
  float position_error_accurate = 0.;
  float position_error = 0.;
  float orientation_error = 0.;
  ControlParam* param = new ControlParam;
};

class StrutController
{ 
  private:
    DifferentialDriver *base2_driver;
    MPU6050_Filter* orientation_filter;

    QueueHandle_t* send_key_queue;

    //ControlState base1_state;
    ControlState base2_state;

    //std::queue<ControlParam*> base1_task_param_queue;
    std::queue<ControlParam*> base2_task_param_queue;

    void close_position_control_loop(ControlState*);
    void cooperate_triangle_loop(ControlState*);
    void close_velocity_control_loop(ControlState*);
    bool check_filter_status(ControlState*);
    void calculate_error(ControlState*);
    bool check_reach_target(ControlState*);
    void reset_state(ControlState* state, ControlParam* param);
    bool try_fetch_new_task(ControlState*);
    bool fast_task_switch(ControlState*);
    void switch_mode(ControlState*, enum ControllerMode);
    bool check_stop(ControlState*);
    void base_loop(ControlState*);
    
  public:
    StrutController(DifferentialDriver*, MPU6050_Filter*, QueueHandle_t*);
    
    void loop(void);
    void pwm(bool, float, float);
    void set_target_position(bool, enum ControllerMode, float, float, float, bool, bool, bool, bool, bool, float, uint32_t=0);   
    void set_target_velocity(bool, float, float, uint32_t=0);
};
  
#endif
