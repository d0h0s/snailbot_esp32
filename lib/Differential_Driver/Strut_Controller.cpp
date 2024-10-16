#include "Strut_Controller.h"

StrutController::StrutController(DifferentialDriver* base2_driver_, MPU6050_Filter* filter, QueueHandle_t* send_key_queue_) 
{
    //base1_driver = base1_driver_;
    base2_driver = base2_driver_;
    orientation_filter = filter;
    send_key_queue = send_key_queue_;

    //base1_state.is_base2 = false;
    base2_state.is_base2 = true;
    //switch_mode(&base1_state, MANUALLY);
    switch_mode(&base2_state, MANUALLY);
}

void StrutController::pwm(bool cmd_base2, float linear, float angular)
{
    // if (cmd_base2) {
    // clear task queue
    while (!base2_task_param_queue.empty()) {
        ControlParam* param = base2_task_param_queue.front();
        base2_task_param_queue.pop();
        delete param;
    }
    
    switch_mode(&base2_state, MANUALLY);
    base2_driver->pwm(linear, angular);
    // }
    // else {
    //     // clear task queue
    //     while (!base1_task_param_queue.empty()) {
    //         ControlParam* param = base1_task_param_queue.front();
    //         base1_task_param_queue.pop();
    //         delete param;
    //     }

    //     switch_mode(&base1_state, MANUALLY);
    //     base1_driver->pwm(linear, angular);
    // }
}

void StrutController::switch_mode(ControlState* state, enum ControllerMode mode)
{
    switch (mode) {
        case MANUALLY:
            state->param->task_key = 0;
            state->first_approach_target = false;
            state->approach_time_cache = 0;
            state->approach_time = 0.f;

            state->reach_target = true;
            state->resource = TASK_RESOURCE_NONE;
            if (state->mode == CO_OPERATE_TRI) {
                //base1_driver->pwm(0., 0.);
                base2_driver->pwm(0., 0.);
            }
            else {
                base2_driver->pwm(0., 0.);
                // if (state->is_base2) base2_driver->pwm(0., 0.);
                // else base1_driver->pwm(0., 0.);
            }
            
            break;

        case CLOSE_POS_CONTROL:
            // if (state->is_base2) {
            state->resource = TASK_RESOURCE_BASE2;
            base2_driver->enable_velocity_control();
            // }
            // else {
            //     state->resource = TASK_RESOURCE_BASE1;
            //     base1_driver->enable_velocity_control();
            // }
            break;

        case CO_OPERATE_TRI:
            state->resource = TASK_RESOURCE_BOTH;
            //base1_driver->enable_velocity_control();
            base2_driver->enable_velocity_control();
            break;

        case CLOSE_VEL_CONTROL:
            // if (state->is_base2) {
            state->resource = TASK_RESOURCE_BASE2;
            base2_driver->enable_velocity_control();
            // }
            // else {
            //     state->resource = TASK_RESOURCE_BASE1;
            //     base1_driver->enable_velocity_control();
            // }
    }
    state->mode = mode;
}

void StrutController::base_loop(ControlState* state)
{
    switch (state->mode) 
    {
        case MANUALLY:
            break;

        case CLOSE_POS_CONTROL:
        case CO_OPERATE_TRI:
            if (!state->reach_target) 
            {
                if (!check_filter_status(state)) {
                    switch_mode(state, MANUALLY);
                }
                calculate_error(state);
                if (check_reach_target(state)) {
                    if (state->param->hold) {
                        //if (state->resource & TASK_RESOURCE_BASE1) base1_driver->pwm(0., 0.);
                        if (state->resource & TASK_RESOURCE_BASE2) base2_driver->pwm(0., 0.);
                        break;
                    }
                    else {
                        uint32_t key = state->param->task_key;
                        
                        // switch to next task without stop
                        if ( !fast_task_switch(state) ) {
                            // not fast switch, wait for the motors stop, then reach the target
                            if (check_stop(state)) switch_mode(state, MANUALLY);
                            else 
                            {
                                //if (state->resource & TASK_RESOURCE_BASE1) base1_driver->pwm(0., 0.);
                                if (state->resource & TASK_RESOURCE_BASE2) base2_driver->pwm(0., 0.);
                                break;
                            }
                        }

                        if (key > 0) xQueueSend( *send_key_queue, ( void * ) &key, ( TickType_t ) 0 );
                        break;
                    }
                }
                else {
                    //if (state->resource & TASK_RESOURCE_BASE1) base1_driver->enable_velocity_control();
                    if (state->resource & TASK_RESOURCE_BASE2) base2_driver->enable_velocity_control();

                    if (state->mode == CLOSE_POS_CONTROL)
                        close_position_control_loop(state);
                    else
                        cooperate_triangle_loop(state);
                }
            }
            break;

        case CLOSE_VEL_CONTROL:
            close_velocity_control_loop(state);
            break;
    }
}

bool StrutController::check_stop(ControlState* state)
{
    // return !(((state->resource & TASK_RESOURCE_BASE1) && (!base1_driver->check_stop())) || 
    //          ((state->resource & TASK_RESOURCE_BASE2) && (!base2_driver->check_stop())));

    return !((state->resource & TASK_RESOURCE_BASE2) && (!base2_driver->check_stop()));
}

void StrutController::loop(void)
{
    //try_fetch_new_task(&base1_state);
    try_fetch_new_task(&base2_state);

    //base_loop(&base1_state);
    base_loop(&base2_state);
    
    //base1_driver->loop();
    base2_driver->loop();
}

void StrutController::close_velocity_control_loop(ControlState* state) 
{
    
    DifferentialDriver *driver;
    driver = base2_driver;
    // if (state->is_base2) driver = base2_driver;
    // else driver = base1_driver;

    float expected_linear = state->param->target_position_x;
    float expected_angular = state->param->target_position_y;

    driver->speed_raw(expected_linear, expected_angular);
}

void StrutController::close_position_control_loop(ControlState* state) 
{
    float expected_linear = 0.f, expected_angular = 0.f;
    DifferentialDriver *driver;
    driver = base2_driver;
    // if (state->is_base2) driver = base2_driver;
    // else driver = base1_driver;

    driver->set_pose_error(state->position_error, state->orientation_error, &expected_linear, &expected_angular);
    if (abs(state->orientation_error) > 5.f * PI / 180.) expected_linear = 0.f;
    driver->speed_rotation_first(expected_linear, expected_angular);
}

void StrutController::cooperate_triangle_loop(ControlState* state) 
{
    float expected_linear, expected_angular;
    // if (state->is_base2) {
    base2_driver->set_pose_error(state->position_error, state->orientation_error, &expected_linear, &expected_angular);
    // }
    // else {
    //     base1_driver->set_pose_error(state->position_error, state->orientation_error, &expected_linear, &expected_angular);
    // }
    if (state->param->orientation_only) {
        expected_linear = 0.f;
        float angular_base2;
        // if (state->is_base2) {
        //angular_base1 = -expected_angular;
        angular_base2 = expected_angular;
        // }
        // else {
        //     angular_base1 = expected_angular; angular_base2 = -expected_angular;
        // }
        //base1_driver->speed_raw(0.f, angular_base1);
        base2_driver->speed_raw(0.f, angular_base2);
    }
    else {
        if (state->param->straight_mainly) {
            if (abs(state->orientation_error) > 40. * PI / 180.) { // rotate first, not good here should never happend
                expected_linear = 0.f;
                float angular_base2;
                // if (state->is_base2) {
                //angular_base1 = -expected_angular;
                angular_base2 = expected_angular;
                // }
                // else {
                //     angular_base1 = expected_angular; angular_base2 = -expected_angular;
                // }
                //base1_driver->speed_raw(0.f, angular_base1);
                base2_driver->speed_raw(0.f, angular_base2);
            }
            else { // mainly go straight
                expected_angular = 0.f;
                // if (state->is_base2) {
                //     base1_driver->speed_raw(0., 0.);
                base2_driver->speed_raw(expected_linear, 0.);
                // }
                // else {
                //     base1_driver->speed_raw(expected_linear, 0.);
                //     base2_driver->speed_raw(0., 0.);
                // }
            }
        }
        else {
            if (abs(state->orientation_error) > 3. * PI / 180.) { // rotate first, not good here should never happend
                expected_linear = 0.f;
                float angular_base2;
                // if (state->is_base2) {
                //angular_base1 = -expected_angular; angular_base2 = expected_angular;
                // }
                // else {
                //     angular_base1 = expected_angular; angular_base2 = -expected_angular;
                // }
                //base1_driver->speed_raw(0.f, angular_base1);
                base2_driver->speed_raw(0.f, angular_base2);
            }
            else { // mainly go straight
                expected_angular = 0.f;
                // if (state->is_base2) {
                //     base1_driver->speed_raw(0., 0.);
                base2_driver->speed_raw(expected_linear, 0.);
                // }
                // else {
                //     base1_driver->speed_raw(expected_linear, 0.);
                //     base2_driver->speed_raw(0., 0.);
                // }
            }
        }
        
    }
}

bool StrutController::check_filter_status(ControlState* state)
{
    enum FilterMode filter_mode = orientation_filter->get_mode();
    if (filter_mode == BOTH) return true;

    switch (state->mode) {
        case MANUALLY:
            break;

        default:
            if (    state->is_base2 && (filter_mode != LIFTER2 )) return false;
            if ( (!state->is_base2) && (filter_mode != LIFTER1 )) return false;

    }
    return true;
}

void StrutController::calculate_error(ControlState* state) // disconnect might happended, then the orientation is not trustable 
{
    float q0, q1, q2, q3;
    q0 = orientation_filter->q[0];
    q1 = orientation_filter->q[1];
    q2 = orientation_filter->q[2];
    q3 = orientation_filter->q[3];

    enum FilterMode filter_mode = orientation_filter->get_mode();
    if (!state->is_base2 && filter_mode == BOTH) { // base1
        // transform the quat base frame base2 -> base1
        // fh: father node horizontal (base2), ch: child node horizontal (base1)
        // q^{world}_{ch} = q^{world}_{fh} * q^{fh}_{ch}
        // quat_z(yaw) = q^{fh}_{ch}
        // q^{ch}_{s} = q^{ch}{fh} * q^{fh}_{s} = quat_z(-yaw) * q^{fh}_{s}
        float yaw_half = - orientation_filter->get_yaw_angle() / 2.;
        float p0, p3, q0_, q1_, q2_, q3_;
        //float p1 = 0.f, p2 = 0.f;
        p0 = cos(yaw_half);
        p3 = sin(yaw_half);

        //q0_ = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3;
        //q1_ = p1 * q0 + p0 * q1 - p3 * q2 + p2 * q3;
        //q2_ = p2 * q0 + p3 * q1 + p0 * q2 - p1 * q3;
        //q3_ = p3 * q0 - p2 * q1 + p1 * q2 + p0 * q3;
        q0_ = p0 * q0 - p3 * q3;
        q1_ = p0 * q1 - p3 * q2;
        q2_ = p3 * q1 + p0 * q2;
        q3_ = p3 * q0 + p0 * q3;

        q0 = q0_; q1 = q1_; q2 = q2_; q3 = q3_;
    }

    if (state->param->relative_to_node) {
        float node_q0, node_q1, node_q2, node_q3;
        if (state->is_base2) {
            orientation_filter->get_node2_quat(node_q0, node_q1, node_q2, node_q3);
        }
        else {
            orientation_filter->get_node1_quat(node_q0, node_q1, node_q2, node_q3);
        }
        // q^N_S = q^Nh_N* x q^Nh_S 
        float q0_, q1_, q2_, q3_;
        q0_ =  node_q0 * q0 + node_q1 * q1 + node_q2 * q2 + node_q3 * q3;
        q1_ = -node_q1 * q0 + node_q0 * q1 + node_q3 * q2 - node_q2 * q3;
        q2_ = -node_q2 * q0 - node_q3 * q1 + node_q0 * q2 + node_q1 * q3;
        q3_ = -node_q3 * q0 + node_q2 * q1 - node_q1 * q2 + node_q0 * q3;
        q0 = q0_; q1 = q1_; q2 = q2_; q3 = q3_;
    } 

    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float R11, R12, R13, R21, R22, R23, R31, R32, R33;

    R11 = 1.f - _2q2*q2 - _2q3*q3;
    R12 = _2q1*q2 - _2q0*q3;
    R13 = _2q0*q2 + _2q1*q3;
    R21 = _2q1*q2 + _2q0*q3;
    R22 = 1.f - _2q1*q1 - _2q3*q3;
    R23 = _2q2*q3 - _2q0*q1;
    R31 = _2q1*q3 - _2q0*q2;
    R32 = _2q2*q3 + _2q0*q1;
    R33 = 1.f - 2.*q1*q1 - 2.*q2*q2;

    if (!state->is_base2) { // base1 -> inverse y and z axis
      R13 = -R13; R23 = -R23; R33 = -R33;
      R12 = -R12; R22 = -R22; R32 = -R32; 
    }

    float v1, v2, v3;
    v1 = state->param->target_position_x - R13;
    v2 = state->param->target_position_y - R23;
    v3 = state->param->target_position_z - R33;

    float u1, u2; //, u3;
    u1 = R11*v1+R21*v2+R31*v3;
    u2 = R12*v1+R22*v2+R32*v3;
    //u3 = R13*v1+R23*v2+R33*v3;

    float theta = state->param->target_position_x * R13 + state->param->target_position_y * R23 + state->param->target_position_z * R33;
    if (theta >  1.f) theta =  1.f;
    if (theta < -1.f) theta = -1.f;
    state->position_error_accurate = acos(theta);
    
    state->orientation_error = -atan2(u2, u1);
    state->position_error = -state->position_error_accurate * cos(state->orientation_error);

    if (!state->param->absolute_orientation) 
    {
        if (state->orientation_error > PI / 2.) state->orientation_error -= PI;
        else if (state->orientation_error < - PI/ 2.) state->orientation_error += PI;
    }

    if (state->param->orientation_only) {
        state->position_error = 0.;
    }
}

bool StrutController::check_reach_target(ControlState* state)
{
    if (state->param->orientation_only) {
        state->position_error = 0.;
        if (abs(state->orientation_error)  < state->param->error_tolerance) {
            state->orientation_error = 0.;
            return true;
        } 
    }
    else {
      if (state->position_error_accurate < 5.f * PI / 180.) {
        state->orientation_error = 0.;
        if (state->first_approach_target) {
          unsigned long current_time = micros();
          state->approach_time += ((unsigned long) (current_time - state->approach_time_cache));
          state->approach_time_cache = current_time;
        }
        else {
          state->first_approach_target = true;
          state->approach_time_cache = micros();
        }
      }
      else {
        state->first_approach_target = false;
        state->approach_time_cache = 0;
        state->approach_time = 0;
      }

      if (state->position_error_accurate < state->param->error_tolerance || state->approach_time > 4000000) { // timeout 4s
        state->first_approach_target = false;
        state->approach_time_cache = 0;
        state->position_error = 0.;
        return true;
      }
    }
    return false;
}

void StrutController::set_target_position(bool is_base2, enum ControllerMode mode, float x, float y, float z, bool relative_to_node, bool orientation_only, bool absolute_orientation, bool straight_mainly, bool hold, float error_tolerance, uint32_t key)
{
    float norm = sqrt(x * x + y * y + z * z);
    norm = 1.f / norm;
    if (error_tolerance < 1.f) error_tolerance = 1.f;  // can't be less than 1 degree

    ControlParam* param = new ControlParam;
    param->mode = mode;
    param->target_position_x = x * norm;
    param->target_position_y = y * norm;
    param->target_position_z = z * norm;
    param->relative_to_node = relative_to_node;
    param->orientation_only = orientation_only;
    param->absolute_orientation = absolute_orientation;
    param->straight_mainly = straight_mainly;
    param->hold = hold;
    param->error_tolerance = error_tolerance * PI / 180.;
    param->task_key = key;
    switch (mode) {
        case MANUALLY:
            param->resource = TASK_RESOURCE_NONE;
            break;

        case CLOSE_POS_CONTROL:
            if (is_base2) param->resource = TASK_RESOURCE_BASE2;
            else param->resource = TASK_RESOURCE_BASE1;
            break;

        case CO_OPERATE_TRI:
            param->resource = TASK_RESOURCE_BOTH;
            break;
    }

    base2_task_param_queue.push(param);
    // if (is_base2) base2_task_param_queue.push(param);
    // else base1_task_param_queue.push(param);

}

void StrutController::set_target_velocity(bool is_base2, float linear, float angular, uint32_t key)
{
    ControlParam* param = new ControlParam;
    param->mode = CLOSE_VEL_CONTROL;
    param->target_position_x = linear;
    param->target_position_y = angular;
    param->task_key = key;
    // if (is_base2) {
    param->resource = TASK_RESOURCE_BASE2;
    base2_task_param_queue.push(param);
    // } 
    // else {
    //     param->resource = TASK_RESOURCE_BASE1;
    //     base1_task_param_queue.push(param);
    // }
            

}

void StrutController::reset_state(ControlState* state, ControlParam* param)
{
    ControlParam* old_param = state->param;
    state->param = param;
    delete old_param;
    state->reach_target = false;
    state->first_approach_target = false;
    state->approach_time_cache = 0;
    state->approach_time = 0.f;
}

bool StrutController::try_fetch_new_task(ControlState* state)
{
    std::queue<ControlParam*>* param_queue;
    param_queue = &base2_task_param_queue;
    // if (state->is_base2) param_queue = &base2_task_param_queue;
    // else param_queue = &base1_task_param_queue;

    if(param_queue->empty()) return false;

    ControlParam* param = param_queue->front();

    // change task target | assume that the tasks are same if the key is same | may also check source
    if ((param->task_key != 0) && (param->task_key == state->param->task_key) && (param->mode == state->mode))
    {
        reset_state(state, param);
        param_queue->pop();
        return true;
    }

    if (!state->reach_target) return false;
    
    uint8_t resource_occupied = base2_state.resource;
    if (param->resource & resource_occupied) return false;

    // load new task param
    reset_state(state, param);
    switch_mode(state, param->mode);

    param_queue->pop();
    return true;
}

bool StrutController::fast_task_switch(ControlState* state)
{
    std::queue<ControlParam*>* param_queue;
    param_queue = &base2_task_param_queue;
    // if (state->is_base2) param_queue = &base2_task_param_queue;
    // else param_queue = &base1_task_param_queue;

    if(param_queue->empty()) return false;

    ControlParam* param = param_queue->front();
    if (param->mode != state->mode) return false;

    // load new task param
    ControlParam* old_param = state->param;
    state->param = param;
    delete old_param;
    state->reach_target = false;
    state->first_approach_target = false;
    state->approach_time_cache = 0;
    state->approach_time = 0.f;

    param_queue->pop();
    return true;
}
