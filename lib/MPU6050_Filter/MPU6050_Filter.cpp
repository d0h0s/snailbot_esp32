#include "MPU6050_Filter.h"


MPU6050_Filter::MPU6050_Filter(ESP32Encoder* encoder_2l, ESP32Encoder* encoder_2r, uint32_t eeprom_offset_, uint8_t addr)
:accelgyro(addr)
{
//     _encoder_1l = encoder_1l;
//     _encoder_1r = encoder_1r;
    _encoder_2l = encoder_2l;
    _encoder_2r = encoder_2r;
    eeprom_offset = eeprom_offset_;
}

// void MPU6050_Filter::update_base1_encoder(void)
// {
//     base1_l_count = _encoder_1l->getCount();
//     base1_r_count = _encoder_1r->getCount();

//     float base1_l_delta =  ((float) (base1_l_count - base1_l_last_count)) * MOTOR_RATIO_RAD;
//     float base1_r_delta = -((float) (base1_r_count - base1_r_last_count)) * MOTOR_RATIO_RAD;

//     _delta_e1y =  0.5f * ENCODER_ALPHA * (base1_r_delta + base1_l_delta); // linear delta
//     _delta_e1z = 0.5f * ENCODER_BETA * (base1_r_delta - base1_l_delta); // angular delta

//     base1_l_last_count = base1_l_count;
//     base1_r_last_count = base1_r_count;
// }

void MPU6050_Filter::update_base2_encoder(void)
{
    base2_l_count = _encoder_2l->getCount();
    base2_r_count = _encoder_2r->getCount();

    float base2_l_delta =  ((float) (base2_l_count - base2_l_last_count)) * MOTOR_RATIO_RAD;
    float base2_r_delta = -((float) (base2_r_count - base2_r_last_count)) * MOTOR_RATIO_RAD;

    _delta_e2y =  0.5f * ENCODER_ALPHA * (base2_r_delta + base2_l_delta); // linear delta
    _delta_e2z = 0.5f * ENCODER_BETA * (base2_r_delta - base2_l_delta); // angular delta

    base2_l_last_count = base2_l_count;
    base2_r_last_count = base2_r_count;
}

void MPU6050_Filter::estimate_alpha(void)
{
    float _m1z_abs = abs(_m1z);
    float _m2z_abs = abs(_m2z);
    if (_m1z_abs > ALPHA_ESTIMATE_MZ_TOLERANCE_HARD || _m2z_abs > ALPHA_ESTIMATE_MZ_TOLERANCE_HARD) return;

    float m1xn, m1yn, m2xn, m2yn;
    float norm = 1.f / sqrt(_m1x * _m1x + _m1y * _m1y);
    m1xn = _m1x * norm;
    m1yn = _m1y * norm;
    norm = 1.f / sqrt(_m2x * _m2x + _m2y * _m2y);
    m2xn = _m2x * norm;
    m2yn = _m2y * norm;
    float alpha_base12 = atan2(m1xn * m2yn - m2xn * m1yn, m1xn * m2xn + m1yn * m2yn);
    
    if (angle_filter_initialized) {
        float extra_variance;
        if (_m1z_abs < ALPHA_ESTIMATE_MZ_TOLERANCE_SOFT || _m2z_abs < ALPHA_ESTIMATE_MZ_TOLERANCE_SOFT) {
            extra_variance = 0.f;
        }
        else {
            extra_variance = ALPHA_ESTIMATE_MZ_TOLERANCE_NORM_RATIO * (_m1z_abs - ALPHA_ESTIMATE_MZ_TOLERANCE_SOFT) * (_m2z_abs - ALPHA_ESTIMATE_MZ_TOLERANCE_SOFT);
            extra_variance = extra_variance * extra_variance;
        }
        yaw_anglle_filter.update(alpha_base12, 0.003 + extra_variance * 0.01); 
    }
    else {
        angle_filter_initialized = true;
        yaw_anglle_filter.setAngle(alpha_base12);
    }
}

void MPU6050_Filter::integrate_alpha(float delta_ey, float delta_ez, float deltat, GyrodomCache &gyro_cache)
{
    // force enable wheel odom
    //float gz_e = R32 * delta_ey + R33 * delta_ez;
    //float gz_ = gz_e/deltat;
    float gz_ = gyrodom_cache_integral(delta_ey, delta_ez, deltat, gyro_cache);
    
    float ez_ = gyrodom_world - gz_; // delta_alpha = ez_base2 + ez_base1 = ez_base2 - (-ez_base1)
    yaw_anglle_filter.predict(deltat * ez_, deltat);
}

float MPU6050_Filter::gyrodom_cache_integral(float delta_ey, float delta_ez, float deltat, GyrodomCache &gyro_cache)
{
    // from gyro
    float gz_ = gyroz_world;
    // from wheel odometry
    float gz_e = R32 * delta_ey + R33 * delta_ez;

    unsigned long time = micros();
    bool start_frame = time > gyro_cache.gyro_start_time;
    bool not_end_frame = time < gyro_cache.current.gyro_end_time;
    bool not_end_frame_next = time < gyro_cache.next.gyro_end_time;
    if ( start_frame ) {
        if (not_end_frame) { 
            gyro_cache.current.gyro_integral += gz_ * deltat;
            gyro_cache.current.odom_integral += gz_e;
            gyro_cache.current.integral_t += deltat;
        }
        else if (not_end_frame_next) {
            gyro_cache.next.gyro_integral += gz_ * deltat;
            gyro_cache.next.odom_integral += gz_e;
            gyro_cache.next.integral_t += deltat;
        }
    }

    gz_e /= deltat;

    //Serial.print(gz_, 8);Serial.print("  ");Serial.print(gz_e, 8);Serial.print("  ");Serial.print(abs(gz_-gz_e), 8);Serial.println("");

    gz_ = gyrodometry(gz_, gz_e);

    if ( start_frame ) {
        if (not_end_frame)
            gyro_cache.current.approx_integral += gz_ * deltat;
        else if (not_end_frame_next)
            gyro_cache.next.approx_integral += gz_ * deltat;
    }
    
    return gz_;
}

float MPU6050_Filter::gyrodometry(float gz_, float gz_e) // gyro , wheel odometry
{
    // Gyrodometry: A New Method for Combining Data from Gyros and Odometry in Mobile Robots
    // assume that the node has zero gyro -> relative to node is relative to world
    // TODO when network is bad and recent node gyro is not zero, trust wheel odometry more
    if (abs(gz_ - gz_e) < 0.08 || (abs(gz_) > 0.1 && abs(gz_e) < 0.01) ) // 0.022 in the paper
        return gz_e;

    return gz_;
}

float MPU6050_Filter::get_yaw_angle(void)
{
    return yaw_anglle_filter.getAngle();
}

// void MPU6050_Filter::set_base1_mag(float mx, float my, float mz)
// {
//     _m1x = -mx; // base1 mag points to -z axis, so we invert it
//     _m1y = -my;
//     _m1z = -mz;
//     _m1_t = micros();
//     if (_mode == BOTH) {
//         estimate_alpha();
//     }
//     if (!fast_estimated && (_mode == LIFTER1)) {
//         initial_yaw_guess(_m1x, _m1y, _m1z);
//     }
// }

void MPU6050_Filter::set_base2_mag(float mx, float my, float mz)
{
    _m2x = mx;
    _m2y = my;
    _m2z = mz;
    _m2_t = micros();
    if (_mode == BOTH) {
        estimate_alpha();
    }
    if (!fast_estimated && (_mode == LIFTER2 || _mode == BOTH) ) {
        initial_yaw_guess(_m2x, _m2y, _m2z);
    }
}

// void MPU6050_Filter::set_base1_gyro(uint32_t frame_id, float angle, float duration, int64_t t)
// {
//     if (_mode == LIFTER1 || _mode == BOTH) {
//         float gyro_revise = gyro_revise_estimate(frame_id, angle, duration, t, base1_cache);
//         if (_mode == BOTH) { yaw_anglle_filter.predict(-gyro_revise, duration); }
//     }
// }

void MPU6050_Filter::set_base2_gyro(uint32_t frame_id, float angle, float duration, int64_t t)
{
    if (_mode == LIFTER2 || _mode == BOTH) {
        float gyro_revise = gyro_revise_estimate(frame_id, angle, duration, t, base2_cache);
        if (_mode == BOTH) {yaw_anglle_filter.predict(gyro_revise, duration);}
    }
}

void MPU6050_Filter::set_opt_result(float alpha, float vx, float vy, float vz)
{
    if (_mode != BOTH) return;

    float norm = 1.f / sqrt(vx * vx + vy * vy + vz * vz);
    vx *= norm;
    vy *= norm;
    vz *= norm;
    if (opt_change_rate.empty()) {
        _optx = vx;
        _opty = vy;
        _optz = vz;
        _opt_alpha = alpha;
        _opt_t = micros();
        opt_change_rate.push_back(0.f);
        return;
    }

    float theta = _optx * vx + _opty * vy + _optz * vz;
    if (theta >  1.f) theta =  1.f;
    if (theta < -1.f) theta = -1.f;
    theta = acos(theta);

    theta += abs(alpha - _opt_alpha);

    theta /= ((float) (micros() - _opt_t + 10)) / 1000000.f;  // change_rate

    opt_rate_sum += theta;

    if (opt_change_rate.size() < OPT_CHANGE_RATE_QUEUE_SIZE) {
        opt_change_rate.push_back(theta);
    }
    else {
        opt_rate_sum -= opt_change_rate.front();
        opt_change_rate.pop_front();
        opt_change_rate.push_back(theta);
        Serial.println(opt_rate_sum,8);
        float avg_rate = opt_rate_sum / ((float) OPT_CHANGE_RATE_QUEUE_SIZE);
        if (avg_rate < 0.1) {
            yaw_anglle_filter.update(alpha, 0.001 + avg_rate * 0.02);
            opt_confidence = 100 - avg_rate * 1000;
        }
        else{
            opt_confidence = 0.f;
        }
    }

    _optx = vx;
    _opty = vy;
    _optz = vz;
    _opt_alpha = alpha;
    _opt_t = micros();
    
}

void MPU6050_Filter::clear_opt_buffers(void) 
{
    opt_change_rate.clear();
    opt_rate_sum = 0.f;
    opt_confidence = 0.f;
}

float MPU6050_Filter::gyro_revise_estimate(uint32_t frame_id, float angle, float duration, int64_t t, GyrodomCache &gyro_cache)
{
    //Serial.print(gyro_frame_id);Serial.print(" ");Serial.println(frame_id);
    if (!gyro_cache.gyro_first_revieved) { // TODO check recieved delay time here
        init_gyro_frame(frame_id, duration, t, gyro_cache);
        return 0.;
    }
    if (frame_id < gyro_cache.current.gyro_frame_id) {
        gyro_cache.next.gyro_frame_id = frame_id + 2;
        return 0.;
    }
    else if (frame_id == gyro_cache.current.gyro_frame_id){

    }
    else { // lost frame
        gyro_cache.gyro_first_revieved = false; 
        if (frame_id == gyro_cache.current.gyro_frame_id + 1) { // lost one frame
            Serial.println("lost one frame");
            update_gyro_values(frame_id, duration, t, gyro_cache);
        }
        else { // lost mutiple frame, reset the process
            Serial.println("lost multiple frame");
            return 0.;
        }
    }
    
    if (abs(gyro_cache.current.integral_t - duration) > duration * 0.1 || duration < 0.01) { // duration should be larger than 10 ms
        //Serial.print("error gyro time ");Serial.print(gyro_cache.current.integral_t, 8);Serial.print(" ");Serial.print(" ");Serial.println(duration, 8);
        update_gyro_values(frame_id, duration, t, gyro_cache);
        return 0.;
    }
    float gz_ = (gyro_cache.current.gyro_integral - angle) / duration;
    float gz_e = gyro_cache.current.odom_integral / duration;

    // should use much more complex logic here
    gz_ = gyrodometry(gz_, gz_e);

    gyro_cache.gyro_revise = gz_ * duration - gyro_cache.current.approx_integral;
    update_gyro_values(frame_id, duration, t, gyro_cache);
    return gyro_cache.gyro_revise;
}

void MPU6050_Filter::init_gyro_frame(uint32_t frame_id, float duration, int64_t t, GyrodomCache &gyro_cache)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t current_time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

    unsigned long duration_us = ((unsigned long)(duration * 1.e6));
    if ( (t >= current_time) || (t + duration_us + duration_us < current_time) ) return;  // better be: t + duration_us < current_time
    gyro_cache.gyro_first_revieved = true; 
    unsigned long start_time = current_time - micros();
    gyro_cache.gyro_start_time = ((unsigned long) t) + duration_us - start_time;
    gyro_cache.current.gyro_end_time = gyro_cache.gyro_start_time + duration_us;
    gyro_cache.next.gyro_end_time = gyro_cache.gyro_start_time + duration_us;
    gyro_cache.current.gyro_frame_id = frame_id + 2;
}

void MPU6050_Filter::update_gyro_values(uint32_t frame_id, float duration, int64_t t, GyrodomCache &gyro_cache)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t current_time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

    if (t >= current_time) {
        gyro_cache.gyro_first_revieved = false; 
        return;
    }

    gyro_cache.current.gyro_integral = gyro_cache.next.gyro_integral;
    gyro_cache.current.odom_integral = gyro_cache.next.odom_integral;
    gyro_cache.current.approx_integral = gyro_cache.next.approx_integral;
    gyro_cache.current.integral_t = gyro_cache.next.integral_t;
    gyro_cache.current.gyro_frame_id = gyro_cache.next.gyro_frame_id;
    gyro_cache.current.gyro_end_time = gyro_cache.next.gyro_end_time;
    
    gyro_cache.next.gyro_integral = 0.f;
    gyro_cache.next.odom_integral = 0.f;
    gyro_cache.next.approx_integral = 0.f;
    gyro_cache.next.integral_t = 0.f;
    gyro_cache.next.gyro_frame_id = frame_id + 2;
    
    unsigned long start_time = current_time - micros();
    unsigned long duration_us = ((unsigned long)(duration * 1.e6));
    gyro_cache.gyro_start_time = ((unsigned long) t) - start_time;
    gyro_cache.next.gyro_end_time = gyro_cache.gyro_start_time + duration_us + duration_us;
}

void MPU6050_Filter::initial_yaw_guess(float mx, float my, float mz)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];         // short name local variable for readability
    float q0_, q1_, q2_, q3_;
    float p0, p3;

    float _q0q1__q2q3 = q0*q1 - q2*q3;
    float _q0q2_q1q3  = q0*q2 + q1*q3;

    float sin_theta = mx * _q0q1__q2q3 + my * _q0q2_q1q3;
    float cos_theta = mx * _q0q2_q1q3 - my * _q0q1__q2q3;
    
    if (sin_theta * sin_theta + cos_theta * cos_theta < 4e-3) return;
    //Serial.println(sin_theta * sin_theta + cos_theta * cos_theta, 8);

    float half_theta = atan2(sin_theta, cos_theta) / 2.f;
    p0 = cos(half_theta); p3 = sin(half_theta);

    q0_ = p0 * q0 - p3 * q3;
    q1_ = p0 * q1 - p3 * q2;
    q2_ = p3 * q1 + p0 * q2;
    q3_ = p3 * q0 + p0 * q3;

    float norm = sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);    // normalise quaternion, may not needed here
    norm = 1.0f / norm;
    q[0] = q0_ * norm;
    q[1] = q1_ * norm;
    q[2] = q2_ * norm;
    q[3] = q3_ * norm;

    fast_estimated = true;
}

void MPU6050_Filter::fast_gravity_align(float ax, float ay, float az) // vec should be normalized
{
    return;
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];         // short name local variable for readability
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float f1, f2, f3;
    float R31, R32, R33;
    float grad0, grad1, grad2, grad3;
    float norm;
    float step = beta_dynamic * 0.01;
    
    unsigned long start_t = micros();
    while(micros() - start_t < 1e4) 
    {
        _2q0 = 2.f * q0;
        _2q1 = 2.f * q1;
        _2q2 = 2.f * q2;
        _2q3 = 2.f * q3;

        R31 = _2q1 * q3 - _2q0 * q2;
        R32 = _2q0 * q1 + _2q2 * q3;
        R33 = 1.0f - _2q1 * q1 - _2q2 * q2;

        if (R31 * ax + R32 * ay + R33 * az > 0.95) break;

        f1 = R31 - ax;
        f2 = R32 - ay;
        f3 = R33 - az;

        grad0 =  _2q1 * f2 - _2q2 * f1;
        grad1 = _2q3 * f1 + _2q0 * f2 - 2.0 * _2q1 * f3;
        grad2 = _2q3 * f2 - _2q0 * f1 - 2.0 * _2q2 * f3;
        grad3 =  _2q1 * f1 + _2q2 * f2;

        norm = sqrt(grad0 * grad0 + grad1 * grad1 + grad2 * grad2 + grad3 * grad3);
        if (norm < 1.e-5)
        {
            grad0 = 0.;
            grad1 = 0.;
            grad2 = 0.;
            grad3 = 0.;
        }
        else
        {
            norm = 1.0/norm;
            grad0 *= norm;
            grad1 *= norm;
            grad2 *= norm;
            grad3 *= norm;
        }

        q0 -= grad0 * step;
        q1 -= grad1 * step;
        q2 -= grad2 * step;
        q3 -= grad3 * step;

        norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        norm = 1.0/norm;
        q0 = q0 * norm;
        q1 = q1 * norm;
        q2 = q2 * norm;
        q3 = q3 * norm;
    }
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

void MPU6050_Filter::update_mode(int lifter1_node_id, int lifter2_node_id)
{
    if (lifter1_node_id == -1 && lifter2_node_id == -1) {
        _mode = FREE;
    }
    else {
        fast_estimated = false;

        if (lifter2_node_id == -1) { // only lifter1 connected
            _mode = LIFTER1;
            angle_filter_initialized = false;
            base2_cache.gyro_first_revieved = false;
            //stabled = false;
            firstUpdate = micros();
            _m1x = 0.f, _m1y = 0.f, _m1z = 1.f;
        }
        else {
            if (lifter1_node_id == -1) { // only lifter2 connected
                _mode = LIFTER2;
                angle_filter_initialized = false;
                //base1_cache.gyro_first_revieved = false;
                //stabled = false;
                firstUpdate = micros();
                _m2x = 0.f, _m2y = 0.f, _m2z = 1.f;
            }
            else { // both connected
                clear_opt_buffers();
                _m1x = 0.f, _m1y = 0.f, _m1z = 1.f;
                _m2x = 0.f, _m2y = 0.f, _m2z = 1.f;
                _mode = BOTH;
            }
        }
    }
    
    Serial.print("Update filter mode: ");Serial.println(_mode);
}

void MPU6050_Filter::init(void)
{
    accelgyro.initialize();
    accelgyro.setRate(7); // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    accelgyro.setExternalFrameSync(0); // Disable FSYNC
    accelgyro.setDLPFMode(0); // Set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // Set Gyro Full Scale Range to ±250deg/s
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // Set Accelerometer Full Scale Range to ±2g

    if (EEPROM.readByte(eeprom_offset) == IMU_CALIBRATE_OK) {
        accelgyro.setXAccelOffset(EEPROM.readShort(eeprom_offset + 1));
        accelgyro.setYAccelOffset(EEPROM.readShort(eeprom_offset + 3));
        accelgyro.setZAccelOffset(EEPROM.readShort(eeprom_offset + 5));
        
        accelgyro.setXGyroOffset(EEPROM.readShort(eeprom_offset + 7));
        accelgyro.setYGyroOffset(EEPROM.readShort(eeprom_offset + 9));
        accelgyro.setZGyroOffset(EEPROM.readShort(eeprom_offset + 11));
    }
    else {
        accelgyro.setXAccelOffset(0);
        accelgyro.setYAccelOffset(0);
        accelgyro.setZAccelOffset(0);
        
        accelgyro.setXGyroOffset(0);
        accelgyro.setYGyroOffset(0);
        accelgyro.setZGyroOffset(0);
    }
    Serial.print("Your offsets:\t");
    Serial.print(EEPROM.readShort(eeprom_offset + 1));
    Serial.print("\t");
    Serial.print(EEPROM.readShort(eeprom_offset + 3));
    Serial.print("\t");
    Serial.print(EEPROM.readShort(eeprom_offset + 5));
    Serial.print("\t");
    Serial.print(EEPROM.readShort(eeprom_offset + 7));
    Serial.print("\t");
    Serial.print(EEPROM.readShort(eeprom_offset + 9));
    Serial.print("\t");
    Serial.println(EEPROM.readShort(eeprom_offset + 11));

    timer = micros();
    firstUpdate = micros();
}

void MPU6050_Filter::reset_timer(void)
{
    timer = micros();
}

void MPU6050_Filter::loop(void)
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // TODO handle timer overflow
    unsigned long dt_us = (unsigned long)(micros() - timer);
    dt = ((float)(dt_us)) / 1000000.f; // Calculate delta time, in seconds
    timer = micros();

    // convert data from sensor frame to body frame  X -> z ; Y -> y ; X -> -x
    //accX = (float) az; accY = (float) -ay; accZ = (float) ax; // v1.0 PCB
    accX = (float) az; accY = (float) ay; accZ = (float) -ax; 
    
    gyroXrate = ((float) gz) / 131.072; // Convert to deg/s
    gyroYrate = ((float) gy) / 131.072; // Convert to deg/s
    gyroZrate = ((float) -gx) / 131.072; // Convert to deg/s

    if ((abs(gyroXrate) < 0.6) & (abs(gyroYrate) < 0.6) & (abs(gyroZrate) < 0.6))  // assume sensor is static if gyro values are small
    {
        gyroXrate = 0.;
        gyroYrate = 0.;
        gyroZrate = 0.;
        //if (stabled) beta = beta_static;
    }
    else
    {
        gyroXrate *= PI/180.;
        gyroYrate *= PI/180.;
        gyroZrate *= PI/180.;
        if (stabled) beta = beta_dynamic;
    }

    if ( (!stabled) & ((unsigned long)(timer - firstUpdate) > 5000000)) {
        stabled = true;
    }

    float mt, m_ratio = 1.0f;
    
    switch (_mode) {
        case FREE:
            MadgwickQuaternionUpdate6(accX, accY, accZ, gyroXrate, gyroYrate, gyroZrate, dt);
            break;
        case LIFTER1:
            //update_base1_encoder();
            mt = ((float)((unsigned long)(timer - _m1_t))) / 1000.f; // ms
            //if (mt > 300) // don't trust mag data if there are no new data after 300 ms, TODO add time decay
            //    m_ratio = 0.f;
            MadgwickQuaternionUpdate9(accX, accY, accZ, gyroXrate, gyroYrate, gyroZrate, _m1x, _m1y, _m1z, m_ratio, -_delta_e1y, -_delta_e1z, dt, base2_cache); // convert encoder data to imu frame
            //initial_yaw_guess(_m1x, _m1y, _m1z);
            break;
        case LIFTER2:
            update_base2_encoder();
            mt = ((float)((unsigned long)(timer - _m2_t))) / 1000.f; // ms
            //if (mt > 300) // don't trust mag data if there are no new data after 300 ms, TODO add time decay
            //    m_ratio = 0.f;
            MadgwickQuaternionUpdate9(accX, accY, accZ, gyroXrate, gyroYrate, gyroZrate, _m2x, _m2y, _m2z, m_ratio, _delta_e2y, _delta_e2z, dt, base2_cache);
            //initial_yaw_guess(_m2x, _m2y, _m2z);
            break;
        case BOTH:
            //update_base1_encoder();
            update_base2_encoder();
            float ot, o_ratio, ratio;
            ot = ((float)((unsigned long)(timer - _opt_t))) / 1000.f; // ms
            if (ot > 400.f) o_ratio = 0.f;
            else o_ratio = opt_confidence;

            float vx, vy, vz, norm;
            vx = m_ratio * _m2x + o_ratio * _optx;
            vy = m_ratio * _m2y + o_ratio * _opty;
            vz = m_ratio * _m2z + o_ratio * _optz;
            ratio = sqrt(vx * vx + vy * vy + vz * vz);
            norm = 1.f / ratio;
            vx *= norm;
            vy *= norm;
            vz *= norm;
            //Serial.print(vx,6);Serial.print(" ");Serial.print(vy,6);Serial.print(" ");Serial.print(vz,6);Serial.print(" ");
            //Serial.print(_m2x,6);Serial.print(" ");Serial.print(_m2y,6);Serial.print(" ");Serial.print(_m2z,6);Serial.print(" ");
            //Serial.print(_optx,6);Serial.print(" ");Serial.print(_opty,6);Serial.print(" ");Serial.print(_optz,6);Serial.println(" ");
            
            // the estimated orientation is related to base2, we don't fuse mag1 & mag2 for strut orientation here.
            MadgwickQuaternionUpdate9(accX, accY, accZ, gyroXrate, gyroYrate, gyroZrate, vx, vy, vz, 1.f, _delta_e2y, _delta_e2z, dt, base2_cache);
            // the z axis difference of mag1 & mag2 is used to estimate alpha when update mag1/2.
            // intergate alpha by the wheel encoders
            integrate_alpha(-_delta_e1y, -_delta_e1z, dt, base2_cache);
            break;
    }

}

void MPU6050_Filter::meansensors(int buffersize, int* mean_ax, int* mean_ay, int* mean_az, int* mean_gx, int* mean_gy, int* mean_gz)
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (buffersize + 100))
        { //First 100 measures are discarded
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        if (i == (buffersize + 100))
        {
            *mean_ax = buff_ax / buffersize;
            *mean_ay = buff_ay / buffersize;
            *mean_az = buff_az / buffersize;
            *mean_gx = buff_gx / buffersize;
            *mean_gy = buff_gy / buffersize;
            *mean_gz = buff_gz / buffersize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}


void MPU6050_Filter::calibrate(void)
{
    int buffersize = CALIB_BUFFER_SIZE; 
    int acel_deadzone = CALIB_ACEl_DEADZONE; 
    int giro_deadzone = CALIB_GIRO_DEADZONE; 

    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
    
#ifdef CALIB_SERIAL_DEBUG
    Serial.println("\nReading sensors for first time...");
#endif
    meansensors(buffersize, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
    delay(1000);
#ifdef CALIB_SERIAL_DEBUG
    Serial.println("\nCalculating offsets...");
#endif

    int ax_baseline = 0, ay_baseline = 0, az_baseline = 0; // auto calibration
    /*if ( abs(mean_ax) > abs(mean_ay) ) {
        if ( abs(mean_ax) > abs(mean_az) ) ax_baseline = 16384; // x axis gravity
        else az_baseline = 16384; // z axis gravity
    }
    else {
        if ( abs(mean_ay) > abs(mean_az) ) ay_baseline = 16384; // y axis gravity
        else az_baseline = 16384; // z axis gravity
    }
    if (mean_ax < 0) ax_baseline = -ax_baseline;
    if (mean_ay < 0) ay_baseline = -ay_baseline;
    if (mean_az < 0) az_baseline = -az_baseline;*/
    ax_baseline = -16384;
#ifdef CALIB_SERIAL_DEBUG
    Serial.print("\ngravity baseline:\t");
    Serial.print(ax_baseline); Serial.print("\t");
    Serial.print(ay_baseline); Serial.print("\t");
    Serial.println(az_baseline);
#endif

    ax_offset = (ax_baseline - mean_ax) / 8;
    ay_offset = (ay_baseline - mean_ay) / 8;
    az_offset = (az_baseline - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;
    while (1)
    {
        int ready = 0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);

        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        meansensors(buffersize, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
#ifdef CALIB_SERIAL_DEBUG
        Serial.println("...");
#endif
        if (abs(ax_baseline - mean_ax) <= acel_deadzone) ready++;
        else ax_offset = ax_offset + (ax_baseline - mean_ax) / acel_deadzone;

        if (abs(ay_baseline - mean_ay) <= acel_deadzone) ready++;
        else ay_offset = ay_offset + (ay_baseline - mean_ay) / acel_deadzone;

        if (abs(az_baseline - mean_az) <= acel_deadzone) ready++;
        else az_offset = az_offset  + (az_baseline - mean_az) / acel_deadzone;

        if (abs(mean_gx) <= giro_deadzone) ready++;
        else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

        if (abs(mean_gy) <= giro_deadzone) ready++;
        else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

        if (abs(mean_gz) <= giro_deadzone) ready++;
        else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

#ifdef CALIB_SERIAL_DEBUG
        Serial.print(abs(ax_baseline - mean_ax));Serial.print(" ");
        Serial.print(abs(ay_baseline - mean_ay));Serial.print(" ");
        Serial.print(abs(az_baseline - mean_az));Serial.print(" ");
        Serial.print(abs(mean_gx));Serial.print(" ");
        Serial.print(abs(mean_gy));Serial.print(" ");
        Serial.print(abs(mean_gz));Serial.println(" ");
#endif
        if (ready >= 6) break;
    }
    delay(1000);

#ifdef CALIB_SERIAL_DEBUG
    meansensors(buffersize, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); Serial.print("\t");
    Serial.print(mean_ay); Serial.print("\t");
    Serial.print(mean_az); Serial.print("\t");
    Serial.print(mean_gx); Serial.print("\t");
    Serial.print(mean_gy); Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset); Serial.print("\t");
    Serial.print(ay_offset); Serial.print("\t");
    Serial.print(az_offset); Serial.print("\t");
    Serial.print(gx_offset); Serial.print("\t");
    Serial.print(gy_offset); Serial.print("\t");
    Serial.println(gz_offset);
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.print("Check that your sensor readings are close to ");
    Serial.print(ax_baseline); Serial.print("\t");
    Serial.print(ay_baseline); Serial.print("\t");
    Serial.print(az_baseline);
    Serial.println(" 0 0 0");
    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
#endif

    EEPROM.writeByte(eeprom_offset, IMU_CALIBRATE_OK);
    EEPROM.writeShort(eeprom_offset + 1, ax_offset);
    EEPROM.writeShort(eeprom_offset + 3, ay_offset);
    EEPROM.writeShort(eeprom_offset + 5, az_offset);
    
    EEPROM.writeShort(eeprom_offset + 7, gx_offset);
    EEPROM.writeShort(eeprom_offset + 9, gy_offset);
    EEPROM.writeShort(eeprom_offset + 11, gz_offset);
    EEPROM.commit();

    timer = micros();
}

void MPU6050_Filter::MadgwickQuaternionUpdate6(float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                        // objective funcyion elements
    float grad0, grad1, grad2, grad3;
    float qDotw0, qDotw1, qDotw2, qDotw3;
    float qDotwYaw0, qDotwYaw1, qDotwYaw2, qDotwYaw3;
    float temp0, temp1, temp2, temp3, tempn;
    float R11,R12,R13,R21,R22,R23,R31,R32,R33;
    //float gerrx, gerry, gerrz;  // gyro bias error
    
    // Auxiliary variables to avoid repeated arithmetic
    float _halfq0 = 0.5f * q0;
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    R31 = _2q1 * q3 - _2q0 * q2;
    R32 = _2q0 * q1 + _2q2 * q3;
    R33 = 1.0f - _2q1 * q1 - _2q2 * q2;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    bool valid_gravity = (bool) norm > 1.e-5f;
    if (valid_gravity) // handle NaN
    {
        norm = 1.0f/norm;
        ax *= norm;
        ay *= norm;
        az *= norm;
        
        // Compute the objective function and Jacobian 

        f1 = R31 - ax;
        f2 = R32 - ay;
        f3 = R33 - az;

        // Compute the gradient (matrix multiplication)
        grad0 =  _2q1 * f2 - _2q2 * f1;
        grad1 = _2q3 * f1 + _2q0 * f2 - 2.0f * _2q1 * f3;
        grad2 = _2q3 * f2 - _2q0 * f1 - 2.0f * _2q2 * f3;
        grad3 =  _2q1 * f1 + _2q2 * f2;
        
        // Normalize the gradient
        norm = sqrt(grad0 * grad0 + grad1 * grad1 + grad2 * grad2 + grad3 * grad3);
        if (norm < 1.e-5)
        {
            grad0 = 0.;
            grad1 = 0.;
            grad2 = 0.;
            grad3 = 0.;
        }
        else
        {
            norm = 1.0f/norm;
            grad0 *= norm;
            grad1 *= norm;
            grad2 *= norm;
            grad3 *= norm;
        }
        
    }
    else
    {
        grad0 = 0.;
        grad1 = 0.;
        grad2 = 0.;
        grad3 = 0.;
    }
    
    // Compute estimated gyroscope biases  // only applicable to a MARG implementation
    /*gerrx = _2q0 * grad1 - _2q1 * grad0 - _2q2 * grad3 + _2q3 * grad2;
    gerry = _2q0 * grad2 + _2q1 * grad3 - _2q2 * grad0 - _2q3 * grad1;
    gerrz = _2q0 * grad3 - _2q1 * grad2 + _2q2 * grad1 - _2q3 * grad0;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;*/

    /*R11 = 1.f - _2q2*q2 - _2q3*q3;
    R12 = _2q1*q2 - _2q0*q3;
    R13 = _2q0*q2 + _2q1*q3;
    R21 = _2q1*q2 + _2q0*q3;
    R22 = 1.f - _2q1*q1 - _2q3*q3;
    R23 = _2q2*q3 - _2q0*q1;
    //R31 = _2q1*q3 - _2q0*q2;
    //R32 = _2q2*q3 + _2q0*q1;

    temp1 = R11 * gx + R12 * gy + R13 * gz;
    temp2 = R21 * gx + R22 * gy + R23 * gz;

    gx = R11 * temp1 + R21 * temp2;
    gy = R12 * temp1 + R22 * temp2;
    gz = R13 * temp1 + R23 * temp2;*/
    
    // Compute the quaternion derivative
    qDotw0 = -_halfq1 * gx - _halfq2 * gy - _halfq3 * gz;
    qDotw1 =  _halfq0 * gx + _halfq2 * gz - _halfq3 * gy;
    qDotw2 =  _halfq0 * gy - _halfq1 * gz + _halfq3 * gx;
    qDotw3 =  _halfq0 * gz + _halfq1 * gy - _halfq2 * gx;

    /*Serial.print(qDotw0); Serial.print("\t");
    Serial.print(qDotw1); Serial.print("\t");
    Serial.print(qDotw2); Serial.print("\t");
    Serial.print(qDotw3); Serial.print("\t");*/
    

    //beta = 0.;

    // Compute then integrate estimated quaternion derivative
    q0 += (qDotw0 - (beta * grad0)) * deltat;
    q1 += (qDotw1 - (beta * grad1)) * deltat;
    q2 += (qDotw2 - (beta * grad2)) * deltat;
    q3 += (qDotw3 - (beta * grad3)) * deltat;

    /*Serial.print(q0); Serial.print("\t");
    Serial.print(q1); Serial.print("\t");
    Serial.print(q2); Serial.print("\t");
    Serial.print(q3); Serial.print("\t");
    Serial.print("\r\n");*/

    // Normalize the quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q0 * norm;
    q[1] = q1 * norm;
    q[2] = q2 * norm;
    q[3] = q3 * norm;

    /*qDotwYaw0 = 2. - 2. * (q[2]*q[2] + q[3]*q[3]);  // remove the yaw angle part in quat
    qDotwYaw1 = 0.f;
    qDotwYaw2 = 0.f;
    qDotwYaw3 = 2.f * (q[1]*q[2] + q[0]*q[3]);

    norm = sqrt(qDotwYaw0 * qDotwYaw0 + qDotwYaw3 * qDotwYaw3);
    norm = 1.0f/norm;
    qDotwYaw0 *= norm;
    qDotwYaw3 *= -norm;

    temp0 = qDotwYaw0 * q[0] - qDotwYaw3 * q[3];
    temp1 = qDotwYaw0 * q[1] - qDotwYaw3 * q[2];
    temp2 = qDotwYaw0 * q[2] + qDotwYaw3 * q[1];
    temp3 = qDotwYaw0 * q[3] + qDotwYaw3 * q[0];

    q[0] = temp0;
    q[1] = temp1;
    q[2] = temp2;
    q[3] = temp3;*/
    // if (valid_gravity && (R31 * ax + R32 * ay + R33 * az < 0.95)) // 18.2 deg - exists large error, faster gradient descent, can hardly work due to noise of a
    // {
    //     fast_gravity_align(ax, ay, az);
    // }
}


void MPU6050_Filter::MadgwickQuaternionUpdate9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float decay, float delta_ey, float delta_ez, float deltat, GyrodomCache &gyro_cache)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3, f4, f5, f6;                                        // objective funcyion elements
    float grad0, grad1, grad2, grad3;
    float qDotw0, qDotw1, qDotw2, qDotw3;
    float gx_, gy_, gz_, gz_e;

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq0 = 0.5f * q0;
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    float _2q0q1 = _2q0*q1;
    float _2q0q2 = _2q0*q2;
    float _2q0q3 = _2q0*q3;
    float _2q1q2 = _2q1*q2;
    float _2q1q3 = _2q1*q3;
    float _2q2q3 = _2q2*q3;
    float _2q1q1 = _2q1*q1;
    float _2q2q2 = _2q2*q2;
    float _2q3q3 = _2q3*q3;
    
    R11 = 1.0f - _2q2q2 - _2q3q3;
    R12 = _2q1q2 - _2q0q3;
    R13 = _2q0q2 + _2q1q3;
    R21 = _2q1q2 + _2q0q3;
    R22 = 1.0f - _2q1q1 - _2q3q3;
    R23 = _2q2q3 - _2q0q1;
    R31 = _2q1q3 - _2q0q2;
    R32 = _2q2q3 + _2q0q1;
    R33 = 1.0f - _2q1q1 - _2q2q2;

    float f2_p_f5, f2_m_f5, f4_p_f1, f4_m_f1, f3_p_f6;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    bool valid_gravity = (bool) norm > 1e-5;
    if (valid_gravity) // handle NaN
    {
        norm = 1.0f/norm;
        ax *= norm;
        ay *= norm;
        az *= norm;
        
        // Compute the objective function and Jacobian    
        f1 = R31 - ax;
        f2 = R32 - ay;
        f3 = R33 - az;
    }
    else
    {
        f1 = 0.;
        f2 = 0.;
        f3 = 0.;
    }

    float abs_mz = abs(mz);
    float m_confidence = decay;
    if (abs_mz > 0.84) m_confidence = decay * (1.f - abs_mz) * 3.f;
    //if (abs(mz) > 0.99) m_confidence = 0.0f;

    f4 = (R13 - mx) * m_confidence;
    f5 = (R23 - my) * m_confidence;
    f6 = (R33 - mz) * m_confidence;

    f2_p_f5 = f2 + f5;
    f2_m_f5 = f2 - f5;
    f4_p_f1 = f4 + f1;
    f4_m_f1 = f4 - f1;
    f3_p_f6 = f3 + f6;

    // Compute the gradient (matrix multiplication)
    grad0 = _2q1 * f2_m_f5 + _2q2 * f4_m_f1;
    grad1 = _2q0 * f2_m_f5 + _2q3 * f4_p_f1 - 2.0f * _2q1 * f3_p_f6;
    grad2 = _2q0 * f4_m_f1 + _2q3 * f2_p_f5 - 2.0f * _2q2 * f3_p_f6;
    grad3 = _2q1 * f4_p_f1 + _2q2 * f2_p_f5;
    
    // Normalize the gradient
    norm = sqrt(grad0 * grad0 + grad1 * grad1 + grad2 * grad2 + grad3 * grad3);
    if (norm < 1e-5)
    {
        grad0 = 0.;
        grad1 = 0.;
        grad2 = 0.;
        grad3 = 0.;
    }
    else
    {
        norm = 1.0f/norm;
        grad0 *= norm;
        grad1 *= norm;
        grad2 *= norm;
        grad3 *= norm;
    }

    // Compute estimated gyroscope biases  // only applicable to a MARG implementation
    /*gerrx = _2q0 * grad1 - _2q1 * grad0 - _2q2 * grad3 + _2q3 * grad2;
    gerry = _2q0 * grad2 + _2q1 * grad3 - _2q2 * grad0 - _2q3 * grad1;
    gerrz = _2q0 * grad3 - _2q1 * grad2 + _2q2 * grad1 - _2q3 * grad0;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;*/

    // gyro in world frame
    gx_ = R11 * gx + R12 * gy + R13 * gz;
    gy_ = R21 * gx + R22 * gy + R23 * gz;

    // from gyro z
    gz_ = R31 * gx + R32 * gy + R33 * gz;

    gyroz_world = gz_; // cache the gz_ for yaw angle filter

    // cache the parameters for gyrodom revise and apply gyrodometry
    gz_ = gyrodom_cache_integral(delta_ey, delta_ez, deltat, gyro_cache); 

    gyrodom_world = gz_; // cache the value and update together in yaw angle intergral

    gz_ += gyro_cache.gyro_revise / deltat; // gyro_revise should be set before imu loop
    gyro_cache.gyro_revise = 0.f;

    gx = R11 * gx_ + R21 * gy_ + R31 * gz_;
    gy = R12 * gx_ + R22 * gy_ + R32 * gz_;
    gz = R13 * gx_ + R23 * gy_ + R33 * gz_;

    // Compute the quaternion derivative
    qDotw0 = -_halfq1 * gx - _halfq2 * gy - _halfq3 * gz;
    qDotw1 =  _halfq0 * gx + _halfq2 * gz - _halfq3 * gy;
    qDotw2 =  _halfq0 * gy - _halfq1 * gz + _halfq3 * gx;
    qDotw3 =  _halfq0 * gz + _halfq1 * gy - _halfq2 * gx;

    q0 += (qDotw0 - (beta * grad0)) * deltat;
    q1 += (qDotw1 - (beta * grad1)) * deltat;
    q2 += (qDotw2 - (beta * grad2)) * deltat;
    q3 += (qDotw3 - (beta * grad3)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q0 * norm;
    q[1] = q1 * norm;
    q[2] = q2 * norm;
    q[3] = q3 * norm;

    if ((decay > 0.8) && (R13 * mx + R23 * my + R33 * mz < 0.95)) // 18.2 deg re-estimate yaw angle if exists large error
    {
        initial_yaw_guess(mx, my, mz);
    }

    // if (valid_gravity && (R31 * ax + R32 * ay + R33 * az < 0.95)) // 18.2 deg re-estimate yaw angle if exists large error
    // {
    //     fast_gravity_align(ax, ay, az);
    // }
}
