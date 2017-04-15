#include "Axis.h"

#define PWM_MAX 0.6

Axis::Axis(
            PinName enc_a, PinName enc_b, PinName enc_i,
            float mot_dir,
            PinName mot_in1, PinName mot_in2, PinName mot_pwm, PinName mot_fb,
            Timer* timer, float pid_rate,
            float pos_pid_p, float pos_pid_i, float pos_pid_d,
            float vel_pid_p, float vel_pid_i, float vel_pid_d, 
            float pos_min, float pos_max, float vel_max,
            float curr_max, float home_pwm
        ):
        qei_(enc_a, enc_b, enc_i, REV_TO_TICKS, timer),
        mot_dir_(mot_dir),
        mot_in1_(mot_in1), mot_in2_(mot_in2), mot_pwm_(mot_pwm), mot_fb_(mot_fb),
        timer_(timer), pid_rate_(pid_rate),
        pos_pid_(pos_pid_p, pos_pid_i, pos_pid_d, pid_rate),
        vel_pid_(vel_pid_p, vel_pid_i, vel_pid_d, pid_rate),
        pos_min_(pos_min), pos_max_(pos_max), vel_max_(vel_max),
        curr_max_(curr_max), home_pwm_(home_pwm),
        zero_ticks_(0), mode_(MODE_NONE), last_time_us_(-1), last_pos_(0.0), last_cmd_(0.0) {
    
    mot_pwm_.period_us(2000);
    
    pos_pid_.setInputLimits(-4.0 * PI, 4.0 * PI);
    pos_pid_.setOutputLimits(-PWM_MAX, PWM_MAX);
    pos_pid_.setBias(0.0);
    
    vel_pid_.setInputLimits(-1.5*vel_max_, 1.5*vel_max_);
    vel_pid_.setOutputLimits(-PWM_MAX, PWM_MAX);
    vel_pid_.setBias(0.0);
}

Axis::AxisMode Axis::get_mode(void) {
    return mode_;
}
    
int Axis::get_ticks(void) {
    return qei_.getPulses();
}

bool Axis::homed(void) {
    return qei_.homed();
}

float Axis::get_position(void) {
    return ((float)(get_ticks() - zero_ticks_) * TICKS_TO_RADIANS) /*+ (last_vel_ * (timer_->read_us() - last_time_us_) / 1000000.0)*/;
}

float Axis::get_last_position(void) {
    return last_pos_;
}

float Axis::get_velocity(void) {
  return TICKS_TO_RADIANS * qei_.getPulseFreq();
}

float Axis::get_last_velocity(void) {
    return last_vel_;
}
        
float Axis::get_current(void) {
    return mot_fb_.read() * FB_TO_CURRENT;
}

float Axis::get_command(void) {
    return last_cmd_;
}

float Axis::get_err_curr(void) {
    return err_curr_;
}

void Axis::get_pid_state(float* state) {
    pos_pid_.get_state(state);
}

void Axis::set_mode(AxisMode mode) {
    if (mode != MODE_NO_CHANGE)
        mode_ = mode;
}

void Axis::set_mode_command(AxisMode mode, float command) {
    set_mode(mode);
    switch(mode) {
        case MODE_PWM:
            set_motor(command);
            break;
        
        case MODE_POS:
            set_position(command);
            break;
        
        case MODE_VEL:
            set_velocity(command);
            break;
    }
}

void Axis::set_motor(float motor) {
    motor = motor * mot_dir_;
    if (motor > 1.0)
        motor = 1.0;
    else if (motor < -1.0)
        motor = -1.0;
        
    if (abs(motor) < 0.001) {
        mot_in1_.write(0);
        mot_in2_.write(0);
        mot_pwm_.write(0.0);   
    } else {
        if (motor > 0.0) {
            mot_in1_.write(0);
            mot_in2_.write(1);
            mot_pwm_.write(motor);
        } else {
            mot_in2_.write(0);
            mot_in1_.write(1);
            mot_pwm_.write(-motor);
        }
    }
    last_cmd_ = motor;
}

void Axis::set_zero(int zero_ticks) {
    zero_ticks_ = zero_ticks;
}
    
void Axis::set_position(float position) {
    if (position < pos_min_)
        position = pos_min_;
    
    if (position > pos_max_) 
        position = pos_max_;
        
    pos_pid_.setSetPoint(position);
}
        
void Axis::set_velocity(float velocity) {
    if (abs(velocity) > vel_max_)
        velocity = velocity > 0.0 ? vel_max_ : - vel_max_;
        
    vel_pid_.setSetPoint(velocity);
}
        
void Axis::update(void) {
    float cmd = 0.0;
    float curr = get_current();
    
    if (curr > curr_max_) {
        mode_ = MODE_ERR;
        err_curr_ = curr;
    }
    
    float pos = get_position();
    float vel = get_velocity();
    last_vel_ = 0.5 * vel + 0.5 * last_vel_;
    
    switch (mode_) {
        case (MODE_NONE):
            set_motor(0.0);
            break;
        
        case (MODE_HOME):
            if (qei_.homed())
                mode_ = MODE_NONE;
            else {
                cmd = home_pwm_;
                set_motor(cmd);
            }
            break;
        
        case (MODE_PWM):
            break;
        
        case (MODE_POS):
            if (abs(pos - pos_pid_.getSetPoint()) < 0.03) {
                cmd = 0.0;
            } else {
                pos_pid_.setProcessValue(pos);
                cmd = pos_pid_.compute();
            }
            set_motor(cmd);
            break;
        
        case (MODE_VEL):
            vel_pid_.setProcessValue(last_vel_);
            cmd = vel_pid_.compute();
            if (cmd > 0.01)
              cmd += 0.1;
            else if (cmd < -0.01)
              cmd -= 0.1;
            set_motor(cmd);
            break;
        
        case (MODE_ERR):
            set_motor(0.0);
            break;
            
        default:
            mode_ = MODE_NONE;
            set_motor(0.0);
            break;
    }
    
    if (mode_ != MODE_ERR)
        last_cmd_ = cmd;
    
    last_pos_ = pos;
    
    last_time_us_ = timer_->read_us();
}
