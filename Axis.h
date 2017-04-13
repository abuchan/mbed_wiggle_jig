#ifndef AXIS_H
#define AXIS_H

#include "QEI.h"
#include "PID.h"

#define PI                  3.141592653589793
#define REV_TO_TICKS        (4096 * 2)
#define TICKS_TO_RADIANS    (2.0 * PI / REV_TO_TICKS)
#define FB_TO_CURRENT       (5.0 / 0.525)

class Axis {
    public:
        typedef enum AxisMode {
            MODE_NONE, MODE_HOME, MODE_PWM, MODE_POS, MODE_VEL, MODE_ERR, MODE_NO_CHANGE
        } AxisMode;
        
        Axis(
            PinName enc_a, PinName enc_b, PinName enc_i,
            float mot_dir,
            PinName mot_in1, PinName mot_in2, PinName mot_pwm, PinName mot_fb,
            Timer* timer, float pid_rate,
            float pos_pid_p, float pos_pid_i, float pos_pid_d,
            float vel_pid_p, float vel_pid_i, float vel_pid_d, 
            float pos_min, float pos_max, float vel_max,
            float curr_max, float home_pwm
        );
        
        AxisMode get_mode(void);
        
        int get_ticks(void);
        
        bool homed(void);
        
        float get_position(void);
        
        float get_last_position(void);
        
        float get_velocity(void);
        
        float get_last_velocity(void);
        
        float get_current(void);
        
        float get_command(void);
        
        float get_err_curr(void);
        
        void get_pid_state(float* state);
        
        void set_mode(AxisMode mode);
        
        void set_mode_command(AxisMode mode, float command);
        
        void set_motor(float motor);
        
        void set_zero(int zero_ticks);
        
        void set_position(float position);
        
        void set_velocity(float velocity);
        
        void update(void);
    
    private:
        AxisMode mode_;
        Timer* timer_;
        float mot_dir_, pid_rate_, pos_min_, pos_max_, vel_max_, curr_max_, err_curr_, home_pwm_;
        
        int zero_ticks_;
        int last_time_us_;
        float last_pos_, last_vel_, last_cmd_;
        
        QEI qei_;
        DigitalOut mot_in1_, mot_in2_;
        PwmOut mot_pwm_;
        AnalogIn mot_fb_;
        PID pos_pid_, vel_pid_;
};

#endif