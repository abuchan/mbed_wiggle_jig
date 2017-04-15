#include "mbed.h"
#include "rtos.h"

#include "Axis.h"
#include "protocol.h"
#include "packet_parser.h"

#define QEI_0_A_PIN p14
#define QEI_0_B_PIN p15
#define QEI_0_I_PIN p13

#define QEI_1_A_PIN p17
#define QEI_1_B_PIN p18
#define QEI_1_I_PIN p16

#define MOT_0_IN1_PIN   p25
#define MOT_0_IN2_PIN   p26
#define MOT_0_PWM_PIN   p24
#define MOT_0_FB_PIN    p19

#define MOT_1_IN1_PIN   p22
#define MOT_1_IN2_PIN   p23
#define MOT_1_PWM_PIN   p21
#define MOT_1_FB_PIN    p20

#define POS_PID_P   100.0
#define POS_PID_I   1.0
#define POS_PID_D   0.0001

#define VEL_PID_P   1.0
#define VEL_PID_I   0.1
#define VEL_PID_D   0.00005

#define PID_RATE    0.001

Timer main_timer;

#define AXIS_0_POS_MIN  (-PI / 4.0)
#define AXIS_0_POS_MAX  (PI / 4.0)

#define VEL_MAX     2.0 * PI
#define CURR_MAX    5.0
#define HOME_PWM    0.25

#define AXIS_0_MOT_DIR -1.0
#define AXIS_1_MOT_DIR 1.0

Axis axis_0(
    QEI_0_A_PIN, QEI_0_B_PIN, QEI_0_I_PIN,
    AXIS_0_MOT_DIR,
    MOT_0_IN1_PIN, MOT_0_IN2_PIN, MOT_0_PWM_PIN, MOT_0_FB_PIN,
    &main_timer, PID_RATE,
    POS_PID_P, POS_PID_I, POS_PID_D,
    VEL_PID_P, VEL_PID_I, VEL_PID_D,
    AXIS_0_POS_MIN, AXIS_0_POS_MAX,
    VEL_MAX, CURR_MAX, -HOME_PWM
);

#define AXIS_1_POS_MIN  (-4.0 * PI)
#define AXIS_1_POS_MAX  (4.0 * PI)

Axis axis_1(
    QEI_1_A_PIN, QEI_1_B_PIN, QEI_1_I_PIN,
    AXIS_1_MOT_DIR,
    MOT_1_IN1_PIN, MOT_1_IN2_PIN, MOT_1_PWM_PIN, MOT_1_FB_PIN,
    &main_timer, PID_RATE,
    POS_PID_P, POS_PID_I, POS_PID_D,
    VEL_PID_P, VEL_PID_I, VEL_PID_D,
    AXIS_1_POS_MIN, AXIS_1_POS_MAX,
    VEL_MAX, CURR_MAX, HOME_PWM
);

Axis* axes[2] = {&axis_0, &axis_1};

//Serial pc(USBTX, USBRX, 230400);

#define SERIAL_BAUDRATE 230400

#define XBEE_TX p9
#define XBEE_RX p10

Serial xbee(XBEE_TX, XBEE_RX);

DigitalOut led_0(LED1), led_1(LED2);

void fill_sensor_packet(packet_t* pkt) {
    pkt->header.type = PKT_TYPE_SENSOR;
    pkt->header.flags = (axis_1.get_mode() << 4) | (axis_0.get_mode());
    pkt->header.length = sizeof(header_t) + sizeof(sensor_data_t) + 1;
    
    sensor_data_t* sensor_data = (sensor_data_t*)pkt->data_crc;
    sensor_data->time = main_timer.read_us();
    for(int i = 0; i<2; i++) {
        sensor_data->position[i] = axes[i]->get_position();
        sensor_data->velocity[i] = axes[i]->get_last_velocity();
        sensor_data->current[i] = axes[i]->get_current();
        sensor_data->pwm[i] = axes[i]->get_command();
    }
}
  
int main(void) {
    main_timer.start();
    int current_time = 0 , last_time = 0;
    
    RtosTimer axis_timer_0(&axis_0, &Axis::update);
    RtosTimer axis_timer_1(&axis_1, &Axis::update);
    
    PacketParser parser(SERIAL_BAUDRATE, USBTX, USBRX, LED3, LED4);
    packet_union_t* sensor_pkt = parser.get_send_packet();
    packet_union_t* recv_pkt = NULL;
    command_data_t* command;
    
    axis_timer_0.start((int)(PID_RATE*1000));
    axis_timer_1.start((int)(PID_RATE*1000));
    
    axis_0.set_mode(Axis::MODE_HOME);
    while(axis_0.get_mode() == Axis::MODE_HOME) {   
        wait(0.1);
        led_0 = !led_0;
    }
    
    wait(0.2);
    
    axis_0.set_zero(1024);
    
    axis_0.set_position(0.0);
    axis_0.set_mode(Axis::MODE_POS);
    
    wait(0.2);
    
    axis_1.set_mode(Axis::MODE_HOME);
    while(axis_1.get_mode() == Axis::MODE_HOME) {  
        wait(0.1);
        led_0 = !led_0;
    }
    
    led_0 = 0;
    
    wait(0.2);
    
    axis_1.set_zero(3500);
    
    axis_1.set_position(0.0);
    axis_1.set_mode(Axis::MODE_POS);
    
    int led_count = 0;
    
    while(1) {
        recv_pkt = parser.get_received_packet();
        
        if (recv_pkt != NULL) {
        
            switch (recv_pkt->packet.header.type) {
            
                case PKT_TYPE_COMMAND:
                    command = (command_data_t*)recv_pkt->packet.data_crc;
                    uint8_t flags = recv_pkt->packet.header.flags;
                    Axis::AxisMode m0 = (Axis::AxisMode)(flags & 0xF);
                    Axis::AxisMode m1 = (Axis::AxisMode)((flags>>4) & 0xF);
                    axis_0.set_mode_command(m0, command->command[0]);
                    axis_1.set_mode_command(m1, command->command[1]);
                    led_0 = !led_0;
                    break;
            }
            
            parser.free_received_packet(recv_pkt);
        }
        
        current_time = main_timer.read_ms();

        if (current_time - last_time > 5) {
            if (sensor_pkt != NULL) {
                fill_sensor_packet(&(sensor_pkt->packet));
                parser.send_packet(sensor_pkt);
                sensor_pkt = parser.get_send_packet();
            }
            last_time = current_time;
            
            if((led_count++) % 100 == 0)
                led_1 = !led_1;
        }
        
        Thread::yield();
    }
}
