#ifndef _SONARLIB_H_
#define _SONARLIB_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


struct naze_sensor_state_t
{
    double sim_time;

    double imu_linear_acceleration_x;
    double imu_linear_acceleration_y;
    double imu_linear_acceleration_z;

    double imu_angular_velocity_r;
    double imu_angular_velocity_p;
    double imu_angular_velocity_y;

    double imu_orientation_quat_w;
    double imu_orientation_quat_x;
    double imu_orientation_quat_y;
    double imu_orientation_quat_z;

    double pos_x;
    double pos_y;
    double pos_z;

    double sonar_range;
};


struct naze_actuator_state_t 
{
    float motor[4];
    uint32_t flags;
};


typedef void (*naze_sensor_state_callback_t)(struct naze_sensor_state_t* state);

void register_naze_sensor_state_callback(naze_sensor_state_callback_t cb); 

void start_ros_worker();

int get_sonar_range();

int get_joystick_axis(int axis);

void set_actuator_state(struct naze_actuator_state_t* state);


#ifdef __cplusplus
}
#endif

#endif