#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "nazecom/MotorControl.h"
#include <stdint.h>
#include "nazecom.h"

#ifdef __cplusplus
extern "C"
{
#endif

    static pthread_t ros_thread;
    static ros::NodeHandlePtr node;
    static ros::Publisher motor_pub;
    static int joystick_axix[4];
    static struct naze_sensor_state_t sensor_state;
    static naze_sensor_state_callback_t sensor_state_callback = NULL;


    void register_naze_sensor_state_callback(naze_sensor_state_callback_t cb)
    {
        printf("register_naze_sensor_state_callback %p\n", cb);
        sensor_state_callback = cb;
    }


    int get_joystick_axis(int i)
    {
        return joystick_axix[i];
    }


    int get_sonar_range()
    {
        //printf("get_range: %d\n", sonar_range);
        return (int)(sensor_state.sonar_range * 100);
    }


    void sonar_update(const sensor_msgs::Range::ConstPtr &msg)
    {
        //printf("sonar_update: %f\n", msg->range);
        sensor_state.sonar_range = msg->range;
    }


    void imu_update(const sensor_msgs::Imu::ConstPtr &msg)
    {
        sensor_state.imu_linear_acceleration_x = msg->linear_acceleration.x;
        sensor_state.imu_linear_acceleration_y = msg->linear_acceleration.y;
        sensor_state.imu_linear_acceleration_z = msg->linear_acceleration.z;
        sensor_state.imu_angular_velocity_r = msg->angular_velocity.x;
        sensor_state.imu_angular_velocity_p = msg->angular_velocity.y;
        sensor_state.imu_angular_velocity_y = msg->angular_velocity.z;
        sensor_state.imu_orientation_quat_w = msg->orientation.w;
        sensor_state.imu_orientation_quat_x = msg->orientation.x;
        sensor_state.imu_orientation_quat_y = msg->orientation.y;
        sensor_state.imu_orientation_quat_z = msg->orientation.z;
    }


    void pos_update(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        sensor_state.pos_x = msg->x;
        sensor_state.pos_y = msg->y;
        sensor_state.pos_z = msg->z;
    }


    void simtime_update(const std_msgs::Float64::ConstPtr &msg)
    {
        sensor_state.sim_time = msg->data;
        //printf("simtime_update %f, %p\n", sensor_state.sim_time, sensor_state_callback);
        if (sensor_state_callback)
        {
            sensor_state_callback(&sensor_state);
        }
    }


    static int scale_joystick(float val)
    {
        return (int)((500.0 * val) + 1500);
    }


    void joystick_update(const sensor_msgs::Joy::ConstPtr &msg)
    {
        joystick_axix[0] = scale_joystick(msg->axes[0]);
        joystick_axix[1] = scale_joystick(msg->axes[1]);
        joystick_axix[2] = scale_joystick(msg->axes[2]);
        joystick_axix[3] = scale_joystick(msg->axes[3]);
    }


    void set_actuator_state(struct naze_actuator_state_t* state)
    {
        nazecom::MotorControl msg;
        msg.m0 = state->motor[0];
        msg.m1 = state->motor[1];
        msg.m2 = state->motor[2];
        msg.m3 = state->motor[3];
        msg.flags = state->flags;
        motor_pub.publish(msg);
    }


    static void *ros_worker(void *data)
    {
        printf("+ros_worker\n");

        ros::Subscriber sub_sonar = node->subscribe("naze_sonar", 100, sonar_update);
        ros::Subscriber sub_joystick = node->subscribe("joy", 100, joystick_update);
        ros::Subscriber sub_imu = node->subscribe("naze_imu", 100, imu_update);
        ros::Subscriber sub_pos = node->subscribe("naze_pos", 100, pos_update);
        ros::Subscriber sub_simtime = node->subscribe("naze_time", 100, simtime_update);

        printf("ros_worker: before ros::spin()\n");
        ros::spin();
        printf("-ros_worker\n");
        return NULL;
    }


    void start_ros_worker()
    {
        printf("+start_ros_worker\n");

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "nazelib", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            printf("after ros::init\n");
        }

        node = boost::make_shared<ros::NodeHandle>();
        motor_pub = node->advertise<nazecom::MotorControl>("motor_data", 1000);

        pthread_create(&ros_thread, NULL, ros_worker, NULL);
        printf("-start_ros_worker\n");
    }

#ifdef __cplusplus
}
#endif