#include "ros/ros.h"
#include "control/HardwareCommand.h"
#include "control/HardwareState.h"
#include "control/TogglePWM.h"
#include <signal.h>
#include <vector>

// Global variables
control::HardwareState sensor_data;
bool robot_status = false;
control::HardwareCommand pwm;

// Signal handler
void signal_callback_handler(int signum) {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<control::HardwareCommand>("/control/command/hardware", 1);
    
    ROS_INFO("SHUTTING DOWN");
    
    pwm.motor1 = 0;
    pwm.motor2 = 0;
    pwm.motor3 = 0;
    pub.publish(pwm);
    
    ROS_INFO("motor1: %f", pwm.motor1);
    ROS_INFO("motor2: %f", pwm.motor2);
    ROS_INFO("motor3: %f", pwm.motor3);

    exit(signum);
}

// Rosservice for toggle on/off
bool toggle_pwm(control::TogglePWM::Request &req, control::TogglePWM::Response &res) {
    res.active = req.toggle;
    robot_status = res.active;
    return true;
}

// Callback function for subscriber 
void update_cb(const control::HardwareState& msg) {
    sensor_data = msg;
    // ROS_INFO("sensor1: %f", sensor_data.sensor1);
    // ROS_INFO("sensor2: %f", sensor_data.sensor2);
    // ROS_INFO("sensor3: %f", sensor_data.sensor3);
}

int main(int argc, char **argv) {
    
    // Initialize node
    ros::init(argc, argv, "pwmnode");
    ros::NodeHandle nh;

    // Publisher and Subscriber
    ros::Publisher pub = nh.advertise<control::HardwareCommand>("/control/command/hardware", 1);
    ros::Subscriber sub = nh.subscribe("arduino/state/hardware", 1, update_cb);
    
    // Service for robot toggle
    ros::ServiceServer service = nh.advertiseService("toggle_pwm", toggle_pwm);
    
    // Loop rate
    ros::Rate loop_rate(20);
    
    // Array for storing parameters
    std::vector<float> condition_normal;
    std::vector<float> condition_1;
    std::vector<float> condition_2;
    std::vector<float> condition_3;
    
    // Initial conditions
    condition_normal={-0.15,0.17,0};
    condition_1={-0.15,-0.17,-0.15}; 
    condition_2={0.15,0.17,0.15};
    // condition_3={0.2,0.2,0.2};
    float stop;
    float p=20;

    // Signal handler
    signal(SIGINT, signal_callback_handler);

    while(ros::ok()) {
        
        // Checking initial conditions
        float motor_1=condition_1[0];
        float motor_2=condition_1[1];
        float motor_3=condition_1[2];
        // ROS_INFO("condition_1_1: %f",motor_1);
        // ROS_INFO("condition_1_2: %f",motor_2);
        // ROS_INFO("condition_1_3: %f",motor_3);
        ROS_INFO("sensor_front: %f", sensor_data.sensor1); // front
        ROS_INFO("sensor_right: %f", sensor_data.sensor2); // right
        ROS_INFO("sensor_left: %f", sensor_data.sensor3); // left

        // Get paramaters from the initial conditions
        nh.getParam("/condition_normal", condition_normal);
        nh.getParam("/condition_1", condition_1);
        nh.getParam("/condition_2", condition_2);
        // nh.getParam("/condition_3", condition_3);
        nh.param("/stop",stop,p);

        // Toggle ON
        if (robot_status) {
            if ((sensor_data.sensor2 <= stop) && (sensor_data.sensor3 <= stop) && (sensor_data.sensor1 > stop)) {
                ROS_INFO("Moving Forward");
                pwm.motor1 = condition_normal[0];
                pwm.motor2 = condition_normal[1];
                pwm.motor3 = condition_normal[2];
            }
           // if there is an object in the front sensor or left sensor, turn right
            else if ((sensor_data.sensor1 <= stop) || (sensor_data.sensor3 <= stop)) {
                ROS_INFO("Turning Right");
                pwm.motor1 = condition_1[0];
                pwm.motor2 = condition_1[1];
                pwm.motor3 = condition_1[2];
            }
            
            // if there is an object in the right sensor, turn left
            else if (sensor_data.sensor2 <= stop) {
                ROS_INFO("Turning Left");
                pwm.motor1 = condition_2[0];
                pwm.motor2 = condition_2[1];
                pwm.motor3 = condition_2[2];
            }
             // if the sensor is clear, move forward
            else  {
                ROS_INFO("Moving Forward");
                pwm.motor1 = condition_normal[0];
                pwm.motor2 = condition_normal[1];
                pwm.motor3 = condition_normal[2];
            }
        }

        // Toggle OFF
        else {
            pwm.motor1 = 0;
            pwm.motor2 = 0;
            pwm.motor3 = 0;
        }

        // Checking PWM values
        ROS_INFO("motor1: %f", pwm.motor1);
        ROS_INFO("motor2: %f", pwm.motor2);
        ROS_INFO("motor3: %f", pwm.motor3);

        // Publish PWM value to rostopic
        pub.publish(pwm);
        ros::spinOnce();
        loop_rate.sleep();
    }

   return 0;
}