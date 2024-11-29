#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pid_tunning/motor_controlConfig.h>

void call_back(pid_tunning::motor_controlConfig &config, uint32_t level) {
    ROS_INFO("motor speed = %.2f, Kp = %.2f, Ki = %.2f, Kd = %.2f",
              config.motor_speed, config.P, config.I, config.D);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_reconfigure");
    dynamic_reconfigure::Server<pid_tunning::motor_controlConfig> server;
    dynamic_reconfigure::Server<pid_tunning::motor_controlConfig>::CallbackType f;
    f = boost::bind(&call_back, _1, _2);
    server.setCallback(f);
    ros::spin();
    return 0;
}
