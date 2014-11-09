#include <ros/ros.h>
#include "ros_mavlink.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_mavlink");

    string port = "/dev/ttyUSB0";
    int baud = 115200;
    int sysid = 1;
    int compid = 50;
    RosMavlink rm(port, baud, sysid, compid);

    ros::NodeHandle ros_nh;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        // rm.send_setpoint();

        ros::spinOnce();
        loop_rate.sleep();
    }

    // rm.close_port();

    return 0;
}
