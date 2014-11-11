#include <ros/ros.h>
#include "ros_mavlink.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_mavlink");

    std::string port;
    const std::string PATH_DEFAULT = "/dev/ttyO1";
    const int BAUD_DEFAULT =115200;
    const int COMPID_DEFAULT = 110;
    const int SYSID_DEFAULT = 42;
    int baud;
    int sysid;
    int compid;

    ros::NodeHandle ros_nh;

    // read all necessary parameters provided in launch file, or use standard values, if not provided or started via rosrun
    ros_nh.param("ros_mavlink/uart_port", port, PATH_DEFAULT);
    ros_nh.param("ros_mavlink/uart_baudrate", baud, BAUD_DEFAULT);
    ros_nh.param("ros_mavlink/uart_compid", compid, COMPID_DEFAULT);
    ros_nh.param("ros_mavlink/uart_sysid", sysid, SYSID_DEFAULT);

    RosMavlink rm(port, baud, sysid, compid);



    ros::Rate loop_rate(10);

    while(ros::ok()){
        // rm.send_setpoint();

        ros::spinOnce();
        loop_rate.sleep();
    }

    // rm.close_port();

    return 0;
}
