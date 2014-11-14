/*
@brief Class for ROS and Mavlink communication

@author Rowland O'Flaherty
@date 11/06/2014
*/

#ifndef ROS_MAVLINK_H
#define ROS_MAVLINK_H

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <string.h>
#include <boost/thread/mutex.hpp>

#include "include/mavlink/pixhawk/mavlink.h"

// ROS messages
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>

#include "include/CommandSrv.h"
//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;

//==============================================================================
// RosMavlink Class
//==============================================================================
class RosMavlink
{
public:
    //--------------------------------------------------------------------------
    // Constructors and Destructors
    //--------------------------------------------------------------------------
    RosMavlink(string port="/dev/ttyUSB0", int baud=115200,
               int sysid=42, int compid=110);

    ~RosMavlink();

    //--------------------------------------------------------------------------
    // Public Member Getters and Setters
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // Public Methods
    //--------------------------------------------------------------------------

private:
    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------
    // ROS
    ros::NodeHandle m_node_handle;

    ros::Publisher m_pub_mavlink_attitude;

    ros::Subscriber m_sub_quad_pose_act;
    ros::Subscriber m_sub_quad_pose_des;
    ros::ServiceServer m_sub_cmds;

    // Serial
    string m_port;
    int m_baud;
    int m_sysid;
    int m_compid;
    int m_fd;
    char m_buf[300];

    bool armed ;

    boost::mutex m_UART_mutex;
    vector<mavlink_message_t> message_queue;

    //--------------------------------------------------------------------------
    // Private Methods
    //--------------------------------------------------------------------------
    // Ros
    void m_setup_publishers();
    void m_setup_subscribers();


    void m_write_UART(mavlink_message_t *msg);

    void m_handle_quad_pose_des(const geometry_msgs::Pose &ros_pose);
    void m_handle_quad_pose_act(const geometry_msgs::Pose &ros_pose);
    bool m_handle_cmds(ros_mavlink::CommandSrv::Request &req, ros_mavlink::CommandSrv::Response &res);

    void m_decode_mavlink(mavlink_message_t &message);
    void m_publish_ros_pose(mavlink_message_t &message);

    // Serial
    int m_open_port();
    void m_close_port();
    bool m_setup_port();
    static void* serial_wait(void* rm_ptr);
    void m_start();
    void m_main_thread();

};

#endif
