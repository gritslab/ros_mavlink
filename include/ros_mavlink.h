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

#include "mavlink.h"


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

    // Serial
    string m_port;
    int m_baud;
    int m_sysid;
    int m_compid;
    int m_fd;


    //--------------------------------------------------------------------------
    // Private Methods
    //--------------------------------------------------------------------------
    // Ros
    void m_setup_publishers();
    void m_decode_mavlink_publish_ros(mavlink_message_t &message);

    // Serial
    int m_open_port();
    void m_close_port();
    bool m_setup_port();
    static void* serial_wait(void* rm_ptr);
    void m_start();
    void m_main_thread();

};

#endif
