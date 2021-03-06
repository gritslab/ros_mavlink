/**
 * @file ros_mavlink.cpp
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "ros_mavlink.h"
#include "mmath.h"
#include "quad_finite_state.h"

// Serial Port
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <glib.h> /* Thread definitions */

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------


//==============================================================================
// RosMavlink Class
//==============================================================================
//------------------------------------------------------------------------------
// Constructors and Destructors
//------------------------------------------------------------------------------
RosMavlink::RosMavlink(string port, int baud, int sysid, int compid)
:
m_port(port),
m_baud(baud),
m_sysid(sysid),
m_compid(compid),
m_fd(0)
{
	armed = false;
	m_setup_publishers();
    m_setup_subscribers();
    m_start();
}

RosMavlink::~RosMavlink()
{
    if (m_fd > 0) {
        m_close_port();
    }
    m_UART_mutex.~mutex();
}

//--------------------------------------------------------------------------
// Public Member Getters and Setters
//--------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Public Methods
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Private Methods
//------------------------------------------------------------------------------

// ROS
void RosMavlink::m_setup_publishers()
{
    m_pub_mavlink_attitude =
    m_node_handle.advertise<geometry_msgs::Vector3>("mavlink_attitude", 1);
    //publish cmds service, all commands to the quad use this channel
    m_sub_cmds = m_node_handle.advertiseService("/quad/cmds", &RosMavlink::m_handle_cmds, this);
    system_state = 0;
}

void RosMavlink::m_setup_subscribers()
{
    m_sub_quad_pose_act = m_node_handle.subscribe("/quad/pose", 1,
                                              &RosMavlink::m_handle_quad_pose_act,
                                              this);
    //subscribe to topic over which the waypoints are sent
    m_sub_quad_pose_des = m_node_handle.subscribe("/quad/pose_des", 1, &RosMavlink::m_handle_quad_pose_des, this);

}

bool RosMavlink::m_handle_cmds(ros_mavlink::CommandSrv::Request &req, ros_mavlink::CommandSrv::Response &res){
    mavlink_message_t msg;
	mavlink_command_int_t m_cmd;
	mavlink_command_long_t m_cmd_l;
	//switch over all commands in Commands.h
	switch (req.command) {
		case TOGGLE_ARM:
			ROS_INFO("Received arming command");
            armed = !armed;
			m_cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
            m_cmd.target_system = m_sysid;
            m_cmd.target_component = m_compid;
            if(system_state==MAV_STATE_STANDBY){
            	m_cmd.param1 = 1;
            }
            else{
            	m_cmd.param1 = 0;
            }
			mavlink_msg_command_int_encode(m_sysid, m_compid, &msg, &m_cmd);
			break;
		case FLY_TO:
			ROS_INFO("Received GoTo");

			break;
		case TAKEOFF:
			ROS_INFO("Received take-off command");
			m_cmd.command = MAV_CMD_NAV_TAKEOFF;
			mavlink_msg_command_int_encode(m_sysid, m_compid, &msg, &m_cmd);
			break;
		case LAND:
			ROS_INFO("Received landing command");
			m_cmd.command = MAV_CMD_NAV_LAND;
			mavlink_msg_command_int_encode(m_sysid, m_compid, &msg, &m_cmd);
			break;
		case CALIBRATE:
			ROS_INFO("Preflight Calibration");
			m_cmd_l.command = MAV_CMD_PREFLIGHT_CALIBRATION;
            m_cmd_l.target_system = m_sysid;
            m_cmd_l.target_component = m_compid;
			m_cmd_l.param1 = 0;
			m_cmd_l.param2 = 0;
			m_cmd_l.param3 = 0;
			m_cmd_l.param4 = 0;
			m_cmd_l.param5 = 1;
			m_cmd_l.param6 = 0;
			m_cmd_l.param7 = 0;
			mavlink_msg_command_long_encode(m_sysid, m_compid, &msg, &m_cmd_l);

			break;
		case RETURN:
			ROS_INFO("Return to launch");
			m_cmd_l.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
			m_cmd_l.target_system = m_sysid;
			m_cmd_l.target_component = m_compid;
			m_cmd_l.param1 = 0;
			m_cmd_l.param2 = 0;
			m_cmd_l.param3 = 0;
			m_cmd_l.param4 = 0;
			m_cmd_l.param5 = 1;
			m_cmd_l.param6 = 0;
			m_cmd_l.param7 = 0;
			mavlink_msg_command_long_encode(m_sysid, m_compid, &msg, &m_cmd_l);
			break;
		default:
			ROS_ERROR("Command not recognized!");
			res.success = FALSE;
			return 0;
	}

	m_write_UART(&msg);
	res.success = TRUE;
	return 1;
}

void RosMavlink::m_write_UART(mavlink_message_t *msg){

    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)m_buf, msg);
    //lock UART write

    ROS_INFO("Sending message");
    ROS_DEBUG("Trying to write message");
    // m_UART_mutex.lock();
    write(m_fd, m_buf, len);
    tcdrain(m_fd);
    // m_UART_mutex.unlock();
}

void RosMavlink::m_handle_quad_pose_des(const geometry_msgs::Pose &ros_pose){
    double r = ros_pose.orientation.w;
    double i = ros_pose.orientation.x;
    double j = ros_pose.orientation.y;
    double k = ros_pose.orientation.z;
    double phi, theta, psi;
    quat2euler(r, i, j, k, phi, theta, psi);

    //mavlink_set_position_target_local_ned_t



}


void RosMavlink::m_handle_quad_pose_act(const geometry_msgs::Pose &ros_pose)
{
    double r = ros_pose.orientation.w;
    double i = ros_pose.orientation.x;
    double j = ros_pose.orientation.y;
    double k = ros_pose.orientation.z;
    double phi, theta, psi;
    quat2euler(r, i, j, k, phi, theta, psi);

    mavlink_vicon_position_estimate_t mav_pose;

    mav_pose.x = ros_pose.position.x;
    mav_pose.y = ros_pose.position.y;
    mav_pose.z = -ros_pose.position.z;
    mav_pose.roll = phi;
    mav_pose.pitch = theta;
    mav_pose.yaw = psi;

    mavlink_message_t msg;
    mavlink_msg_vicon_position_estimate_encode(m_sysid, m_compid, &msg,
                                               &mav_pose);
    m_write_UART(&msg);
}


void RosMavlink::m_decode_mavlink(mavlink_message_t &message)
{
	//check if message contains a pose, if yes send it over ROS, otherwise store message for internal use
    if(message.msgid == MAVLINK_MSG_ID_ATTITUDE){
    		m_publish_ros_pose(message);
    }
    else{
    	if(message.msgid == MAVLINK_MSG_ID_COMMAND_ACK){
			__mavlink_command_ack_t* msg;
			mavlink_msg_command_ack_decode(&message, msg);
			ROS_INFO("Received message Command: %d, Result %d", msg->command, msg->result);
			message_queue.push_back(message);
    	}
    	if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT){
    		__mavlink_heartbeat_t* msg;
    		mavlink_msg_heartbeat_decode(&message, msg);
    		ROS_INFO("System state %d",msg->system_status);
    		system_state = msg->system_status;
    	}
    	if(message.msgid ==  MAVLINK_MSG_ID_BATTERY_STATUS){
    		__mavlink_battery_status_t* msg;
    		mavlink_msg_battery_status_decode(&message, msg);
    		ROS_INFO("Battery Status: Percent left: %d", msg->battery_remaining);

    	}
    }

}
void RosMavlink::m_publish_ros_pose(mavlink_message_t &message){
	mavlink_attitude_t att;
	mavlink_msg_attitude_decode(&message, &att);
	geometry_msgs::Vector3 msg;
	msg.x = att.roll;
	msg.y = att.pitch;
	msg.z = att.yaw;
	m_pub_mavlink_attitude.publish(msg);
}


// Serial
int RosMavlink::m_open_port()
{
    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_fd == -1) {
        return -1;
    } else {
        fcntl(m_fd, F_SETFL, 0);
        return 0;
    }
}

void RosMavlink::m_close_port()
{
    close(m_fd);
    m_fd = 0;
}

bool RosMavlink::m_setup_port()
{
    int data_bits = 8;
    int stop_bits = 1;
    bool parity = false;
    bool hardware_control = false;

    struct termios config;
    if (!isatty(m_fd))
    {
        ROS_WARN("\nERROR: file descriptor %d is NOT a serial port\n", m_fd);
        return false;
    }
    if (tcgetattr(m_fd, &config) < 0)
    {
        ROS_WARN("\nERROR: could not read configuration of fd %d\n", m_fd);
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR \
                        | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    #ifdef OLCUC
        config.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
        config.c_oflag &= ~ONOEOT;
    #endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    switch (m_baud)
    {
        case 1200:
            if (cfsetispeed(&config, B1200) < 0 ||
                    cfsetospeed(&config, B1200) < 0)
            {
                ROS_WARN("\nERROR: Could not set desired \
                          baud rate of %d Baud\n",
                          m_baud);
                return false;
            }
            break;
        case 1800:
            cfsetispeed(&config, B1800);
            cfsetospeed(&config, B1800);
            break;
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            if (cfsetispeed(&config, B38400) < 0 ||
                    cfsetospeed(&config, B38400) < 0)
            {
                ROS_WARN("\nERROR: Could not set desired \
                          baud rate of %d Baud\n",
                          m_baud);
                return false;
            }
            break;
        case 57600:
            if (cfsetispeed(&config, B57600) < 0 ||
                    cfsetospeed(&config, B57600) < 0)
            {
                ROS_WARN("\nERROR: Could not set desired \
                          baud rate of %d Baud\n",
                          m_baud);
                return false;
            }
            break;
        case 115200:
            if (cfsetispeed(&config, B115200) < 0 ||
                    cfsetospeed(&config, B115200) < 0)
            {
                ROS_WARN("\nERROR: Could not set desired \
                          baud rate of %d Baud\n",
                          m_baud);
                return false;
            }
            break;

        // These two non-standard (by the 70'ties )
        // rates are fully supported on
        // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if (cfsetispeed(&config, 460800) < 0 ||
                    cfsetospeed(&config, 460800) < 0)
            {
                ROS_WARN("\nERROR: Could not set desired\
                          baud rate of %d Baud\n",
                          m_baud);
                return false;
            }
            break;
        case 921600:
            if (cfsetispeed(&config, 921600) < 0 ||
                    cfsetospeed(&config, 921600) < 0)
            {
                ROS_WARN("\nERROR: Could not set desired \
                          baud rate of %d Baud\n",
                        m_baud);
                return false;
            }
            break;
        default:
            ROS_WARN("ERROR: Desired baud rate %d could\
                      not be set, aborting.\n",
                      m_baud);
            return false;

            break;
    }
    //
    // Finally, apply the configuration
    //
    if (tcsetattr(m_fd, TCSAFLUSH, &config) < 0)
    {
        ROS_WARN("\nERROR: could not set configuration\
                  of fd %d\n", m_fd);
        return false;
    }

    return true;
}

void* RosMavlink::serial_wait(void* rm_ptr)
{
    RosMavlink* rm = ((RosMavlink*)(rm_ptr));
    rm->m_main_thread();

    return NULL;
}

void RosMavlink::m_start()
{
    // Connect
    ROS_DEBUG("Trying to connect to %s.. ", m_port.c_str());
    m_open_port();
    if (m_fd == -1) {
        ROS_ERROR("failure, could not open port %s\n", m_port.c_str());
        exit(EXIT_FAILURE);
    } else {
        ROS_DEBUG("success.\n");
    }

    // Configure
    ROS_DEBUG("Trying to configure %s.. ", m_port.c_str());

    if (!m_setup_port()) {
        ROS_ERROR("failure, could not configure port.\n");
        exit(EXIT_FAILURE);
    } else {
        ROS_DEBUG("success.\n");
    }

    ROS_DEBUG("PORT::%d",m_fd);
    int* fd_ptr = &m_fd;

    GThread* serial_thread;
    GError* err;

    // Run indefinitely while the ROS and serial threads handle the data
    ROS_INFO("\nREADY, waiting for serial/ROS data.\n");

    if ((serial_thread = g_thread_new("Ros-Mavlink-Serial",
                                       (GThreadFunc)(&RosMavlink::serial_wait),
                                       (void *)(this))) == NULL) {
        ROS_ERROR("Failed to create serial handling thread: %s!!\n",
                  err->message);
        g_error_free(err);
    }

    int noErrors = 0;
    if (m_fd == -1 || m_fd == 0) {
        ROS_ERROR("Connection attempt to port %s with %d \
                  baud, 8N1 failed, exiting.\n",
                  m_port.c_str(),
                  m_baud);
        exit(EXIT_FAILURE);
    } else {
        ROS_WARN("\nConnected to %s with %d baud, \
                 8 data bits, no parity, 1 stop bit (8N1)\n",
                 m_port.c_str(),
                 m_baud);
    }

    if (m_fd == -1 || m_fd == 0) {
        exit(noErrors);
    }

    ROS_INFO("\nMAVLINK SERIAL TO ROS BRIDGE STARTED ON MAV %d \
              (COMPONENT ID:%d) - RUNNING..\n\n",
              m_sysid,
              m_compid);
}

void RosMavlink::m_main_thread()
{
    mavlink_status_t lastStatus;
    lastStatus.packet_rx_drop_count = 0;
    // Blocking wait for new data

    ros::Rate r(100);
    while (ros::ok()) {
        uint8_t cp;
        mavlink_message_t message;
        mavlink_status_t status;
        uint8_t msgReceived = false;

        if (read(m_fd, &cp, 1) > 0) {
            // Check if a message could be decoded,
            // return the message in case yes
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1,
                                             cp,
                                             &message,
                                             &status);
            if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) {
                ROS_WARN("ERROR: DROPPED %d PACKETS\n",
                         status.packet_rx_drop_count);
                unsigned char v = cp;
                ROS_WARN("%02x ", v);
            }
                lastStatus = status;
        } else {
            ROS_ERROR("ERROR: Could not read from port %s\n",
                      m_port.c_str());
            exit(EXIT_FAILURE);
        }

        // If a message could be decoded, handle it
        if (msgReceived) {
            ROS_DEBUG("Forwarding SERIAL -> ROS: ");
            unsigned int i;
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            unsigned int messageLength =
            mavlink_msg_to_send_buffer(buffer, &message);
            if (messageLength > MAVLINK_MAX_PACKET_LEN) {
                ROS_WARN("\nFATAL ERROR: MESSAGE LENGTH \
                         IS LARGER THAN BUFFER SIZE\n");
            } else {

                for (i = 0; i < messageLength; i++) {
                    unsigned char v = buffer[i];
                    ROS_DEBUG("%02x ", v);
                }
            }

            /*ROS_INFO("Received message from serial \
                     with ID #%d (sys:%d|comp:%d):\n",

                     message.msgid,
                     message.sysid,
                     message.compid);*/

            m_decode_mavlink(message);
        }
        //let the loop sleep, so it runs at a rate of 10Hz
        //r.sleep();
    }
}
