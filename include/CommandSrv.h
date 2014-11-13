/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /home/urs/catkin_ws/src/ros_mavlink/srv/CommandSrv.srv
 *
 */


#ifndef ROS_MAVLINK_MESSAGE_COMMANDSRV_H
#define ROS_MAVLINK_MESSAGE_COMMANDSRV_H

#include <ros/service_traits.h>


#include <ros_mavlink/CommandSrvRequest.h>
#include <ros_mavlink/CommandSrvResponse.h>


namespace ros_mavlink
{

struct CommandSrv
{

typedef CommandSrvRequest Request;
typedef CommandSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CommandSrv
} // namespace ros_mavlink


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ros_mavlink::CommandSrv > {
  static const char* value()
  {
    return "5c8a2b607525c6afcd3f3f70d56f3c09";
  }

  static const char* value(const ::ros_mavlink::CommandSrv&) { return value(); }
};

template<>
struct DataType< ::ros_mavlink::CommandSrv > {
  static const char* value()
  {
    return "ros_mavlink/CommandSrv";
  }

  static const char* value(const ::ros_mavlink::CommandSrv&) { return value(); }
};


// service_traits::MD5Sum< ::ros_mavlink::CommandSrvRequest> should match 
// service_traits::MD5Sum< ::ros_mavlink::CommandSrv > 
template<>
struct MD5Sum< ::ros_mavlink::CommandSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ros_mavlink::CommandSrv >::value();
  }
  static const char* value(const ::ros_mavlink::CommandSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_mavlink::CommandSrvRequest> should match 
// service_traits::DataType< ::ros_mavlink::CommandSrv > 
template<>
struct DataType< ::ros_mavlink::CommandSrvRequest>
{
  static const char* value()
  {
    return DataType< ::ros_mavlink::CommandSrv >::value();
  }
  static const char* value(const ::ros_mavlink::CommandSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ros_mavlink::CommandSrvResponse> should match 
// service_traits::MD5Sum< ::ros_mavlink::CommandSrv > 
template<>
struct MD5Sum< ::ros_mavlink::CommandSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ros_mavlink::CommandSrv >::value();
  }
  static const char* value(const ::ros_mavlink::CommandSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_mavlink::CommandSrvResponse> should match 
// service_traits::DataType< ::ros_mavlink::CommandSrv > 
template<>
struct DataType< ::ros_mavlink::CommandSrvResponse>
{
  static const char* value()
  {
    return DataType< ::ros_mavlink::CommandSrv >::value();
  }
  static const char* value(const ::ros_mavlink::CommandSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROS_MAVLINK_MESSAGE_COMMANDSRV_H
