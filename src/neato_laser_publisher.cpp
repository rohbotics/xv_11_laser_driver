/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <xv_11_laser_driver/xv11_laser.h>
#include <std_msgs/UInt16.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "neato_laser_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  std::string port;
  int baud_rate;
  std::string frame_id;
  int firmware_number;
 
  std_msgs::UInt16 rpms; 

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("baud_rate", baud_rate, 115200);
  priv_nh.param("frame_id", frame_id, std::string("laser"));
  priv_nh.param("firmware_version", firmware_number, 1);

  boost::asio::io_service io;

  try {
    xv_11_laser_driver::XV11Laser laser(port, baud_rate, firmware_number, io);
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms",1000);

    while (ros::ok()) {
      sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
      scan->header.frame_id = frame_id;
      scan->header.stamp = ros::Time::now();
      laser.poll(scan);
      rpms.data=laser.rpms;
      laser_pub.publish(scan);
      motor_pub.publish(rpms);

    }
    laser.close();
    return 0;
  } catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
