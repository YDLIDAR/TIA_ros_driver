//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "src/CYdLidar.h"


#define SDKROSVerision "1.0.2"
CYdLidar lidar;

bool stop_scan(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_DEBUG("Stop scan");
  return lidar.turnOff();
}

bool start_scan(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_DEBUG("Start scan");
  return lidar.turnOn();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "ydlidar_ros_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::ServiceServer stop_scan_service = nh.advertiseService("stop_scan", stop_scan);
  ros::ServiceServer start_scan_service = nh.advertiseService("start_scan", start_scan);
  ROS_INFO("YDLIDAR ROS Driver Version: %s", SDKROSVerision);	

  
  //////////////////////string property/////////////////  
  std::string frame_id;
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

  std::string str_optvalue;
  nh_private.param<std::string>("port", str_optvalue, "192.168.0.100");
  lidar.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  //////////////////////int property/////////////////
  int optval;

  nh_private.param<int>("baudrate", optval, 8090);
  lidar.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));

  nh_private.param<int>("lidar_type", optval, TYPE_TIA);
  lidar.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  bool b_optvalue;

  nh_private.param<bool>("auto_reconnect", b_optvalue, true);
  lidar.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue;

  nh_private.param<float>("angle_max", f_optvalue, 330.f);
  lidar.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));

  nh_private.param<float>("angle_min", f_optvalue, 30.f);
  lidar.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  nh_private.param<float>("range_max", f_optvalue, 64.f);
  lidar.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));

  nh_private.param<float>("range_min", f_optvalue, 0.01);
  lidar.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

  /// unit: Hz
  nh_private.param<float>("frequency", f_optvalue, 30.f);
  lidar.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
  
  // initialize SDK and LiDAR
  bool ret = lidar.initialize();
  if (ret) {
    ret = lidar.turnOn();
  } else {
    ROS_ERROR("%s\n", lidar.DescribeError());
  }
  ros::Rate r(30);	
  while (ros::ok() && ret) {
    LaserScan scan;
    if(lidar.doProcessSimple(scan))
    {
      sensor_msgs::LaserScan scan_msg;
      ros::Time start_scan_time = ros::Time::now();

      scan_msg.header.stamp = start_scan_time;
      scan_msg.header.frame_id = frame_id;
      scan_msg.angle_min = scan.config.min_angle;
      scan_msg.angle_max = scan.config.max_angle;
      scan_msg.angle_increment = scan.config.angle_increment;//点与点之间的角度间隔
      scan_msg.scan_time = scan.config.scan_time;//扫描一圈所用的时间(秒)
      scan_msg.time_increment = scan.config.time_increment;//采样一个点所用的时间(秒)
      scan_msg.range_min = scan.config.min_range;
      scan_msg.range_max = scan.config.max_range;

      scan_msg.ranges.resize(scan.points.size());
      scan_msg.intensities.resize(scan.points.size());
      for(size_t i = 0; i < scan.points.size(); i++) {
        scan_msg.ranges[i] = scan.points[i].range;
        scan_msg.intensities[i] = scan.points[i].intensity;
      }
      scan_pub.publish(scan_msg);
    }
    else {
      ROS_ERROR("Failed to get Lidar Data");
    }
    r.sleep();
    ros::spinOnce();
  }
  lidar.turnOff();
  ROS_INFO("[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  lidar.disconnecting();
  return 0;
}

