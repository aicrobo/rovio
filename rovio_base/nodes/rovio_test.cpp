/*!
 * Author: Yanbo She yuanboshe@126.com
 * Maintainer: guqiyang guqiyang@aicrobo.com
 * Group: AICRobo http://www.aicrobo.com
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013~2016, AICRobo.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include"rovio_base/report.h"

using namespace std;

ros::Publisher crlpub;

void showimg(sensor_msgs::Image img)
{
  ROS_INFO("Image size: %dx%d",img.height,img.width);
  cv_bridge::CvImagePtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);
  cv::imshow("", cvImgPtr->image);
  cv::waitKey(100);
}

void report(rovio_base::report report)
{
  int length = report.length;
  int lDirection = report.lDirection;
  int lNum = report.lNum;
  int rDirection = report.rDirection;
  int rNum = report.rNum;
  int rearDirection = report.rearDirection;
  int rearNum = report.rearNum;
  int headPosition = report.headPosition;
  int isLedOn = report.isLedOn;
  ROS_INFO("MCU Report:\nlength=%d", length);
  ROS_INFO("Left direction:num=%d:%d", lDirection, lNum);
  ROS_INFO("Right direction:num=%d:%d", rDirection, rNum);
  ROS_INFO("Rear direction:num=%d:%d", rearDirection, rearNum);
  ROS_INFO("headPosition=%d", headPosition);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rovioTest");
  ros::NodeHandle nh;

  crlpub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber imgSub=nh.subscribe<sensor_msgs::Image>("image_raw", 10, showimg);
  ros::Subscriber reportSub=nh.subscribe<rovio_base::report>("rovio_report", 10, report);

  ros::Rate loopRate(10);
  ROS_INFO("Rovio test ON...");
  while (ros::ok())
  {
    geometry_msgs::Twist twist;
    twist.linear.x = -6;
    twist.linear.y = -6;
    twist.angular.z = 0;
    crlpub.publish(twist);

    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}
