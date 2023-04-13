#pragma once

#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <map>
#include <string>

#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include "fork_msg/forkmotion.h"
#include "fork_msg/forkposition.h"

class ROSCommunication
{
private:
    float WheelSpeed, WheelAngle, WheelBase, ForkMotor;
    
    fork_msg::forkmotion ForkMotion;
    nav_msgs::Odometry Odom;
    sensor_msgs::Imu IMUData;
    geometry_msgs::Twist CommandVelocity;
    geometry_msgs::TransformStamped OdomTFStamp, ForkTFStamp;
    geometry_msgs::Quaternion OdomQuaternion, IMUQuaternion;

public:
    void GetLaunchParameter();
    void Initial();
    void CommandVelocityCallBack(const geometry_msgs::Twist &msg);
    void ForkMotionCallBack(const fork_msg::forkmotion &msg);
    void PublishOdomTopic();
    void PublishOdomTF();
    void PublishIMUTopic();
    void PublishIMUTF();
    void PublishForkTopic();
    void PublishForkTF();
    int StateDectect();
    int SignNumber(float Number);
};
