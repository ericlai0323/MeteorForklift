#pragma once

#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <map>
#include <string>

#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

class ROSCommunication
{
private:
    float WheelSpeed, WheelAngle, WheelBase, ForkMotor;
    float AngularVelocityX, AngularVelocityY, AngularVelocityZ;
    geometry_msgs::Twist CommandVelocity;
    std_msgs::Float32 ForkMotion;

    sensor_msgs::Imu IMUData;
    geometry_msgs::TransformStamped OdomTFStamp, ForkTFStamp;
    geometry_msgs::Quaternion OdomQuaternion, IMUQuaternion;

public:
    void GetLaunchParameter();
    void CommandVelocityCallBack(const geometry_msgs::Twist &msg);
    void ForkMotionCallBack(const std_msgs::Float32 &msg);
    void PublishOdomTopic();
    void PublishOdomTF();
    void PublishIMUTopic();
    void PublishIMUTF();
    int StateDectect();
    int SignNumber(float Number);
};
