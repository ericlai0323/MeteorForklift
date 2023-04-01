#include <ros_communication.hpp>

void ROSCommunication::CommandVelocityCallBack(const geometry_msgs::Twist &msg)
{
    CommandVelocity = msg;

    if (CommandVelocity.linear.x == 0 && CommandVelocity.angular.z != 0)
    {
        WheelAngle = 90;
        WheelSpeed = (fabs(CommandVelocity.angular.z) > 1.0 ? SignNumber(CommandVelocity.angular.z) * 1.0 : CommandVelocity.angular.z) * WheelBase;
    }
    else if (CommandVelocity.linear.x != 0)
    {
        WheelAngle = tanh(WheelBase * CommandVelocity.angular.z / CommandVelocity.linear.x) * 180 / M_PI;
        WheelSpeed = CommandVelocity.linear.x / cos(WheelAngle / 180 * M_PI);
    }
    else
    {
        WheelAngle = WheelSpeed = 0;
    }

    WheelAngle = fabs(WheelAngle) > 90 ? SignNumber(WheelAngle) * 90 : WheelAngle;
    WheelSpeed = fabs(WheelSpeed) > 0.8 ? SignNumber(WheelSpeed) * 0.8 : WheelSpeed;

    if (CommandVelocity.linear.x == 0 && CommandVelocity.angular.z == 0)
    {
        WheelAngle = 0;
        WheelSpeed = 0;
    }
}

void ROSCommunication::ForkMotionCallBack(const fork_msg::forkmotion &msg)
{
    int ForkControl = msg.forkmotion;
    switch (ForkControl)
    {
    case 1:
        ForkMotor = 0;
        break;
    case 2:
        ForkMotor = -2000;
        break;
    case 3:
        ForkMotor = 1700;
        break;
    default:
        ForkMotor = 0;
        break;
    }
}
int ROSCommunication::SignNumber(float Number)
{
    if (Number > 0)
    {
        return 1;
    }
    else if (Number < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

// void ROSCommunication::PublishOdomTopic()
// {
//     Odom.header.stamp = ros::Time::now();
//     Odom.header.frame_id = "wheel_odom";
//     Odom.pose.pose.position.x 
// }