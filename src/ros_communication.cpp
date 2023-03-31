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

    // // stop
    // if (CommandVelocity.linear.x || CommandVelocity.angular.z || CommandVelocity.angular.y)
    // {
    //     i
    //         Flag_start = 1;
    // }
    // else
    // {
    //     Flag_start = 0;
    //     WheelSpeed = WheelAngle = motor_Fork = 0.0;
    // }
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