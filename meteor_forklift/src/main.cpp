#include <ros_communication.h>
#include <stm32_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MeteorForklift");
    ros::NodeHandle n;
    ros::NodeHandle nh_priv("~");

    ROSCommunication ROSCommunicationObject;
    STM32Controller STM32ControllerObject;
    // Subscriber
    ros::Subscriber SubCommandVelocity = n.subscribe("cmd_vel", 200, &ROSCommunication::CommandVelocityCallBack, &ROSCommunicationObject); // Subcribe teleop_twist_keyboard
    ros::Subscriber SubForkMotion = n.subscribe("ForkUpDown",100, &ROSCommunication::ForkMotionCallBack, &ROSCommunicationObject);

    // Publisher
    ros::Publisher PubOdom = n.advertise<nav_msgs::Odometry>("wheel_odom", 50);
    ros::Publisher PubIMU = n.advertise<sensor_msgs::Imu>("imu_data", 20);
    ros::Publisher PubForkPosition = n.advertise<std_msgs::Float32>("fork_position", 20);

    // Transform TF
    tf::TransformBroadcaster OdomTFBroadCaster;
}