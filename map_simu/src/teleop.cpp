#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sstream>

class Teleop
{
public:
  Teleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int throttle, yaw, pitch, roll, ROLL, PITCH, THROTTLE, YAW, mid, offset;
  
  ros::Publisher rc_message;
  ros::Subscriber joy_sub;

};

Teleop::Teleop():
// Joystick side
  throttle(1),
  yaw(0),
  pitch(5),
  roll(4),
// Drone channels
  ROLL(0),
  PITCH(1),
  THROTTLE(2),
  YAW(3),
// Other values
  mid(1500),
  offset(500)
{
  rc_message = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1, true);
  joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  mavros_msgs::OverrideRCIn rc_command;
  rc_command.channels[THROTTLE] = mid + offset*joy->axes[throttle];
  rc_command.channels[YAW] = mid + offset*joy->axes[yaw];
  rc_command.channels[PITCH] = mid - offset*joy->axes[pitch];
  rc_command.channels[ROLL] = mid - offset*joy->axes[roll];

  rc_message.publish(rc_command);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  Teleop teleop;

  ros::spin();
}