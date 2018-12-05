#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/Control.h"
#include "std_msgs/Empty.h"
#include <sstream>


int main(int argc, char **argv) {

  ros::init(argc, argv, "intelli_bot");

  ros::NodeHandle n;

  Control _control;

  _control.setPathPts();

  ros::Subscriber sub = n.subscribe("/ground_truth/state", 1000,
                                    &Control::navMessageReceived, &_control);
  ros::Publisher takeOff_msg = n.advertise<std_msgs::Empty>("/ardrone/takeoff",
                                                            1, true);
  std_msgs::Empty emptyMsg;
  takeOff_msg.publish(emptyMsg);

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {

    geometry_msgs::Twist new_vel;

    // get Twist msg from control class algorithm

    new_vel = _control.getVelocityPose();
    ROS_INFO_STREAM(
        "vel_pub = " << new_vel.linear.x << "," << new_vel.linear.y << ","<< new_vel.linear.z);

    // Publish the computed velocity
    vel_pub.publish(new_vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
