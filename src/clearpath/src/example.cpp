//Required include files
#include <stdio.h>		
#include <iostream>
#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace sFnd;	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "HUB_PUB");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Node_Chat", 1000);
  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "test ";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
