#include <iostream>
#include <sFoundation_ROS.h>
#include <stdio.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "clearpath_client");
  ClearPath_Driver clearpath;
  ros::Rate loop_rate(clearpath.rate);
  try {
    clearpath.connect();
    while (ros::ok()) // exit this if ros is down
    {
      clearpath.publish_rt_status();
      ros::spinOnce();
      loop_rate.sleep();
    }
    clearpath.disconnect();
  } catch (mnErr &theErr) { // Printable errors
    ROS_ERROR("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr,
              theErr.ErrorCode, theErr.ErrorMsg);
    clearpath.disconnect();
    return (2);
  } catch (...) { // default
    ROS_ERROR("Generic error caught on Clearpath-SC\n");
    clearpath.disconnect();
    return (3);
  }
  return 0;
}