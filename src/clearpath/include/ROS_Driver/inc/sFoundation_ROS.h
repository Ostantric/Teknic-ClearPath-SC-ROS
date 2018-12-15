#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdio.h>

#define DEFAULT_REALTIME_REG_HZ 100
#define DEFAULT_VEL_RPM_LIMIT 840
#define DEFAULT_ACC_RPM_SEC_LIMIT 800
#define DEFAULT_DECEL_RPM_SEC_LIMIT 800
#define DEFAULT_TORQ_PERC_LIMIT 100
#define DEFAULT_ANGULER_SCALE 0.5
#define DEFAULT_LINEAR_SCALE 0.5
#define DEFAULT_START_POINT 0
//#define DEBUG

using namespace sFnd;
/****** Creating class to publish and subscribe in one node
 ******/
struct Ports {
  IPort *port;
  std::vector<INode *> servolist;
};
class ClearPath_Driver {
public:
  size_t portCount = 0;
  std::vector<std::string> comHubPorts;
  int rate;
  ClearPath_Driver();
  ~ClearPath_Driver();
  int connect();
  void disconnect();
  void publish_rt_status();

private:
  ros::NodeHandle nh_;
  ros::Subscriber vel_sub_, estop_sub_, release_sub_;
  ros::Publisher realtime_reg_pub_;
  SysManager Manager;
  cpmStatusRegFlds temp_rt_reg;
   //std::vector<IPort *> portlist;
   std::vector<INode *> servolist;
  std::vector<Ports> portlist;
  bool Release_Flag;
  bool EStop_Flag;
  double desired_velocity, current_velocity, difference_velocity,
      current_velocity_command;
  double desired_position, current_position, difference_position,
      current_position_command;
  double current_torque;
  int rtreg_status_publish_rate_, vel_limit_rpm_, acc_lim_per_sec_,
      torq_percantage_, decel_lim_per_sec_;
  int servo_id;
  Ports temp_port;
  double scale_linear_, scale_angular_;
  void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void estopCallback(const std_msgs::Bool::ConstPtr &msg);
  void releaseServosCallback(const std_msgs::Bool::ConstPtr &msg);
};