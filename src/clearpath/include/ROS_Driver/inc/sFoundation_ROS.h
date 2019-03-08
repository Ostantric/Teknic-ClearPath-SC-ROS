#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <geometry_msgs/Twist.h>
#include <clearpath/ClearpathClient.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <sstream>
#include <stdio.h>

#define DEFAULT_REALTIME_REG_HZ 50
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
  IPort *getPort;
  std::vector<INode *> getServolist;
};
struct servoinfo {
  double desired_velocity = 0, current_velocity = 0 ,
      current_velocity_command = 0;
  double difference_velocity = 0; 
  double desired_position = 0, current_position = 0 , difference_position = 0,
      current_position_command = 0;
  double current_torque = 0;
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
  ros::Subscriber vel_sub_1, vel_sub_2, vel_sub_3, vel_sub_4, estop_sub_,
      release_sub_;
  ros::Publisher realtime_reg_pub_;
  SysManager Manager;
  cpmStatusRegFlds temp_rt_reg;
  // std::vector<IPort *> portlist;
  std::vector<INode *> servolist;
  std::vector<Ports *> portlist;
  std::vector<servoinfo > servoinfolist;
  bool Release_Flag;
  bool EStop_Flag;
  
  int rtreg_status_publish_rate_, vel_limit_rpm_, acc_lim_per_sec_,
      torq_percantage_, decel_lim_per_sec_;
  int servo_id;
  Ports temp_port;
  double scale_linear_, scale_angular_;
  void cmdvelCallback_1(const geometry_msgs::Twist::ConstPtr &msg);
  void cmdvelCallback_2(const geometry_msgs::Twist::ConstPtr &msg);
  void cmdvelCallback_3(const geometry_msgs::Twist::ConstPtr &msg);
  void cmdvelCallback_4(const geometry_msgs::Twist::ConstPtr &msg);
  void estopCallback(const std_msgs::Bool::ConstPtr &msg);
  void releaseServosCallback(const std_msgs::Bool::ConstPtr &msg);
  void send_command(int *b, const double *x);
  
};