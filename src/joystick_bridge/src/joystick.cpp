/*
Author: Murat Terzi
-Find Xbox 360 controller(TODO)
-Start reading joystick data
-Publish data
*/
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#define DEFAULT_ANGULER_SCALE 0.5
#define DEFAULT_LINEAR_SCALE 0.5
#define DEFAULT_CMD_VEL_HZ 100

/********** Creating class to publish and subscribe in one node
 * ********************/
class Teleop {
public: // can be used by main
  Teleop();
  ~Teleop();
  void publish_cmd_vel();
  void publish_Estop();

private:
  ros::Publisher vel_pub_, estop_pub_, release_pub_;
  ros::Subscriber joy_sub_, estop_sub_;
  geometry_msgs::Twist twist; // create Twist object
  std_msgs::Bool EStop_Flag, Release_Flag;
  int x, y, xbox_button, a, b, rightleft, updown, start_button, end_button;
  int x_, b_, xbox_button_, rightleft_, updown_, start_button_, end_button_;
  int publish_rate_;
  int linear_, angular_,vel_limit_rpm_;
  bool temp_estop;
  double l_scale_, a_scale_;
  double gear_reduction_, wheel_radius_, wheel_base_, wheel_base_multiplier_, wheel_radius_multiplier_, vehicle_width_, vehicle_width_multiplier_;
  float x_linear = 0.0, z_angular = 0.0;
  float linear_max = 0.0, angular_max = 0.0;
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void estopCallback(const std_msgs::Bool::ConstPtr &msg);
  ros::NodeHandle nh_; // create a handle
};
/***********************************************************************************/
// Constructor
Teleop::Teleop()
    : // creating a function to get parameters and setting up
      l_scale_(DEFAULT_LINEAR_SCALE),
      a_scale_(DEFAULT_ANGULER_SCALE), angular_(1), linear_(1), x_(2), b_(1),
      updown_(7), rightleft_(6), start_button_(7), end_button_(6),
      xbox_button_(8),
      vel_limit_rpm_(500),
      gear_reduction_(15),
      publish_rate_(
          DEFAULT_CMD_VEL_HZ) { // including default axes, buttons and rate
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("start_button", start_button_, start_button_);
  nh_.param("end_button", end_button_, end_button_);
  nh_.param("xbox_button", xbox_button_, xbox_button_);
  nh_.param("x", x_, x_);
  nh_.param("b", b_, b_);
  nh_.param("updown", updown_, updown_);
  nh_.param("rightleft", rightleft_, rightleft_);
  nh_.param("cmd_vel_hz", publish_rate_, publish_rate_);
  nh_.param("vel_limit_rpm", vel_limit_rpm_, vel_limit_rpm_);
  nh_.param("gear_reduction", gear_reduction_, gear_reduction_);
  nh_.param("wheel_radius", wheel_radius_, wheel_radius_);
  nh_.param("wheel_radius_multiplier", wheel_radius_multiplier_, wheel_radius_multiplier_);
  nh_.param("wheel_base", wheel_base_, wheel_base_);
  nh_.param("wheel_base_multiplier", wheel_base_multiplier_, wheel_base_multiplier_);
  nh_.param("vehicle_width", vehicle_width_, vehicle_width_);
  nh_.param("vehicle_width_multiplier", vehicle_width_multiplier_, vehicle_width_multiplier_);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  estop_pub_ = nh_.advertise<std_msgs::Bool>("estop", 1);
  release_pub_ = nh_.advertise<std_msgs::Bool>("release_servos", 1);
  joy_sub_ =
      nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
  estop_sub_ =
      nh_.subscribe<std_msgs::Bool>("estop", 1, &Teleop::estopCallback, this);
  EStop_Flag.data = false;
  Release_Flag.data = false;
  linear_max=((wheel_radius_*2*3.14))*(vel_limit_rpm_/(60*gear_reduction_));
  angular_max=((360*linear_max)/(2*3.14*vehicle_width_))/(360/(2*3.14));
}
// Destructor
Teleop::~Teleop() {}

// Publish CMD_VEL
void Teleop::publish_cmd_vel() {
  ros::Rate loop_rate(publish_rate_); // frequency
  while (ros::ok()) {                 // exit this if ros is down
    twist.angular.z = z_angular;
    twist.linear.x = x_linear;
    if (EStop_Flag.data) { // stop if estop pressed
      twist.angular.z = 0;
      twist.linear.x = 0;
    }
    vel_pub_.publish(twist); // publishing twist commands
    ros::spinOnce();         // necessary because there is a callback
    loop_rate.sleep();       // sleep for 1/publish_rate ms
  }
}
// This is an alternative E-STOP
void Teleop::estopCallback(const std_msgs::Bool::ConstPtr &msg) {
  EStop_Flag.data = msg->data;
}
// Read Joystick data
void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
  z_angular = angular_max * joy->axes[angular_] * a_scale_;
  x_linear =  linear_max * joy->axes[linear_] * l_scale_;
  // get buttons from the joy array
  b = joy->buttons[b_];
  x = joy->buttons[x_];
  start_button = joy->buttons[start_button_];
  end_button = joy->buttons[end_button_];
  xbox_button = joy->buttons[xbox_button_];
  rightleft = joy->axes[rightleft_];
  updown = joy->axes[updown_];
  if (xbox_button) {
    EStop_Flag.data = !EStop_Flag.data;
    estop_pub_.publish(EStop_Flag);
  }

  if (b) {
    Release_Flag.data = !Release_Flag.data;
    release_pub_.publish(Release_Flag);
  }
  // release_pub_.publish(Release_Flag);
}

int main(int argc, char **argv) {
  // TODO:add a search for xbox vendor id and product id
  // find a match (js*) under /dev/input/
  /*std::system("rosparam set joy_node/dev
  '/dev/input/js0'");std::system("rosrun joy joy_node");*/
  ros::init(argc, argv, "xbox_teleop");
  Teleop Xbox360_Controller; // create class object
  Xbox360_Controller.publish_cmd_vel();
}