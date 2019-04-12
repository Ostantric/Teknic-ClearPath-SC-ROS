///*******************************************************************************
/**
    ClearPath-SC ROS Example-Publishing string msg
    1)Create "Node_Info" topic - publisher
    2)Find Hub
    3)Find Node count
    4)Clear Nodes
    5)Initialize Node 0
    6)Publish position,velocity and torque

    TODO: implement multithread
**/
//******************************************************************************
#include <sFoundation_ROS.h>

using namespace sFnd;
// Constructor
ClearPath_Driver::ClearPath_Driver()
    : rtreg_status_publish_rate_(DEFAULT_REALTIME_REG_HZ),
      vel_limit_rpm_(DEFAULT_VEL_RPM_LIMIT),
      acc_lim_per_sec_(DEFAULT_ACC_RPM_SEC_LIMIT),
      decel_lim_per_sec_(DEFAULT_DECEL_RPM_SEC_LIMIT),
      torq_percantage_(DEFAULT_TORQ_PERC_LIMIT),
      scale_angular_(DEFAULT_ANGULER_SCALE), EStop_Flag(false),
      Release_Flag(false), scale_linear_(DEFAULT_LINEAR_SCALE) {
  nh_.param("realtime_reg_status_publish_rate", rtreg_status_publish_rate_,
            rtreg_status_publish_rate_);
  nh_.param("vel_limit_rpm", vel_limit_rpm_, vel_limit_rpm_);
  nh_.param("acc_lim_per_sec", acc_lim_per_sec_, acc_lim_per_sec_);
  nh_.param("decel_lim_per_sec", decel_lim_per_sec_, decel_lim_per_sec_);
  nh_.param("torq_percantage", torq_percantage_, torq_percantage_);
  nh_.param("scale_angular", scale_angular_,
            scale_angular_); // angular axis becomes axis in twist
  nh_.param("scale_linear", scale_linear_,
            scale_linear_); // linear axis becomes linear in twist
  nh_.param("gear_reduction",gear_reduction_,gear_reduction_);
  nh_.param("wheel_radius", wheel_radius_, wheel_radius_);
  nh_.param("wheel_radius_multiplier", wheel_radius_multiplier_, wheel_radius_multiplier_);
  nh_.param("wheel_base", wheel_base_, wheel_base_);
  nh_.param("wheel_base_multiplier", wheel_base_multiplier_, wheel_base_multiplier_);
  nh_.param("vehicle_width", vehicle_width_, vehicle_width_);
  nh_.param("vehicle_width_multiplier", vehicle_width_multiplier_, vehicle_width_multiplier_);
  rate = rtreg_status_publish_rate_;
  vel_sub_1 = nh_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1000, &ClearPath_Driver::cmdvelCallback_1, this);
  vel_sub_2 = nh_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1000, &ClearPath_Driver::cmdvelCallback_2, this);
  /*vel_sub_3 = nh_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1000, &ClearPath_Driver::cmdvelCallback_3, this);
  vel_sub_4 = nh_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1000, &ClearPath_Driver::cmdvelCallback_4, this);*/
  estop_sub_ = nh_.subscribe<std_msgs::Bool>(
      "estop", 1000, &ClearPath_Driver::estopCallback, this);
  release_sub_ = nh_.subscribe<std_msgs::Bool>(
      "release_servos", 1000, &ClearPath_Driver::releaseServosCallback, this);
  realtime_reg_pub_ =
      nh_.advertise<clearpath::ClearpathClient>("realtime_reg", 10);
}
// Destructor
ClearPath_Driver::~ClearPath_Driver() { disconnect(); }
// Find Hub & Connect
int ClearPath_Driver::connect() {
  SysManager::FindComHubPorts(comHubPorts);
  ROS_INFO("Found %d SC Hubs\n", comHubPorts.size());
  for (portCount = 0;
       portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX;
       portCount++) {
    Manager.ComHubPort(portCount,
                       comHubPorts[portCount].c_str()); // define the first SC
                                                        // Hub port (port 0) to
                                                        // be associated
    // with COM portnum (as seen in device manager)
  }
  if (portCount < 0) {
    ROS_ERROR("Unable to locate SC hub port\n");
    return -1;
  }
  Manager.PortsOpen(portCount); // Open the port
  // check each port

  for (size_t port_n = 0; port_n < portCount; port_n++) {
    temp_port.getPort = &Manager.Ports(port_n);
    // portlist.push_back(&Manager.Ports(port_n));

    ROS_INFO(" Port[%d]: state=%d, nodes=%d\n", temp_port.getPort->NetNumber(),
             temp_port.getPort->OpenState(), temp_port.getPort->NodeCount());
    servoinfolist.resize(temp_port.getPort->NodeCount());
    // Node init
    for (size_t node_n = 0; node_n < temp_port.getPort->NodeCount(); node_n++) {
      temp_port.getPort->Nodes(node_n).Status.AlertsClear();
      temp_port.getPort->Nodes(node_n).Motion.NodeStop(STOP_TYPE_ABRUPT);
      temp_port.getPort->Nodes(node_n).Motion.NodeStop(STOP_TYPE_CLR_ALL);
      temp_port.getPort->Nodes(node_n).Setup.Ex.App.AutoRefresh(true);
      temp_port.getPort->Nodes(node_n).EnableReq(true);

      // Make sure we are talking to a ClearPath SC (advanced or basic model
      // will work)
      if (temp_port.getPort->Nodes(node_n).Info.NodeType() !=
              IInfo::CLEARPATH_SC_ADV &&
          temp_port.getPort->Nodes(node_n).Info.NodeType() !=
              IInfo::CLEARPATH_SC) {
        ROS_ERROR("In port %d Node %d is not a ClearPath-SC Motor\n", port_n,
                  node_n);
        return -1;
      }
      if (!temp_port.getPort->Nodes(node_n).Setup.AccessLevelIsFull()) {
        ROS_ERROR("Need a full access for Node %d under Port \n", node_n,
                  port_n);
        return -1;
      }
      temp_port.getServolist.push_back((&temp_port.getPort->Nodes(node_n)));
      temp_port.getServolist.at(node_n)->Motion.AccLimit =
          acc_lim_per_sec_; // Set Acceleration Limit (RPM/Sec)
      temp_port.getServolist.at(node_n)->Motion.VelLimit =
          vel_limit_rpm_; // Set Velocity Limit (RPM)
      temp_port.getServolist.at(node_n)->Limits.TrqGlobal.Value(
          torq_percantage_); // Set Torque MAX (%)
      temp_port.getServolist.at(node_n)->Motion.NodeStopDecelLim =
          decel_lim_per_sec_; // Set Deceleration Limit (RPM/sec)
      // Autorefresh on
      // this might cause problems, check it later
      temp_port.getServolist.at(node_n)->Motion.VelCommanded.AutoRefresh(true);
      temp_port.getServolist.at(node_n)->Motion.VelMeasured.AutoRefresh(true);
      temp_port.getServolist.at(node_n)->Motion.PosnMeasured.AutoRefresh(true);
      temp_port.getServolist.at(node_n)->Motion.TrqMeasured.AutoRefresh(true);
      temp_port.getServolist.at(node_n)->Status.RT.AutoRefresh(true);
      // Zero out position
      double posn = temp_port.getServolist.at(node_n)->Motion.PosnMeasured.Value();
      temp_port.getServolist.at(node_n)->Motion.AddToPosition(-posn);
      //std::fill(servoinfolist.begin(), servoinfolist.end(), 0.0);
      
    }

    temp_port.getServolist.at(0)->Info.UserID = "front_right";
    temp_port.getServolist.at(1)->Info.UserID = "front_left";
    // temp_port.getServolist.at(1)->Info.UserID = "1";
    // temp_port.getServolist.at(2)->Info.UserID = "2";
    // temp_port.getServolist.at(3)->Info.UserID = "3";

    portlist.push_back(&temp_port);
  }
}
// Disconnect
void ClearPath_Driver::disconnect() {
  for (int a = 0; a < portlist.size(); a++) {
    for (size_t i = 0; i < portlist.at(0)->getServolist.size(); i++) {
      portlist.at(0)->getServolist.at(i)->EnableReq(false);
      delete portlist.at(a)->getServolist.at(i);
    }
    delete portlist.at(a)->getPort;
  }
  Manager.PortsClose();
  servoinfolist.shrink_to_fit();
  portlist.shrink_to_fit();
  servolist.shrink_to_fit();
}

// Publish Real-Time Stats
void ClearPath_Driver::publish_rt_status() {
  clearpath::ClearpathClient client;
  clearpath::Port port;
  clearpath::Servo servo;
  client.numberOfPorts = portlist.size();
  //ROS_INFO("listsize: %d", client.numberOfPorts);
  for (int i = 0; i < portlist.size(); i++) {
    port.port_id = portlist.at(i)->getPort->NetNumber();
    port.numberOfServos = portlist.at(i)->getPort->NodeCount();
    //ROS_INFO("servos: %d", port.numberOfServos);
    for (int a = 0; a < portlist.at(i)->getServolist.size(); a++) {
      servo.servo_id = portlist.at(i)->getServolist.at(a)->Info.UserID.Value();

      servo.position =
          portlist.at(i)->getServolist.at(a)->Motion.PosnMeasured.Value();
      servo.velocity =
          portlist.at(i)->getServolist.at(a)->Motion.VelMeasured.Value();
      servo.torque =
          portlist.at(i)->getServolist.at(a)->Motion.TrqMeasured.Value();
      //ROS_INFO("pos: %f", portlist.at(a)->getServolist.at(i)->Motion.PosnMeasured.Value());
      port.Servos.push_back(servo);
    }
  }

  client.header.stamp = ros::Time::now();
  client.Ports.push_back(port);
  realtime_reg_pub_.publish(client);
}
// Execute CM_VEL commands
int a = 0; // temp port number init
void ClearPath_Driver::send_command(int *b, const double *x){
  int i = *b;
  double c = *x;
  servoinfolist.at(i).desired_velocity = c;
    temp_rt_reg = portlist.at(a)->getServolist.at(i)->Status.RT.Value().cpm;
    if (temp_rt_reg.Enabled) {
      //ROS_INFO("hi");
      switch (temp_rt_reg.InMotion) {
      case 0: // near zero from last check
        if (!(-5 < servoinfolist.at(i).desired_velocity && servoinfolist.at(i).desired_velocity < 5)) {
          if (temp_rt_reg.MoveBufAvail) {
            portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(
                servoinfolist.at(i).desired_velocity);
  #ifdef DEBUG
          ROS_INFO("near_zero,servoinfolist.at(0).desired_velocity: %f, current_Velocity : %f",
                   servoinfolist.at(i).desired_velocity, servoinfolist.at(i).current_velocity);
  #endif
          }
        } else {
          portlist.at(a)->getServolist.at(i)->Motion.NodeStop(
              STOP_TYPE_RAMP_AT_DECEL);
          portlist.at(a)->getServolist.at(i)->Motion.NodeStopClear();
        }
        break;
      case 1: // positive direction from last check
        if (servoinfolist.at(i).current_velocity_command >
            servoinfolist.at(i).desired_velocity) {
          if (servoinfolist.at(i).desired_velocity > 0) {
            if (temp_rt_reg.MoveBufAvail) {
              portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(
                  servoinfolist.at(i).desired_velocity);
            }
    #ifdef DEBUG
          ROS_INFO(
              "positive-notavail,servoinfolist.at(i).desired_velocity: %f, current_Velocity : %f",
              servoinfolist.at(i).desired_velocity, servoinfolist.at(i).current_velocity);
    #endif
        } else {
    #ifdef DEBUG
          ROS_INFO("positive-else,servoinfolist.at(i).desired_velocity: %f, current_Velocity : %f",
                   servoinfolist.at(i).desired_velocity, servoinfolist.at(i).current_velocity);
    #endif
          portlist.at(a)->getServolist.at(i)->Motion.NodeStop(
              STOP_TYPE_RAMP_AT_DECEL);
          portlist.at(a)->getServolist.at(i)->Motion.NodeStopClear();
          portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(servoinfolist.at(i).desired_velocity);
        }
      } else if (servoinfolist.at(i).current_velocity_command < servoinfolist.at(i).desired_velocity) {
        if (temp_rt_reg.AtTargetVel) {
          portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(servoinfolist.at(i).desired_velocity);
        }
      } else {
        if (temp_rt_reg.MoveBufAvail) {
          portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(servoinfolist.at(i).desired_velocity);
        } else {
    #ifdef DEBUG
          ROS_INFO("positive-equals-notavail,servoinfolist.at(i).desired_velocity: %f, "
                   "current_Velocity : %f",
                   servoinfolist.at(i).desired_velocity, servoinfolist.at(i).current_velocity);
    #endif
        }
      }
      break;
    case 2: // negative direction from last check
      if (servoinfolist.at(i).current_velocity_command < servoinfolist.at(i).desired_velocity) {
        if (servoinfolist.at(i).desired_velocity < 0) {
          if (temp_rt_reg.MoveBufAvail) {
            portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(
                servoinfolist.at(i).desired_velocity);
          }
    #ifdef DEBUG
          ROS_INFO(
              "negative-notavail,servoinfolist.at(i).desired_velocity: %f, current_Velocity : %f",
              servoinfolist.at(i).desired_velocity, servoinfolist.at(i).current_velocity);
    #endif
        } else {
    #ifdef DEBUG
          ROS_INFO("negative-else,servoinfolist.at(i).desired_velocity: %f, current_Velocity : %f",
                   servoinfolist.at(i).desired_velocity, servoinfolist.at(i).current_velocity);
    #endif
          portlist.at(a)->getServolist.at(i)->Motion.NodeStop(
              STOP_TYPE_RAMP_AT_DECEL);
          portlist.at(a)->getServolist.at(i)->Motion.NodeStopClear();
          portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(servoinfolist.at(i).desired_velocity);
        }
      } else if (servoinfolist.at(i).current_velocity_command > servoinfolist.at(i).desired_velocity) {
        if (temp_rt_reg.AtTargetVel) {
          portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(servoinfolist.at(i).desired_velocity);
        }
      } else {
        if (temp_rt_reg.MoveBufAvail) {
          portlist.at(a)->getServolist.at(i)->Motion.MoveVelStart(servoinfolist.at(i).desired_velocity);
        } else {
    #ifdef DEBUG
          ROS_INFO("negative-equals-notavail,servoinfolist.at(i).desired_velocity: %f, "
                   "current_Velocity : %f", 
                   servoinfolist.at(i).desired_velocity, servoinfolist.at(i).current_velocity);
    #endif
        }
      }
      break;
    case 3: // both direction from last check
      // update here for slow loops
      ROS_ERROR(
          "NOTHING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      break;
    default:
      break;
      // nothing
    }
    servoinfolist.at(i).current_velocity_command =
        servoinfolist.at(i).desired_velocity;
    }
}
//TODO:make these as one function with clearpath multi-threaded layer
void ClearPath_Driver::cmdvelCallback_1(//servo 0
    const geometry_msgs::Twist::ConstPtr &msg) {
    int servo_no = 0;
    right_rad_per_second_ = (msg->linear.x + msg->angular.z * (vehicle_width_*vehicle_width_multiplier_)/2.0)/wheel_radius_;
    //right_rad_per_second_=sqrt(pow(msg->linear.x,2)+pow((msg->angular.z*vehicle_width_)/2,2))/wheel_radius_;
    const double vel_rpm = (right_rad_per_second_/(2*3.14)*60)*gear_reduction_;
    ROS_INFO("vel0: %f", vel_rpm);
    send_command(&servo_no,&vel_rpm);
}
void ClearPath_Driver::cmdvelCallback_2(//servo 1
    const geometry_msgs::Twist::ConstPtr &msg) {
    int servo_no = 1;
    left_rad_per_second_ = (msg->linear.x - msg->angular.z * (vehicle_width_*vehicle_width_multiplier_)/2.0)/wheel_radius_;
    const double vel_rpm = (left_rad_per_second_/(2*3.14)*60)*gear_reduction_;
    ROS_INFO("vel1: %f", vel_rpm);
    send_command(&servo_no,&vel_rpm);
}
void ClearPath_Driver::cmdvelCallback_3(//servo 2
    const geometry_msgs::Twist::ConstPtr &msg) {
    int servo_no = 2;
    left_rad_per_second_ = (msg->linear.x - msg->angular.z * (vehicle_width_*vehicle_width_multiplier_)/2.0)/wheel_radius_;
    const double vel_rpm = (left_rad_per_second_/(2*3.14)*60)*gear_reduction_;
    ROS_INFO("vel3: %f", vel_rpm);
    send_command(&servo_no,&vel_rpm);
}
void ClearPath_Driver::cmdvelCallback_4(//servo 3
    const geometry_msgs::Twist::ConstPtr &msg) {
    int servo_no = 3;
    right_rad_per_second_ = (msg->linear.x + msg->angular.z * (vehicle_width_*vehicle_width_multiplier_)/2.0)/wheel_radius_;
    const double vel_rpm = (right_rad_per_second_/(2*3.14)*60)*gear_reduction_;
    ROS_INFO("vel3: %f", vel_rpm);
    send_command(&servo_no,&vel_rpm);
}
// Check for E-STOP
void ClearPath_Driver::estopCallback(const std_msgs::Bool::ConstPtr &msg) {

  EStop_Flag = msg->data;
  if (EStop_Flag) {
#ifdef DEBUG
    ROS_INFO("estop pressed");
#endif
    portlist.at(a)->getServolist.at(0)->Motion.NodeStop(STOP_TYPE_RAMP_AT_DECEL);
  } else {
#ifdef DEBUG
    ROS_INFO("estop released");
#endif
    portlist.at(a)->getServolist.at(0)->Motion.NodeStopClear();
  }
}
// Check for Release/Unrelease
// Releasing the node will make the motor run freely.
void ClearPath_Driver::releaseServosCallback(
    const std_msgs::Bool::ConstPtr &msg) {

  Release_Flag = msg->data;
  if (Release_Flag) {
#ifdef DEBUG
    ROS_INFO("Servos-released");
#endif
    for (int i = 0; i < portlist.at(a)->getServolist.size(); i++) {
      portlist.at(a)->getServolist.at(i)->Motion.NodeStop(STOP_TYPE_RAMP_AT_DECEL);
      portlist.at(a)->getServolist.at(i)->EnableReq(false);
    }
  } else {
#ifdef DEBUG
    ROS_INFO("Servos-Engaged");
#endif DEBUG
    for (int i = 0; i < portlist.at(a)->getServolist.size(); i++) {
      portlist.at(a)->getServolist.at(i)->EnableReq(true);
    }
  }
}
