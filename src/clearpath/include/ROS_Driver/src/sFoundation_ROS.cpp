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
      Release_Flag(false), scale_linear_(DEFAULT_LINEAR_SCALE),
      current_velocity(DEFAULT_START_POINT),
      current_position(DEFAULT_START_POINT),
      difference_position(DEFAULT_START_POINT),
      difference_velocity(DEFAULT_START_POINT),
      desired_position(DEFAULT_START_POINT),
      desired_velocity(DEFAULT_START_POINT),
      current_velocity_command(DEFAULT_START_POINT) {
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

  rate = rtreg_status_publish_rate_;
  vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1000, &ClearPath_Driver::cmdvelCallback, this);
  estop_sub_ = nh_.subscribe<std_msgs::Bool>(
      "estop", 1000, &ClearPath_Driver::estopCallback, this);
  release_sub_ = nh_.subscribe<std_msgs::Bool>(
      "release_servos", 1000, &ClearPath_Driver::releaseServosCallback, this);
  realtime_reg_pub_ = nh_.advertise<std_msgs::String>("realtime_reg", 10);
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
    temp_port.port = &Manager.Ports(port_n);
    // portlist.push_back(&Manager.Ports(port_n));

    ROS_INFO(" Port[%d]: state=%d, nodes=%d\n", temp_port.port->NetNumber(),
             temp_port.port->OpenState(), temp_port.port->NodeCount());

    // Node init
    for (size_t node_n = 0; node_n < temp_port.port->NodeCount(); node_n++) {
      temp_port.port->Nodes(node_n).Status.AlertsClear();
      temp_port.port->Nodes(node_n).Motion.NodeStop(STOP_TYPE_ABRUPT);
      temp_port.port->Nodes(node_n).Motion.NodeStop(STOP_TYPE_CLR_ALL);
      temp_port.port->Nodes(node_n).Setup.Ex.App.AutoRefresh(true);
      temp_port.port->Nodes(node_n).EnableReq(true);

      // Make sure we are talking to a ClearPath SC (advanced or basic model
      // will work)
      if (temp_port.port->Nodes(node_n).Info.NodeType() !=
              IInfo::CLEARPATH_SC_ADV &&
          temp_port.port->Nodes(node_n).Info.NodeType() !=
              IInfo::CLEARPATH_SC) {
        ROS_ERROR("In port %d Node %d is not a ClearPath-SC Motor\n", port_n,
                  node_n);
        return -1;
      }
      if (!temp_port.port->Nodes(node_n).Setup.AccessLevelIsFull()) {
        ROS_ERROR("Need a full access for Node %d under Port \n", node_n,
                  port_n);
        return -1;
      }
      temp_port.servolist.push_back((&temp_port.port->Nodes(node_n)));
      temp_port.servolist.at(node_n)->Motion.AccLimit =
          acc_lim_per_sec_; // Set Acceleration Limit (RPM/Sec)
      temp_port.servolist.at(node_n)->Motion.VelLimit =
          vel_limit_rpm_; // Set Velocity Limit (RPM)
      temp_port.servolist.at(node_n)->Limits.TrqGlobal.Value(
          torq_percantage_); // Set Torque MAX (%)
      temp_port.servolist.at(node_n)->Motion.NodeStopDecelLim =
          decel_lim_per_sec_; // Set Deceleration Limit (RPM/sec)
      // Autorefresh on
      // this might cause problems, check it later
      temp_port.servolist.at(node_n)->Motion.VelCommanded.AutoRefresh(true);
      temp_port.servolist.at(node_n)->Motion.VelMeasured.AutoRefresh(true);
      temp_port.servolist.at(node_n)->Motion.PosnMeasured.AutoRefresh(true);
      temp_port.servolist.at(node_n)->Motion.TrqMeasured.AutoRefresh(true);
      temp_port.servolist.at(node_n)->Status.RT.AutoRefresh(true);
      // Zero out position
      double posn = temp_port.servolist.at(node_n)->Motion.PosnMeasured.Value();
      temp_port.servolist.at(node_n)->Motion.AddToPosition(-posn);
    }

    // temp_port.servolist.at(0)->Info.UserID = "0";
    // temp_port.servolist.at(1)->Info.UserID = "1";
    // temp_port.servolist.at(2)->Info.UserID = "2";
    // temp_port.servolist.at(3)->Info.UserID = "3";

    portlist.push_back(temp_port);
  }
}
// Disconnect
void ClearPath_Driver::disconnect() {
  for (int a = 0; a < portlist.size(); a++) {
    for (size_t i = 0; i < portlist.at(0).servolist.size(); i++) {
      portlist.at(0).servolist.at(i)->EnableReq(false);
      delete portlist.at(a).servolist.at(i);
    }
    delete portlist.at(a).port;
  }
  Manager.PortsClose();
}

// Publish Real-Time Stats
void ClearPath_Driver::publish_rt_status() {

  for (int i = 0; i < portlist.at(0).servolist.size(); i++) {
    servo_id = std::stoi(portlist.at(0).servolist.at(i)->Info.UserID.Value());
    current_velocity =
        portlist.at(0).servolist.at(i)->Motion.VelMeasured.Value();
    current_position =
        portlist.at(0).servolist.at(i)->Motion.VelMeasured.Value();
    current_torque = portlist.at(0).servolist.at(i)->Motion.VelMeasured.Value();
    ROS_INFO("position: %f, current_Velocity : %f", current_position,
             current_velocity);
  }

  // ROS_INFO("hi");
}
// Execute CM_VEL commands
int a = 0; // temp port number init
void ClearPath_Driver::cmdvelCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {

  desired_velocity = (vel_limit_rpm_ / scale_linear_) * (msg->linear.x);
  temp_rt_reg = portlist.at(a).servolist.at(0)->Status.RT.Value().cpm;
  if (temp_rt_reg.Enabled) {
    switch (temp_rt_reg.InMotion) {
    case 0: // near zero from last check
      if (!(-5 < desired_velocity && desired_velocity < 5)) {
        if (temp_rt_reg.MoveBufAvail) {
          portlist.at(a).servolist.at(0)->Motion.MoveVelStart(desired_velocity);
#ifdef DEBUG
          ROS_INFO("near_zero,desired_velocity: %f, current_Velocity : %f",
                   desired_velocity, current_velocity);
#endif
        }
      } else {
        portlist.at(a).servolist.at(0)->Motion.NodeStop(
            STOP_TYPE_RAMP_AT_DECEL);
        portlist.at(a).servolist.at(0)->Motion.NodeStopClear();
      }
      break;
    case 1: // positive direction from last check
      if (current_velocity_command > desired_velocity) {
        if (desired_velocity > 0) {
          if (temp_rt_reg.MoveBufAvail) {
            portlist.at(a).servolist.at(0)->Motion.MoveVelStart(
                desired_velocity);
          }
#ifdef DEBUG
          ROS_INFO(
              "positive-notavail,desired_velocity: %f, current_Velocity : %f",
              desired_velocity, current_velocity);
#endif
        } else {
#ifdef DEBUG
          ROS_INFO("positive-else,desired_velocity: %f, current_Velocity : %f",
                   desired_velocity, current_velocity);
#endif
          portlist.at(a).servolist.at(0)->Motion.NodeStop(
              STOP_TYPE_RAMP_AT_DECEL);
          portlist.at(a).servolist.at(0)->Motion.NodeStopClear();
          portlist.at(a).servolist.at(0)->Motion.MoveVelStart(desired_velocity);
        }
      } else if (current_velocity_command < desired_velocity) {
        if (temp_rt_reg.AtTargetVel) {
          portlist.at(a).servolist.at(0)->Motion.MoveVelStart(desired_velocity);
        }
      } else {
        if (temp_rt_reg.MoveBufAvail) {
          portlist.at(a).servolist.at(0)->Motion.MoveVelStart(desired_velocity);
        } else {
#ifdef DEBUG
          ROS_INFO("positive-equals-notavail,desired_velocity: %f, "
                   "current_Velocity : %f",
                   desired_velocity, current_velocity);
#endif
        }
      }
      break;
    case 2: // negative direction from last check
      if (current_velocity_command < desired_velocity) {
        if (desired_velocity < 0) {
          if (temp_rt_reg.MoveBufAvail) {
            portlist.at(a).servolist.at(0)->Motion.MoveVelStart(
                desired_velocity);
          }
#ifdef DEBUG
          ROS_INFO(
              "negative-notavail,desired_velocity: %f, current_Velocity : %f",
              desired_velocity, current_velocity);
#endif
        } else {
#ifdef DEBUG
          ROS_INFO("negative-else,desired_velocity: %f, current_Velocity : %f",
                   desired_velocity, current_velocity);
#endif
          portlist.at(a).servolist.at(0)->Motion.NodeStop(
              STOP_TYPE_RAMP_AT_DECEL);
          portlist.at(a).servolist.at(0)->Motion.NodeStopClear();
          portlist.at(a).servolist.at(0)->Motion.MoveVelStart(desired_velocity);
        }
      } else if (current_velocity_command > desired_velocity) {
        if (temp_rt_reg.AtTargetVel) {
          portlist.at(a).servolist.at(0)->Motion.MoveVelStart(desired_velocity);
        }
      } else {
        if (temp_rt_reg.MoveBufAvail) {
          portlist.at(a).servolist.at(0)->Motion.MoveVelStart(desired_velocity);
        } else {
#ifdef DEBUG
          ROS_INFO("negative-equals-notavail,desired_velocity: %f, "
                   "current_Velocity : %f",
                   desired_velocity, current_velocity);
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
    current_velocity_command = desired_velocity;
  }
}
// Check for E-STOP
void ClearPath_Driver::estopCallback(const std_msgs::Bool::ConstPtr &msg) {

  EStop_Flag = msg->data;
  if (EStop_Flag) {
#ifdef DEBUG
    ROS_INFO("estop pressed");
#endif
    portlist.at(a).servolist.at(0)->Motion.NodeStop(STOP_TYPE_RAMP_AT_DECEL);
  } else {
#ifdef DEBUG
    ROS_INFO("estop released");
#endif
    portlist.at(a).servolist.at(0)->Motion.NodeStopClear();
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
    for (int i = 0; i < portlist.at(a).servolist.size(); i++) {
      portlist.at(a).servolist.at(i)->Motion.NodeStop(STOP_TYPE_RAMP_AT_DECEL);
      portlist.at(a).servolist.at(i)->EnableReq(false);
    }
  } else {
#ifdef DEBUG
    ROS_INFO("Servos-Engaged");
#endif DEBUG
    for (int i = 0; i < portlist.at(a).servolist.size(); i++) {
      portlist.at(a).servolist.at(i)->EnableReq(true);
    }
  }
}
