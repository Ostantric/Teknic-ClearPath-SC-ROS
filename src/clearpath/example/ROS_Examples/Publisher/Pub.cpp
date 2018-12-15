///*******************************************************************************
/**
    ClearPath-SC ROS Example-Publishing string msg
    1)Create "Node_Info" topic - publisher
    2)Find Hub
    3)Find Node count
    4)Clear Nodes
    5)Initialize Node 0
    6)Publish position,velocity and torque
**/
//******************************************************************************

#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <stdio.h>

using namespace sFnd;
#define VEL_LIM_RPM 840
#define ACC_LIM_RPM_PER_SEC 500
#define TORQ_PERCENTAGE 100

int main(int argc, char **argv) {
  ros::init(argc, argv, "HUB_PUB");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Node_Info", 1000);
  ros::Rate loop_rate(150); // you can go faster than this maybe 200hz. but if
                            // you want to go real fast with lowest 4 node
                            // latency then take a look at multithread example.
  size_t portCount = 0;
  std::vector<std::string> comHubPorts;
  SysManager myMgr; // Create System Manager myMgr
  IPort *myPort;
  try {
    SysManager::FindComHubPorts(comHubPorts);
    ROS_INFO("Found %d SC Hubs\n", comHubPorts.size());
    for (portCount = 0;
         portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX;
         portCount++) {
      myMgr.ComHubPort(portCount,
                       comHubPorts[portCount].c_str()); // define the first SC
                                                        // Hub port (port 0) to
                                                        // be associated with
                                                        // COM portnum (as seen
                                                        // in device manager)
    }
    if (portCount < 0) {
      ROS_ERROR("Unable to locate SC hub port\n");
      return -1;
    }
    myMgr.PortsOpen(portCount); // Open the port
    for (size_t i = 0; i < portCount; i++) {
      myPort = &myMgr.Ports(i);
      ROS_INFO(" Port[%d]: state=%d, nodes=%d\n", myPort->NetNumber(),
               myPort->OpenState(), myPort->NodeCount());
    }
    // Clear Nodes and Enable them
    for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) {
      myPort->Nodes(iNode).Status.AlertsClear();
      myPort->Nodes(iNode).Motion.NodeStop(STOP_TYPE_ABRUPT);
      myPort->Nodes(iNode).Motion.NodeStop(STOP_TYPE_CLR_ALL);
      myPort->Nodes(iNode).Setup.Ex.App.AutoRefresh(true);
      myPort->Nodes(0).EnableReq(true);
    }
    // Node 0 init
    myPort->Nodes(0).Motion.AccLimit =
        ACC_LIM_RPM_PER_SEC; // Set Acceleration Limit (RPM/Sec)
    myPort->Nodes(0).Motion.VelLimit = VEL_LIM_RPM; // Set Velocity Limit (RPM)
    myPort->Nodes(0).Limits.TrqGlobal.Value(
        TORQ_PERCENTAGE); // Set Torque MAX (%)
    //
    myPort->Nodes(0).Motion.VelCommanded.AutoRefresh(true);
    myPort->Nodes(0).Motion.PosnMeasured.AutoRefresh(true);
    myPort->Nodes(0).Motion.TrqMeasured.AutoRefresh(true);
    double posn = myPort->Nodes(0).Motion.PosnMeasured.Value();
    myPort->Nodes(0).Motion.AddToPosition(-posn);

    myPort->Nodes(0).Motion.MoveVelStart(250); // Quick movevel example.

    while (ros::ok()) {
      double myVel = myPort->Nodes(0).Motion.VelCommanded.Value();
      double position = myPort->Nodes(0).Motion.PosnMeasured.Value();
      double torque = myPort->Nodes(0).Motion.TrqMeasured.Value();
      std_msgs::String msg;
      std::stringstream ss;
      ss << "pos:" << (intmax_t)position << " vel:" << myVel
         << " torq:" << torque;
      msg.data = ss.str();
      // ROS_INFO("%s", msg.data.c_str());
      chatter_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
    // Disable nodes
    for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) {
      myPort->Nodes(iNode).EnableReq(false);
    }
    myMgr.PortsClose();
  } catch (mnErr &theErr) { // Printable errors
    fprintf(stderr, "Caught error: addr=%d, err=0x%0x\nmsg=%s\n",
            theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
    printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr,
           theErr.ErrorCode, theErr.ErrorMsg);
    return (2);
  } catch (...) { // default
    fprintf(stderr, "Error generic caught\n");
    printf("Generic error caught\n");
    return (3);
  }
  return 0;
}
