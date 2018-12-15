///*******************************************************************************
/**
 * ClearPath-SC ROS Example-Subscriber
    1)Create "Node_Velocity" topic - subscriber
    2)Find Hub
    3)Find Node count
    4)Clear Nodes
    5)Initialize Node 0
    7)Wait for the callback
**/
//******************************************************************************

#include <stdio.h>		
#include <iostream>
#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>

using namespace sFnd;	
#define VEL_LIM_RPM 840
#define ACC_LIM_RPM_PER_SEC	500
#define TORQ_PERCENTAGE 100
IPort *myPort;

void Node_VelocityCallback(const std_msgs::Float64::ConstPtr& msg)
{
  double vel= msg->data;
  myPort->Nodes(0).Motion.MoveVelStart(vel);//update node 0
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "HUB_PUB");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("Node_Velocity", 1000, Node_VelocityCallback);
  ros::Rate loop_rate(200);
  size_t portCount = 0;
	std::vector<std::string> comHubPorts;
	SysManager myMgr;							//Create System Manager myMgr
  
  try
	{ 
    SysManager::FindComHubPorts(comHubPorts);
    ROS_INFO("Found %d SC Hubs\n", comHubPorts.size());
    for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			myMgr.ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated with COM portnum (as seen in device manager)
		}
    if (portCount < 0) {
			ROS_ERROR("Unable to locate SC hub port\n");
			return -1;
		}
    myMgr.PortsOpen(portCount);				//Open the port
    for (size_t i = 0; i < portCount; i++) {
     myPort = &myMgr.Ports(i);
    ROS_INFO(" Port[%d]: state=%d, nodes=%d\n",myPort->NetNumber(), myPort->OpenState(), myPort->NodeCount());
    }
    //Clear Alerts and Enable Nodes
    for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) {
        myPort->Nodes(iNode).Status.AlertsClear();
		    myPort->Nodes(iNode).Motion.NodeStop(STOP_TYPE_ABRUPT);
		    myPort->Nodes(iNode).Motion.NodeStop(STOP_TYPE_CLR_ALL);
        myPort->Nodes(iNode).Setup.Ex.App.AutoRefresh(true);
        myPort->Nodes(0).EnableReq(true);
			}
    //Node 0 init
    myPort->Nodes(0).Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
    myPort->Nodes(0).Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)
    myPort->Nodes(0).Limits.TrqGlobal.Value(TORQ_PERCENTAGE); // Set Torque MAX (%)
    //... more nodes can be added here
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    //Disable nodes
    for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) {
				myPort->Nodes(iNode).EnableReq(false);
			}
      myMgr.PortsClose();
  }
  catch (mnErr& theErr) { //Printable errors
		fprintf(stderr, "Caught error: addr=%d, err=0x%0x\nmsg=%s\n",
				theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n",
				theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		return(2);
	}
	catch (...) { //default
		fprintf(stderr, "Error generic caught\n");
		printf("Generic error caught\n");
		return(3);
	}
  return 0;
}

