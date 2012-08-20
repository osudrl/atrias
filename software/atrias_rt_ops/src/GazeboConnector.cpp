#include "atrias_rt_ops/GazeboConnector.h"

GazeboConnector::GazeboConnector(RTOps* rt_ops) :
                 Communicator(0) {
	rtOps      = rt_ops;
	guiEnabled = false;
	eStopped   = false;
}

GazeboConnector::~GazeboConnector() {
  ros::shutdown();
}

bool GazeboConnector::init() {
  // Setup ROS
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "controller_to_gazebo_connector");

  // Wait until the simulation has started
  std::vector<ros::master::TopicInfo> topicInfo;
	ros::spinOnce();
  ros::master::getTopics(topicInfo);
      
  bool simData = false;
	for (unsigned int i=0; i < topicInfo.size(); i++)
	{ 
		if (topicInfo[i].name == "/atrias_sim_data")
		{
			simData = true;
			break;
		}
	}

  // Start sim_callback because there is robot_state data
  //gazebo_connector_sub = nh.subscribe("atrias_sim_data", 0, &GazeboConnector::sim_callback, this);
  //gazebo_connector_pub = nh.advertise<atrias_msgs::controller_output>("atrias_controller_requests", 0);
  
  // This should return false if Gazebo is not connected.
  return simData;
}

// ROS output
void GazeboConnector::transmitHook() {
  if (rtOps->controllerOutput.command == 2) {
    eStopped = true;
  }
  if (eStopped || !guiEnabled || rtOps->controllerOutput.command != 0) {
    // The controller's commanded a disable here... it expects no current applied, so give
    // it no output torque.
    rtOps->controllerOutput.lLeg.motorCurrentA = 0.0;
    rtOps->controllerOutput.lLeg.motorCurrentB = 0.0;
    rtOps->controllerOutput.rLeg.motorCurrentA = 0.0;
    rtOps->controllerOutput.rLeg.motorCurrentB = 0.0;
  }
  // Stuff the ROS message with controller torques
  cosi = rtOps->controllerOutput;
  // Queue the request
  rtOps->gazeboDataOut.write(cosi);
}

void GazeboConnector::disable() {
	guiEnabled = false;
}

void GazeboConnector::enable() {
	guiEnabled = true;
}

void GazeboConnector::eStop() {
	eStopped = true;
}

void GazeboConnector::leaveEStop() {
	eStopped = false;
}
