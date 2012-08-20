// Author: Andrew Peekema

#ifndef __ATRIAS20_LEG_H__
#define __ATRIAS20_LEG_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <ros/ros.h>

#include <atrias_msgs/robot_state.h>  // controller input
#include <atrias_msgs/controller_output.h>

namespace gazebo
{
	class ControllerWrapper : public WorldPlugin
	{
		public:
			// Constructor/Destructor
			ControllerWrapper();
			~ControllerWrapper();

			// Functions
			virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
			void OnUpdate();
			void atrias_controller_callback(const atrias_msgs::controller_output &temp_cosi);
			double wrap_angle(double newTheta);

		private: 
			// Pointer to the update event connection
			event::ConnectionPtr updateConnection;

			physics::WorldPtr world;

			physics::ModelPtr model;
			std::string modelName;

			physics::LinkPtr body;
			std::string bodyName;
			physics::LinkPtr motorA;
			std::string motorAName;
			physics::LinkPtr motorB;
			std::string motorBName;
			physics::LinkPtr legA;
			std::string legAName;
			physics::LinkPtr legB;
			std::string legBName;
			physics::LinkPtr toe;
			std::string toeName;

			double angle, theta, toePosZ, PI, gearRatio, legTorqueConstant, hipTorqueConstant;
			math::Vector3 axis, bodyForce;
			common::Time simTime;

			boost::mutex lock;

			ros::Subscriber atrias_sim_sub;
			ros::Publisher atrias_sim_pub;
			atrias_msgs::robot_state ciso;  // Controller in, simulation out
			atrias_msgs::controller_output cosi;  // Controller out, simulation in

	};
}

#endif
