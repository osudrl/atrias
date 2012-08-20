// Author: Andrew Peekema

#ifndef __FREEZE_POSE_H__
#define __FREEZE_POSE_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo
{
	class FreezePose : public WorldPlugin
	{
		// Constructor
		public: FreezePose();

		// Functions
		public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate();

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		private: physics::WorldPtr world;
		private: physics::ModelPtr model;
		private: std::string modelName;
		private: physics::LinkPtr link;
		private: std::string linkName;

		private: math::Pose desired_pose;
		private: math::Vector3 force;

		private: double KP, KD;

	};
}

#endif
