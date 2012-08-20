#ifndef __PLANAR_CONSTRAINT_H__
#define __PLANAR_CONSTRAINT_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo
{
	class PlanarConstraint : public WorldPlugin
	{
		// Constructor
		public: PlanarConstraint();

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
		private: math::Vector3 force, torque;

		private: double KP, KD;

	};
}

#endif
