// Author: Andrew Peekema

#ifndef __ATRIAS21_BIPED_H__
#define __ATRIAS21_BIPED_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo
{
	class ModelPush : public WorldPlugin
	{
		// Constructor/Destructor
		public: ModelPush();
		public: ~ModelPush();

		// Functions
		public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate();

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		private: physics::WorldPtr world;
		private: physics::ModelPtr model;
		private: physics::LinkPtr link;
		//private: physics::JointPtr joint;

		private: math::Vector3 forceVec;

		private: double force;

	};
}

#endif
