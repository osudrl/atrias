// Author: Andrew Peekema

#ifndef __GROUND_CONTACT_H__
#define __GROUND_CONTACT_H__

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
		// Constructor
		public: ModelPush();

		// Functions
		public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate();

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		private: physics::WorldPtr world;
		private: physics::ModelPtr model_1;
		private: physics::ModelPtr model_2;
		private: physics::ModelPtr model_3;
		private: physics::LinkPtr link_1;
		private: physics::LinkPtr link_2;
		private: physics::LinkPtr link_3;
		//private: physics::JointPtr joint;

		private: math::Vector3 forceVec_1;
		private: math::Vector3 forceVec_2;
		private: math::Vector3 forceVec_3;

		private: double force_1;
		private: double force_2;
		private: double force_3;

	};
}

#endif
