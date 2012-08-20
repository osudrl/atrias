// Author: Andrew Peekema

#ifndef __MODEL_PLUGIN_TEMPLATE_H__
#define __MODEL_PLUGIN_TEMPLATE_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
		// Constructor
		public: ModelPush();

		// Functions
		public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate();

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		private: physics::ModelPtr model;
		private: physics::LinkPtr link;
		//private: physics::JointPtr joint;

		private: math::Vector3 forceVec;

		private: double count;
		private: double force;

	};
}

#endif
