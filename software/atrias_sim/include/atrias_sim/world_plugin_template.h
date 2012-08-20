#ifndef __WORLD_PLUGIN_TEMPLATE_H__
#define __WORLD_PLUGIN_TEMPLATE_H__

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
		public:
			// Constructor
			ModelPush();

			// Functions
			virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
			void OnUpdate();

		private:
			// Pointer to the update event connection
			event::ConnectionPtr updateConnection;

			physics::WorldPtr world;
			physics::ModelPtr model;
			physics::LinkPtr link;
			//physics::JointPtr joint;

			math::Vector3 forceVec;

			double force;

	};
}

#endif
