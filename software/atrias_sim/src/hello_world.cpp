#include "common/Plugin.hh"

namespace gazebo
{
	class HelloWorld : public WorldPlugin
	{
		public: HelloWorld() : WorldPlugin() 
		{
			printf("Hello World!\n");
		}

		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
		{
		};

	};
	GZ_REGISTER_WORLD_PLUGIN(HelloWorld)
} 
