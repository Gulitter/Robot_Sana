#include <gazebo/gazebo.hh>

namespace gazebo
{
    class WorldPluginTutorial : public WorldPlugin
    {
    public: WorldPluginTutorial() : WorldPlugin()
    {
        printf("\n\n\n\n\n\nHello World!\n\n\n\n\n\n");
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
    }
    };
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}