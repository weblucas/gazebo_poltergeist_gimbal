#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("gazebo_poltergeist_gimbal A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Hello World! gazebo_poltergeist_gimbal");
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
  }

};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
