#ifndef NEEDLE_SENSOR_PLUGIN_HH
#define NEEDLE_SENSOR_PLUGIN_HH

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <string>


namespace gazebo
{
  class NeedleSensorPlugin: public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: NeedleSensorPlugin();

    /// \brief Destructor
    public: ~NeedleSensorPlugin();

    /// \brief Load the plugin
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // \brief Update the controller
    protected: virtual void UpdateChild();


    /// \brief gazebo world update connection.
    private: gazebo::event::ConnectionPtr updateConnection;

    public: void test();

    /// \brief ROS NodeHandle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS publisher queue for joint states.
    private: ros::Publisher _sensorPublisher;

    /// \brief World pointer.
    private: gazebo::physics::WorldPtr world;

    /// \brief Parent model of the needle.
    private: gazebo::physics::ModelPtr model;

    /// \brief Pointer to the SDF of this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Vector containing all the joint names.
    private: std::vector<std::string> jointNames;

    /// \brief name of the link containing the tip
    private: std::string linkName = "axis_5";

    /// \brief name of the model containing the tumor
    private: std::string specimenName = "cylinder";

    /// \brief gazebo link for the tumor
    private: physics::LinkPtr tumorLink;

    /// \brief radius of tumor in meters
    private: double tumorRadius = 0.05;

    /// \brief timestamp of last log
    private: int logCounter = 0;
  };
}

#endif // NEEDLE_SENSOR_PLUGIN_HH