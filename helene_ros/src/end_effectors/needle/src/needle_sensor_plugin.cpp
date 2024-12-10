#include <needle/needle_sensor_plugin.h>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <string>
#include <chrono>

namespace gazebo
{
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(NeedleSensorPlugin);

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    NeedleSensorPlugin::NeedleSensorPlugin()
    {
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    NeedleSensorPlugin::~NeedleSensorPlugin()
    {
        this->rosNode->shutdown();
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the controller
    void NeedleSensorPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
        // Make sure the ROS node for Gazebo has already been initalized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("NeedleSensorPlugin", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        this->model = _parent;
        this->world = this->model->GetWorld();
        this->sdf = _sdf;

        physics::ModelPtr cube = this->world->ModelByName(this->specimenName);
        if(!cube)
        {
            ROS_FATAL_STREAM_NAMED("NeedleSensorPlugin", "Unable to find link with name tumor in model " << this->specimenName
            << " Abborting startup of plugin.");
            return;
        }
        this->tumorLink = cube->GetLink("tumor");
        if(!this->tumorLink)
        {
            ROS_FATAL_STREAM_NAMED("NeedleSensorPlugin", "Unable to find link with name tumor in model " << this->specimenName
            << " Abborting startup of plugin.");
            return;
        }
        this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&NeedleSensorPlugin::UpdateChild, this));
        ROS_INFO_STREAM_NAMED("NeedleSensorPlugin", "Successfully started needle sensor plugin for gazebo" << std::endl);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Update the controller
    void NeedleSensorPlugin::UpdateChild()
    {
        if (this->logCounter == 200)
        {
            this->logCounter = 0;
            physics::LinkPtr tipLink = this->model->GetLink(this->linkName);
            if (!tipLink)
            {
                ROS_ERROR_STREAM_NAMED("NeedleSensorPlugin", "Unable to find robot link with name: \"" <<
                        this->linkName << "\"" << std::endl);
                return;
            }
            ignition::math::Pose3d linkPose = tipLink->WorldCoGPose();
            ignition::math::Pose3d tipOffset;
            tipOffset.Set(-0.198, 0, 0.014, 0, 0, 0);
            ignition::math::Pose3d tipPose = linkPose * tipOffset;

            double distanceToTumor = tipPose.Pos().Distance(this->tumorLink->WorldCoGPose().Pos());
            if (distanceToTumor < tumorRadius)
            {
                ROS_INFO_STREAM_NAMED("NeedleSensorPlugin", "You have found the tumor!" << std::endl);
            }
            // ROS_INFO_STREAM_NAMED("NeedleSensorPlugin", "Distance to tumor: \"" << "test" << "\"" << std::endl);
            // ROS_INFO_STREAM_NAMED("NeedleSensorPlugin", "Distance to tumor: \"" << distanceToTumor << "\"" << std::endl);
        }
        this->logCounter++;
    }
}