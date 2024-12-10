#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <ros/ros.h>
#include <string>

#include <led_head/led_light_plugin.h>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

#include <std_msgs/UInt8.h>

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(LEDLightPlugin);

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    LEDLightPlugin::LEDLightPlugin()
    {
        this->rosnode_ = NULL;
    }


    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    LEDLightPlugin::~LEDLightPlugin()
    {
        // Custom Callback Queue
        this->queue_.clear();
        this->queue_.disable();
        this->rosnode_->shutdown();
        this->callback_queue_thread_.join();

        delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the controller
    void LEDLightPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
        // Get pointer for parent (robot model)
        this->model_ = _parent;

        // Get pointer for world
        this->world_ = _parent->GetWorld();

        // Get pointer to element
        this->sdf_ = _sdf;

        // load parameters
        this->robot_namespace_ = "";
        if (_sdf->HasElement("robotNamespace"))
            this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("projector", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        // set names of lights
        this->greenLightName = "green_light";
        this->blueLightName = "blue_light";

        // Get pointers for specified lights
        this->greenLightPtr = this->world_->LightByName(this->greenLightName);
        this->blueLightPtr = this->world_->LightByName(this->blueLightName);

        // Check if lights exist
        if (this->greenLightPtr == NULL || this->blueLightPtr == NULL)
        {
            ROS_FATAL_STREAM_NAMED("projector", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        }
        else
        {
            // if lights exist, turn them off
            std::vector<double> off = {0, 0, 0};
            SetLightColor(this->greenLightPtr, off);
            SetLightColor(this->blueLightPtr, off);
        }

        // Create ROS node for subscriber
        this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);


        // Create Subscribers with callback functions
        ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::UInt8>(
            this->green_topic_name_,1,
            boost::bind( &LEDLightPlugin::OnGreenRosMsg,this,_1),
            ros::VoidPtr(), &this->queue_);
        this->greenSubscriber_ = this->rosnode_->subscribe(so);

        ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::UInt8>(
            this->blue_topic_name_,1,
            boost::bind( &LEDLightPlugin::OnBlueRosMsg,this,_1),
            ros::VoidPtr(), &this->queue_);
        this->blueSubscriber_ = this->rosnode_->subscribe(so2);


        // Custom Callback Queue
        this->callback_queue_thread_ = boost::thread( boost::bind( &LEDLightPlugin::QueueThread,this ) );

        ROS_INFO_STREAM_NAMED("light_plugin", "Successfully started LED light plugin for gazebo" << std::endl);
    }


    bool LEDLightPlugin::SetLightColor(physics::LightPtr lightPtr, std::vector<double> color)
    {
        // Return if light pointer is NULL
        if (!lightPtr)
        {
            return false;
        }

        // Copy the light's data into a light message
        msgs::Light lightMsg;
        lightPtr->FillMsg(lightMsg);

        // Create color messages
        msgs::Color* diffuseColorMsg = new msgs::Color();
        msgs::Color* specularColorMsg = new msgs::Color();

        // Copy color into color messages
        if (color.size() == 3 || color.size() == 4)
        {
            diffuseColorMsg->set_r(color[0]);
            diffuseColorMsg->set_g(color[1]);
            diffuseColorMsg->set_b(color[2]);
            diffuseColorMsg->set_a(1);
            if(color.size() == 4)
            {
                diffuseColorMsg->set_a(color[3]);
            }

            specularColorMsg->set_r(color[0]);
            specularColorMsg->set_g(color[1]);
            specularColorMsg->set_b(color[2]);
            specularColorMsg->set_a(1);
            if(color.size() == 4)
            {
                specularColorMsg->set_a(color[3]);
            }
        }
        else
        {
            // Return if color does not have the right size
            return false;
        }

        // Apply new colors to light message
        lightMsg.release_diffuse();
        lightMsg.release_specular();
        lightMsg.set_allocated_diffuse(diffuseColorMsg);
        lightMsg.set_allocated_specular(specularColorMsg);

        // Create transport node to send light message to gazebo
        transport::NodePtr node(new transport::Node());
        node->Init(this->world_->Name());

        // Publish light message to gazebo
        transport::PublisherPtr lightPub = node->Advertise<msgs::Light>("~/light/modify");
        lightPub->Publish(lightMsg);
        return true;
    }


    ////////////////////////////////////////////////////////////////////////////////
    // Callback for green light ROS message
    void LEDLightPlugin::OnGreenRosMsg(const std_msgs::UInt8::ConstPtr& msg)
    {
        // Read brightness from message
        uint brightness = msg->data;
        ROS_INFO_STREAM_NAMED("light_plugin", "Received green light message with brightness: " << brightness);

        // Create color vector
        std::vector<double> color = {0, (double) brightness / 255, 0};

        // Set color
        if (!SetLightColor(this->greenLightPtr, color))
        {
            ROS_ERROR_STREAM_NAMED("light_plugin", "Failed to set brightness of green light.");
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Callback for blue light ROS message
    void LEDLightPlugin::OnBlueRosMsg(const std_msgs::UInt8::ConstPtr& msg)
    {
        // Read brightness from message
        uint brightness = msg->data;
        ROS_INFO_STREAM_NAMED("light_plugin", "Received blue light message with brightness: " << brightness);

        // Create color vector
        std::vector<double> color = {0, 0, (double) brightness / 255};

        // Set color
        if (!SetLightColor(this->blueLightPtr, color))
        {
            ROS_ERROR_STREAM_NAMED("light_plugin", "Failed to set brightness of blue light.");
        }
    }


    ////////////////////////////////////////////////////////////////////////////////
    // Custom callback queue thread
    void LEDLightPlugin::QueueThread()
    {
        static const double timeout = 0.01;

        while (this->rosnode_->ok())
        {
            this->queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

}
