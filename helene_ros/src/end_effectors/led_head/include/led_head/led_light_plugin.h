#ifndef LED_LIGHT_PLUGIN_HH
#define LED_LIGHT_PLUGIN_HH

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


#include <ros/ros.h>
#include <string>

#include <std_msgs/UInt8.h>


namespace gazebo
{
  class LEDLightPlugin: public gazebo::ModelPlugin
  {
      /// \brief Constructor
    public: LEDLightPlugin();

    /// \brief Destructor
    public: ~LEDLightPlugin();

    /// \brief Load the plugin
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    // protected: virtual void UpdateChild();

    private: void OnGreenRosMsg(const std_msgs::UInt8::ConstPtr &msg);
    private: void OnBlueRosMsg(const std_msgs::UInt8::ConstPtr &msg);

    private: bool SetLightColor(physics::LightPtr lightPtr, std::vector<double> color);

    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;
    private: sdf::ElementPtr sdf_;

    /// \brief The parent Model
    private: physics::LinkPtr link_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Subscriber greenSubscriber_;
    private: ros::Subscriber blueSubscriber_;

    /// \brief ROS texture topic name
    private: std::string green_topic_name_ = "/helene_led_green";
    private: std::string blue_topic_name_ = "/helene_led_blue";

    // Pointer to the light in the world
    private: physics::LightPtr greenLightPtr;
    private: physics::LightPtr blueLightPtr;

    // Name of the light
    private: std::string greenLightName;
    private: std::string blueLightName;

    /// \brief For setting ROS name space
    private: std::string robot_namespace_;

    // Custom Callback Queue
    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    private: event::ConnectionPtr add_model_event_;
  };
}
#endif // LED_LIGHT_PLUGIN_HH