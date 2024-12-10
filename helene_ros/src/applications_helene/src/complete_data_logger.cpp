#define _USE_MATH_DEFINES
#include <math.h>
#include <ros/ros.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <time.h>
#include <sys/time.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <control_msgs/JointTrajectoryControllerState.h>

#include <tf2/LinearMath/Quaternion.h>


#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

control_msgs::JointTrajectoryControllerState controller_msg;
std_msgs::Float32 force_msg;
std_msgs::UInt8 reserved_msg;
ros::Duration period;
ros::Time prev_time;
control_msgs::FollowJointTrajectoryActionGoal trajectory;


void controllerCallback(const control_msgs::JointTrajectoryControllerState& msg){
    controller_msg = msg;
}

void forceCallback(const std_msgs::Float32& msg){
    force_msg = msg;
}

void reservedCallback(const std_msgs::UInt8& msg){
    reserved_msg = msg;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_logger");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::Rate loop_rate(50);

    std::string robot_desc_string;
    std::string root_name;
    std::string tip_name;
    nh.param("/helene/robot_base", root_name, std::string());
    nh.param("/helene/robot_endeffector", tip_name, std::string());
    nh.param("/robot_description", robot_desc_string, std::string());
    
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain_;

    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
        ROS_ERROR("Failed to construct kdl tree from description: %s", robot_desc_string.c_str());
        std::cout << "root name, tip name" << root_name << "   " << tip_name << std::endl;
        std::cout << "Namespace: " << nh.getNamespace() << std::endl;
    }
    else{
    }

    if (!kdl_tree.getChain(root_name, tip_name, kdl_chain_)){
        ROS_ERROR("Failed to construct kdl chain with root: %s and tip: %s", root_name.c_str(), tip_name.c_str());
    }
    else{
    }

    unsigned int numberOfJoints = kdl_chain_.getNrOfJoints();

    KDL::ChainFkSolverPos_recursive fkSolverPos(kdl_chain_);

    KDL::JntArray q_in(numberOfJoints);
    KDL::Frame p_eef;
    for (int i = 0; i < numberOfJoints; i++) {
        q_in(i, 0) = 0.0;
    }
    int fk_feedback = fkSolverPos.JntToCart(q_in, p_eef);

    ROS_INFO("Start logging all data");
    

    prev_time = ros::Time::now();
    ros::Subscriber controller_sub = nh.subscribe("/helene/VelocityJointInterface_trajectory_controller/state",1, controllerCallback);
    ros::Subscriber force_sub = nh.subscribe("/raw_meas",1, forceCallback);
    ros::Subscriber reserved_sub = nh.subscribe("/helene_reserved",1, reservedCallback);
    
   
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
    auto str = oss.str();
    std::string filename  = "data_log"+str+".csv";
    std::ofstream myfile;
    myfile.open(filename);

    ros::Duration(2).sleep();

    double q_x, q_y, q_z, q_w;

    struct timeval tv;

    myfile << "x"<<","<< "y"<<","<< "z"<<","<<"q_x"<<"," << "q_y"<<","<< "q_z"<<","<< "q_w"<<","<<"force"<< ","<< "res" << "," << "actual_position_1" << ","<< "actual_position_2" << ","<< "actual_position_3" << ","<< "actual_position_4" << ","<< "actual_position_5" << ","<< "actual_position_6" << ","<< "desired_positon_1" << ","<< "desired_position_2" << ","<< "desired_position_3" << ","<< "desired_position_4" << ","<< "desired_position_5" << ","<< "desired_position_6" << ","<< "actual_velocity_1" << ","<< "actual_velocity_2" << ","<< "actual_velocity_3" << ","<< "actual_velocity_4" << ","<< "actual_velocity_5" << ","<< "actual_velocity_6" << ","<< "desired_velocity_1" << ","<< "desired_velocity_2"<< ","<< "desired_velocity_3" << ","<< "desired_velocity_4" << ","<< "desired_velocity_5" << ","<< "desired_velocity_6" << "," << "time" << ","<< "unix_time" << std::endl; 

    while (ros::ok()){
        const ros::Time time = ros::Time::now();
        period = time - prev_time;

        q_in(0, 0) = controller_msg.actual.positions[0];
        q_in(1, 0) = controller_msg.actual.positions[1];
        q_in(2, 0) = controller_msg.actual.positions[2];
        q_in(3, 0) = controller_msg.actual.positions[3];
        q_in(4, 0) = controller_msg.actual.positions[4];
        q_in(5, 0) = controller_msg.actual.positions[5];
        int fk_feedback = fkSolverPos.JntToCart(q_in, p_eef);
        if (fk_feedback < 0) {
            ROS_WARN("Problem solving forward kinematics. Error: %s", fkSolverPos.strError(fk_feedback));
        }
        p_eef.M.GetQuaternion(q_x,q_y,q_z,q_w);

		gettimeofday(&tv, NULL);
		unsigned long long unix_time = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;

        myfile << p_eef.p[0] <<","<< p_eef.p[1] <<","<< p_eef.p[2]<<","<<q_x<< "," <<q_y<<","<< q_z<<","<< q_w<<","<<force_msg.data<< ","<< float(reserved_msg.data) << "," << controller_msg.actual.positions[0] << ","<< controller_msg.actual.positions[1] << ","<< controller_msg.actual.positions[2] << ","<< controller_msg.actual.positions[3] << ","<< controller_msg.actual.positions[4] << ","<< controller_msg.actual.positions[5] << ","<< controller_msg.desired.positions[0] << ","<< controller_msg.desired.positions[1] << ","<< controller_msg.desired.positions[2] << ","<< controller_msg.desired.positions[3] << ","<< controller_msg.desired.positions[4] << ","<< controller_msg.desired.positions[5] << ","<< controller_msg.actual.velocities[0] << ","<< controller_msg.actual.velocities[1] << ","<< controller_msg.actual.velocities[2] << ","<< controller_msg.actual.velocities[3] << ","<< controller_msg.actual.velocities[4] << ","<< controller_msg.actual.velocities[5] << ","<< controller_msg.desired.velocities[0] << ","<< controller_msg.desired.velocities[1] << ","<< controller_msg.desired.velocities[2] << ","<< controller_msg.desired.velocities[3] << ","<< controller_msg.desired.velocities[4] << ","<< controller_msg.desired.velocities[5] << "," << period << "," << unix_time << std::endl; 
 
        loop_rate.sleep();
    }

    myfile.close();
    return 0;
}
