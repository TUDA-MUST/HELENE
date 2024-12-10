#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/UInt8.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher gripper_pub = node_handle.advertise<std_msgs::UInt8>("helene_reserved", 1000);
    // Initialising and defining the planning group for move_base
    static const std::string PLANNING_GROUP = "helene_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    tf2::Quaternion orientation;
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory_slow;
    trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
    robot_trajectory::RobotTrajectory r_trajec(move_group.getRobotModel(), PLANNING_GROUP);
    // Defining target pose
    geometry_msgs::Pose target_pose;
    orientation.setRPY(M_PI, -M_PI/2, 0);
    target_pose.position.x = 0.2;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.14;
    target_pose.orientation = tf2::toMsg(orientation);
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g",target_pose.position.x, target_pose.position.y, target_pose.position.z);    

    // Initialising a new plan and planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setMaxVelocityScalingFactor(1);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Executing the movement
    move_group.move();
    ros::Duration(1.0).sleep();
    std_msgs::UInt8 gripper;
    gripper.data = 0;
    gripper_pub.publish(gripper);
    

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose ee_point_goal; //end_effector_trajectory
    ee_point_goal = target_pose;
    ee_point_goal.position.x += 0.1;
    waypoints.push_back(ee_point_goal);
    ee_point_goal.position.z += 0.1;
    waypoints.push_back(ee_point_goal);
    ee_point_goal.position.x -= 0.1;
    waypoints.push_back(ee_point_goal);
    ee_point_goal.position.z -= 0.1;
    waypoints.push_back(ee_point_goal);
    double eef_resolution = 0.01;
    double jump_threshold = 0.0; //disable the jump threshold =0
    double fraction =move_group.computeCartesianPath(waypoints, eef_resolution,jump_threshold, trajectory);
    
    r_trajec.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    iptp.computeTimeStamps(r_trajec, 1, 1);
    r_trajec.getRobotTrajectoryMsg(trajectory);
    iptp.computeTimeStamps(r_trajec, 0.03, 0.03);
    r_trajec.getRobotTrajectoryMsg(trajectory_slow);
    move_group.execute(trajectory_slow);

    fraction =move_group.computeCartesianPath(waypoints, eef_resolution,jump_threshold, trajectory);
    
    r_trajec.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    iptp.computeTimeStamps(r_trajec, 1, 1);
    r_trajec.getRobotTrajectoryMsg(trajectory);
    iptp.computeTimeStamps(r_trajec, 1, 1);
    r_trajec.getRobotTrajectoryMsg(trajectory_slow);
    move_group.execute(trajectory_slow);

    ros::shutdown();
    return 0;
}