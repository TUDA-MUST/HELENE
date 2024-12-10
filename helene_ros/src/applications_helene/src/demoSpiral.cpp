#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Initialising and defining the planning group for move_base
    static const std::string PLANNING_GROUP = "helene_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    tf2::Quaternion orientation;
    // Defining target pose
    geometry_msgs::Pose target_pose;
    orientation.setRPY(0, M_PI/2, 0);
    target_pose.position.x = 0.2;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.1;
    target_pose.orientation = tf2::toMsg(orientation);
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g",target_pose.position.x, target_pose.position.y, target_pose.position.z);    

    // Initialising a new plan and planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setMaxVelocityScalingFactor(1);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Executing the movement
    move_group.move();

    //2. Create Cartesian Paths
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose ee_point_goal; //end_effector_trajectory
    ee_point_goal.orientation = target_pose.orientation;
    ee_point_goal.position.z = 0.1;
        // Trajectory parameters (circle)
    double angle_resolution = 5;
    double d_angle = angle_resolution*3.14/180;
    double angle= 0;
    double radius = 0.05;
    double rotAngleX = 0;
    double rotAngleY = 0;
    //----------------------------------------------------------------------
    //3. Plan for the trajectory
    for (int i= 0; i< 5*(360/angle_resolution); i++)
    {
        //discretize the trajectory
        angle+= d_angle;
        ee_point_goal.position.x = target_pose.position.x + (radius)*cos(angle);
        ee_point_goal.position.y = target_pose.position.y + (radius)*sin(angle);
        rotAngleX = -M_PI/12 * cos(angle);
        rotAngleY = M_PI/12 * sin(angle);
        orientation.setRPY(rotAngleX, M_PI/2+rotAngleY, 0);
        ee_point_goal.orientation = tf2::toMsg(orientation);
        waypoints.push_back(ee_point_goal);
        //ROS_INFO("%d",i);
    }

    // We want cartesian path to be interpolated at a eef_resolution
    double eef_resolution = std::min(0.01,radius*angle_resolution);
    double jump_threshold = 0.0; //disable the jump threshold =0
    moveit_msgs::RobotTrajectory trajectory;
    double fraction =move_group.computeCartesianPath(waypoints, eef_resolution,jump_threshold, trajectory);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",  fraction * 100.0);
    move_group.execute(trajectory);
    ros::shutdown();
    return 0;
}