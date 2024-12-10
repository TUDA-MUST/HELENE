#!/usr/bin/env python3

# Python 2/3 compatibility imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import quaternion_multiply

globalVelocityScaling = 0.1

try:
    from math import pi, tau, dist, fabs, cos, sin, exp, pow
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt, sin, exp, pow

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class helene_helper:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("helene_helper", anonymous=True)
        #Initialise all required objects
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("helene_arm")

        #Set initial speed and acceleration 
        self.move_group.set_max_velocity_scaling_factor(0.8)
        self.move_group.set_max_acceleration_scaling_factor(1)

        #Define Publisher to display trajectory
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20
        )
        #Define Publisher to reserved
        self.pub_reserved = rospy.Publisher(
            "/helene_reserved",
            std_msgs.msg.UInt8,
            queue_size=10
        )
        #Define Publisher to helene_led_blue
        self.led_blue = rospy.Publisher(
            "/helene_led_blue",
            std_msgs.msg.UInt8,
            queue_size=10
        )
        #Define Publisher to helene_led_green
        self.led_green = rospy.Publisher(
            "/helene_led_green",
            std_msgs.msg.UInt8,
            queue_size=10
        )
        #Define XYZ_Position message
        self.pose_goal = geometry_msgs.msg.Pose()

        # Set constraints for cartesian planning to None
        self.cartesian_constraints = None


    #Set speed
    def set_speed_scaler(self, speed):
        global globalVelocityScaling
        globalVelocityScaling = speed
        self.move_group.set_max_velocity_scaling_factor(speed)

    #Get Speed
    def get_speed_scaler(self):
        global globalVelocityScaling
        return globalVelocityScaling
    #Set acceleration
    def set_acc_scaler(self, accel):
        self.move_group.set_max_acceleration_scaling_factor(accel)
    #Set reserved Uint8
    def set_reserved(self, reserved):
        self.pub_reserved.publish(reserved)
    #Set reserved Uint8
    def set_led_blue(self, led_blue):
        self.led_blue.publish(led_blue)
    #Set reserved Uint8
    def set_led_green(self, led_green):
        self.led_green.publish(led_green)

    #send new position
    def __position_go__(self):
        self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def __get_last_goal_quaterion__(self):
        #current_pose = self.move_group.get_current_pose().pose
        #return [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        return [self.pose_goal.orientation.x, self.pose_goal.orientation.y,  self.pose_goal.orientation.z,  self.pose_goal.orientation.w]

    #Set absolute position
    def move_ptp_abs_pos(self, x, y, z, a_r, a_p, a_y):
        self.pose_goal.position.x = x
        self.pose_goal.position.y = y
        self.pose_goal.position.z = z
        self.pose_goal.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(a_r, a_p, a_y))
        self.move_group.set_pose_target(self.pose_goal)
        self.__position_go__()

    #Set relative position
    #FIX THIS - Not sure with Quaternion
    def move_ptp_rel_pos(self, x, y, z, ar_r, ar_p, ar_y):
        self.pose_goal = self.move_group.get_current_pose().pose
        self.pose_goal.position.x += x
        self.pose_goal.position.y += y
        self.pose_goal.position.z += z
        self.pose_goal.orientation = geometry_msgs.msg.Quaternion(*(quaternion_multiply(quaternion_from_euler(ar_r, ar_p, ar_y), self.__get_last_goal_quaterion__())))
        self.move_group.set_pose_target(self.pose_goal)
        self.__position_go__()

    #Set relative position and drive cartesian to it
    #FIX THIS - Not sure with Quaternion
    def move_lin_rel_pos(self, x, y, z, ar_r, ar_p, ar_y):
        self.pose_goal = self.move_group.get_current_pose().pose
        self.pose_goal.position.x += x
        self.pose_goal.position.y += y
        self.pose_goal.position.z += z
        self.pose_goal.orientation = geometry_msgs.msg.Quaternion(*(quaternion_multiply(quaternion_from_euler(ar_r, ar_p, ar_y), self.__get_last_goal_quaterion__())))
        #self.move_group.set_pose_target(self.pose_goal)
        waypoints = []
        waypoints.append(copy.deepcopy(self.pose_goal)) #Need to deepcopy that!     
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0, path_constraints = self.cartesian_constraints)  # waypoints to follow  # eef_step  # jump_threshold
        plan = self.move_group.retime_trajectory(self.move_group.get_current_state(),plan,globalVelocityScaling)
        if fraction < 0.2:
            print("Trajectory solution seems bad. Are you trying to drive through a singularity? I WONT drive Cartesian to the destination.")
            self.move_group.set_pose_target(self.pose_goal)
            self.__position_go__()
        else:
            #Show
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            # Publish
            self.display_trajectory_publisher.publish(display_trajectory)
            #Execute
            self.move_group.execute(plan, wait=True)


    #Set cartesian pos
    def move_lin_abs_pos(self, raw_waypoint):
        waypoints = []
        current_pose = self.move_group.get_current_pose().pose
        current_or = self.__get_last_goal_quaterion__()
        curpoint = [current_pose.position.x,current_pose.position.y,current_pose.position.z, current_or]
        waypoints.append(curpoint)
        self.pose_goal.position.x = raw_waypoint[0]
        self.pose_goal.position.y = raw_waypoint[1]
        self.pose_goal.position.z = raw_waypoint[2]
        self.pose_goal.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(raw_waypoint[3], raw_waypoint[4], raw_waypoint[5]))
        waypoints.append(copy.deepcopy(self.pose_goal)) #Need to deepcopy that!            

        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0, path_constraints = self.cartesian_constraints)  # waypoints to follow  # eef_step  # jump_threshold
        plan = self.move_group.retime_trajectory(self.move_group.get_current_state(),plan,globalVelocityScaling)
        #Show
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
        #Execute
        self.move_group.execute(plan, wait=True)

    #Set cartesian path
    def move_lin_path(self, raw_waypoints):
        waypoints = []
        for i in raw_waypoints:
            self.pose_goal.position.x = i[0]
            self.pose_goal.position.y = i[1]
            self.pose_goal.position.z = i[2]
            self.pose_goal.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(i[3], i[4], i[5]))
            waypoints.append(copy.deepcopy(self.pose_goal)) #Need to deepcopy that!            

        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0, path_constraints = self.cartesian_constraints)  # waypoints to follow  # eef_step  # jump_threshold
        plan = self.move_group.retime_trajectory(self.move_group.get_current_state(),plan,globalVelocityScaling)
        #Show
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
        #Execute
        self.move_group.execute(plan, wait=True)

    #Return the position from the last goal
    def get_last_goal_pos(self):
        current_pose = self.pose_goal
        current_or = self.__get_last_goal_quaterion__()
        curpoint = [current_pose.position.x,current_pose.position.y,current_pose.position.z, *euler_from_quaternion(current_or)]
        return curpoint

    #Return the current actual position. Orientation is given as quaternion! This function currently is somehow a bit fucked. 
    def get_actual_pos(self):
        current_pose = self.move_group.get_current_pose().pose
        current_or = [current_pose.orientation.x, current_pose.orientation.y,  current_pose.orientation.z,  current_pose.orientation.w]
        curpoint = [current_pose.position.x,current_pose.position.y,current_pose.position.z, (current_or)]
        return curpoint

    #Set home position
    #FIXED FIX THIS - Maybe add correct home position which is currently named straight in urdf
    def move_ptp_home_pos(self):
        self.move_group.set_named_target("home")
        self.__position_go__()

    #Set straight position
    #FIXED FIX THIS - Fix straight position in urdf
    def move_ptp_straight_pos(self):
        self.move_group.set_named_target("straight")
        self.__position_go__()