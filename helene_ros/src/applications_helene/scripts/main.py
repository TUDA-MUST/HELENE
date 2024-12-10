#Run this with roslaunch applications_helene helpy.launch

#Import module and create object
from helene_helper import *

planer = helene_helper()

#Set speed and acceleration
planer.set_speed_scaler(0.2)
planer.set_acc_scaler(1)

#Go home
planer.move_ptp_home_pos()
planer.move_ptp_rel_pos(0,0,0,*(0,0,0))
print(planer.get_last_goal_pos())
planer.set_led_blue(255)

rospy.sleep(1) #Wait so the motion is finished
#Move relative ptp
planer.move_ptp_rel_pos(0.1,0.1,0,*(0,0,0))
print(planer.get_last_goal_pos())
#Move absolute ptp
planer.move_ptp_abs_pos(0.2,-0.2,0.14, *(pi, -pi/2, 0))
planer.set_led_green(255)
#Plan and move cartesian path
temp_waypoints = [[0.3,-0.2,0.14, *(pi, -pi/2, 0)],
                [0.3,-0.2,0.24, *(pi, -pi/2, 0)],
                [0.2,-0.2,0.24, *(pi, -pi/2, 0)]]
planer.move_lin_path(temp_waypoints)
rospy.sleep(1) #Wait so the motion is finished
print(planer.get_last_goal_pos())

#Send message to reserved
#planer.set_reserved(1)

#Go straight up
planer.move_ptp_home_pos()

