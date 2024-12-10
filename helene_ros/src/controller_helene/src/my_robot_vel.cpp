#define _USE_MATH_DEFINES
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_helene/jointPosition.h>
#include <math.h>
#include <sensor_msgs/JointState.h>






class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(const ros::Publisher& pub): pub_(pub)
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("q1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);
    hardware_interface::JointStateHandle state_handle_b("q2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);
    hardware_interface::JointStateHandle state_handle_c("q3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_c);
    hardware_interface::JointStateHandle state_handle_d("q4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_d);
    hardware_interface::JointStateHandle state_handle_e("q5", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_e);
    hardware_interface::JointStateHandle state_handle_f("q6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_f);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("q1"), &cmd[0]);
    jnt_vel_interface.registerHandle(pos_handle_a);
    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("q2"), &cmd[1]);
    jnt_vel_interface.registerHandle(pos_handle_b);
    hardware_interface::JointHandle pos_handle_c(jnt_state_interface.getHandle("q3"), &cmd[2]);
    jnt_vel_interface.registerHandle(pos_handle_c);
    hardware_interface::JointHandle pos_handle_d(jnt_state_interface.getHandle("q4"), &cmd[3]);
    jnt_vel_interface.registerHandle(pos_handle_d);
    hardware_interface::JointHandle pos_handle_e(jnt_state_interface.getHandle("q5"), &cmd[4]);
    jnt_vel_interface.registerHandle(pos_handle_e);
    hardware_interface::JointHandle pos_handle_f(jnt_state_interface.getHandle("q6"), &cmd[5]);
    jnt_vel_interface.registerHandle(pos_handle_f);

    registerInterface(&jnt_vel_interface);

  }

  
  void anglesCallback(const controller_helene::jointPosition& msg){
    angles = msg;
    g_igotdata = true; 
    g_LastValidDatafromRobot = ros::Time::now();
  }
  
  void velocitiesCallback(const controller_helene::jointPosition& msg){
    velocities = msg;
  }
  
  void write()
  {
    velocity_command.joint1 = cmd[0]*(71583/M_PI*2)/40;
    velocity_command.joint2 = cmd[1]*(71583/M_PI*2)/40;
    velocity_command.joint3 = cmd[2]*(71583/M_PI*2)/40;
    velocity_command.joint4 = cmd[3]*(71583/M_PI*2)/40;
    velocity_command.joint5 = cmd[4]*(71583/M_PI*2)/40;
    velocity_command.joint6 = cmd[5]*(71583/M_PI*2)/40;
    pub_.publish(velocity_command);
    //std::cout<< " " <<std::endl;
    //std::cout<< "target_vel[0]: "<< cmd[0] <<std::endl;
    //std::cout<< "target_vel[1]: "<< cmd[1] <<std::endl;
    //std::cout<< "target_vel[2]: "<< cmd[2] <<std::endl;
    //std::cout<< "target_vel[3]: "<< cmd[3] <<std::endl;
    //std::cout<< "target_vel[4]: "<< cmd[4] <<std::endl;
    //std::cout<< "target_vel[5]: "<< cmd[5] <<std::endl;
  }
  
  void read()
  {
    pos[0] = ((double)angles.joint1/16384)*M_PI*2;
    pos[1] = ((double)angles.joint2/16384)*M_PI*2;
    pos[2] = ((double)angles.joint3/16384)*M_PI*2;
    pos[3] = ((double)angles.joint4/16384)*M_PI*2;
    pos[4] = ((double)angles.joint5/16384)*M_PI*2;
    pos[5] = ((double)angles.joint6/16384)*M_PI*2;

    vel[0] = ((double)velocities.joint1/(71583*M_PI*2))*40;
    vel[1] = ((double)velocities.joint2/(71583*M_PI*2))*40;
    vel[2] = ((double)velocities.joint3/(71583*M_PI*2))*40;
    vel[3] = ((double)velocities.joint4/(71583*M_PI*2))*40;
    vel[4] = ((double)velocities.joint5/(71583*M_PI*2))*40;
    vel[5] = ((double)velocities.joint6/(71583*M_PI*2))*40;


    //std::cout<< " " <<std::endl;
    //std::cout<< "actual_angle[0]: "<< pos[0] <<std::endl;
    //std::cout<< "actual_angle[1]: "<< pos[1] <<std::endl;
    //std::cout<< "actual_angle[2]: "<< pos[2] <<std::endl;
    //std::cout<< "actual_angle[3]: "<< pos[3] <<std::endl;
    //std::cout<< "actual_angle[4]: "<< pos[4] <<std::endl;
    //std::cout<< "actual_angle[5]: "<< pos[5] <<std::endl;

    //std::cout<< " " <<std::endl;
    //std::cout<< "actual_velocity[0]: "<< vel[0] <<std::endl;
    //std::cout<< "actual_velocity[1]: "<< vel[1] <<std::endl;
    //std::cout<< "actual_velocity[2]: "<< vel[2] <<std::endl;
    //std::cout<< "actual_velocity[3]: "<< vel[3] <<std::endl;
    //std::cout<< "actual_velocity[4]: "<< vel[4] <<std::endl;
    //std::cout<< "actual_velocity[5]: "<< vel[5] <<std::endl;
  }

  ros::Time getleastTimethatHelenepublished(){
    return g_LastValidDatafromRobot;
  }

  bool wastherevaliddata(){
    return g_igotdata;
  }


private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[6];
  double pos[6] = {0,0,0,0,0,0};
  double prev_pos[6] = {0,0,0,0,0,0};
  double vel[6];
  double eff[6];
  controller_helene::jointPosition velocity_command;
  controller_helene::jointPosition angles;
  controller_helene::jointPosition velocities;
  ros::Publisher pub_;
  ros::Time g_LastValidDatafromRobot; 
  bool g_igotdata = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_robot_hw_vel");
  ros::NodeHandle nh;

  ros::Rate loop_rate(30);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Publisher pub = nh.advertise<controller_helene::jointPosition>("/target_velocity",1000);
  MyRobot robot(pub);
  ros::Subscriber subang = nh.subscribe("/actual_angles",1000, &MyRobot::anglesCallback, &robot);
  ros::Subscriber subvel = nh.subscribe("/actual_velocities",1000, &MyRobot::velocitiesCallback, &robot);
  ros::Time Starttime = ros::Time::now();
  bool l_connection_established = false;
  while(l_connection_established == false){
    ros::Duration(0.1).sleep();
    l_connection_established = robot.wastherevaliddata();
    if(ros::Time::now()-Starttime > ros::Duration(60)){
      ROS_ERROR("Unable to connect to Helene within 60s! Is the robot even connected? Quitting everything.");
      break;
    }
  }
  
  if(l_connection_established == true){
    ROS_INFO("Established connection to Helene, starting the controllers.");
    controller_manager::ControllerManager cm(&robot,nh);
  
    ros::Time ts = ros::Time::now();
    Starttime = ros::Time::now(); //Reset Starttime. Give the Robot a bit time before killig the whole process!
    ROS_INFO("Controllers are started now - Helene is up!");
    while (ros::ok() && (ros::Time::now()-Starttime < ros::Duration(20)  ||  ros::Time::now()-robot.getleastTimethatHelenepublished() < ros::Duration(0.5)))
    {
      //ROS_INFO("loop");
      ros::Duration d = ros::Time::now() - ts;
      ts = ros::Time::now();
      robot.read();
      cm.update(ts, d);
      robot.write();
      loop_rate.sleep();
    }

    if((ros::Time::now()-Starttime < ros::Duration(20)  ||  ros::Time::now()-robot.getleastTimethatHelenepublished() < ros::Duration(0.5)) == false){
      ROS_ERROR("Lost connection to Helene! Quitting.");
    }
  }

  return 0;
}

