// main_control.cpp
// 161130
// this is the control code for UAV in the simulated enviornment by gazebo.
//

#include <std_msgs/Bool.h>
#include <hector_uav_msgs/EnableMotors.h>
#include "reactive/layers.hpp"

// ============================================================================================
// Main_Control class
// it contains everything necessary for control
// ============================================================================================
class Main_Control : public LAYER_BASE
{
public:
  // the instance of this class must be single
  // so this function must be called to create the object.
  static Main_Control *create_control(const bool _enable);

  // to release the memory, call this function.
  static void kill_control();

  // to receive a message to start operation.
  void OnMessage(const ros::MessageEvent<std_msgs::Bool const>& event);

protected:
  static ros::Publisher vel_pub;

  static ros::Subscriber enable_sub;

private:
  Main_Control(const bool _enable);

  virtual void command();

  static void quit(int);

  static Main_Control *p_control;

  bool enable;
};

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "main_control");

  // enable the motors
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  client.waitForExistence();
  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;
  bool success = client.call(srv);

  if (argc == 2 && std::string(argv[1]) == "wait")
    Main_Control::create_control(false);
  else
    Main_Control::create_control(true);

  ros::spin();

  Main_Control::kill_control();

  return(0);
}

// ============================================================================================
// definitions of static members in the class
// ============================================================================================
Main_Control* Main_Control::p_control = NULL;
ros::Publisher Main_Control::vel_pub;
ros::Subscriber Main_Control::enable_sub;

// ============================================================================================
// create_control
//
// To make the class singleton, this function creates a new object if there is no existing one.
// otherwise, it returns the existing object.
//
// note: signal funciton must be called
//       after the constructor is called where the node hander is created.
// ============================================================================================
Main_Control *Main_Control::create_control(const bool _enable)
{
  if(Main_Control::p_control == NULL)
  {
    // create a new object
    Main_Control::p_control = new Main_Control(_enable);

    // set up for signal handler
    signal(SIGINT,Main_Control::quit);
  }
  return Main_Control::p_control;
}

// ============================================================================================
// kill_control
//
// It releases the memotry after sending a command to stop the UAV
// ============================================================================================
void Main_Control::kill_control()
{
  if(p_control != NULL)
  {
    delete p_control;
    p_control = NULL;
  }
}

// ============================================================================================
// Constructor
// ============================================================================================
Main_Control::Main_Control(const bool _enable): enable(_enable)
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  Main_Control::vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  list_com_sub[TOPIC_OBS] = n.subscribe(TOPIC_OBS, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);

  Main_Control::enable_sub = n.subscribe("/control_switch", 1, &Main_Control::OnMessage, this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Main_Control::command()
{
  boost::mutex::scoped_lock lock(com_mutex);
  quadrotor_tunnel_nav::Com com;

  com = list_com[TOPIC_OBS];

  if (this->enable)
  {
    ROS_INFO("Command: %s", com.message.c_str());
    Main_Control::vel_pub.publish(com.vel);
  }
  else
  {
    ROS_INFO("Command is disabled.");
  }
}

// ============================================================================================
// quit
//
// it is to be called when Ctrl-C is hit on the terminal.
// it kills the running control and releases the memory.
// ============================================================================================
void Main_Control::quit(int sig)
{
  ROS_INFO("UAV Control: signal received, shutting down");
  Main_Control::kill_control();

  geometry_msgs::Twist vel;
  ROS_INFO("Command: STOP");
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = -1.0;//0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  Main_Control::vel_pub.publish(vel);

  ros::shutdown();
}

// ============================================================================================
void Main_Control::OnMessage(const ros::MessageEvent<std_msgs::Bool const>& event)
{
  const std_msgs::Bool::ConstPtr& flag = event.getMessage();
  this->enable = flag->data;
}
