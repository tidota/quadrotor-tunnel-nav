// uav_control.cpp
// 161130
// this is the control code for UAV in the simulated enviornment by gazebo.
// 

#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "boost/thread/mutex.hpp"


// ============================================================================================
// UAV_Control class
// it contains everything necessary for control
// ============================================================================================
class UAV_Control
{
public:
  // the instance of this class must be single
  // so this function must be called to create the object.
  static UAV_Control *create_control();

  // to release the memory, call this function.
  static void kill_control();

private:
  UAV_Control();

  void command();
  void stop();

  static void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);
  void updateScanData(const sensor_msgs::LaserScan::ConstPtr&);

  static void quit(int);

private:
  static UAV_Control *p_control;

  static ros::Publisher vel_pub;
  static ros::Subscriber scan_sub;

  ros::Timer timer;

  boost::mutex scan_mutex;
  //sensor_msgs::LaserScan scan_h; // horizontal
  double scan_h[5]; // horizontal
  double scan_v[3]; // vertical
  //sensor_msgs::LaserScan scan_v; // vertical

  bool f_up;
};

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_control");

  UAV_Control::create_control();

  ros::spin();

  UAV_Control::kill_control();

  return(0);
}

// ============================================================================================
// definitions of static members in the class
// ============================================================================================
UAV_Control* UAV_Control::p_control = NULL;
ros::Publisher UAV_Control::vel_pub;
ros::Subscriber UAV_Control::scan_sub;

// ============================================================================================
// create_control
//
// To make the class singleton, this function creates a new object if there is no existing one.
// otherwise, it returns the existing object.
//
// note: signal funciton must be called
//       after the constructor is called where the node hander is created.
// ============================================================================================
UAV_Control *UAV_Control::create_control()
{
  if(p_control == NULL)
  {
    // create a new object
    p_control = new UAV_Control();

    // set up for signal handler
    signal(SIGINT,UAV_Control::quit);
  }
  return p_control;
}

// ============================================================================================
// kill_control
//
// It releases the memotry after sending a command to stop the UAV
// ============================================================================================
void UAV_Control::kill_control()
{
  if(p_control != NULL)
  {
    p_control->stop(); // stops the UAV and it will stay on the current location.
    delete p_control;
    p_control = NULL;
  }
}

// ============================================================================================
// Constructor
// ============================================================================================
UAV_Control::UAV_Control(): f_up(true)
{
  // set up for publisher, subscriber, and timer
  ros::NodeHandle n;
  UAV_Control::vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  UAV_Control::scan_sub = n.subscribe("/scan", 1, UAV_Control::scanCallback);
  timer = n.createTimer(ros::Duration(4.0), boost::bind(&UAV_Control::command, this));
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void UAV_Control::command()
{
  geometry_msgs::Twist vel;

  vel.linear.x = 0.2;
  vel.angular.z = 0.5;

  if(f_up)
  {
    vel.linear.z = 0.5;
    f_up = false;
  }
  else
  {
    vel.linear.z = -0.5;
    f_up = true;
  }

  ROS_INFO("%1.1f %1.1f %1.1f %1.1f %1.1f | %1.1f %1.1f %1.1f",
    scan_h[0], scan_h[1], scan_h[2], scan_h[3], scan_h[4], scan_v[0], scan_v[1], scan_v[2]);

  UAV_Control::vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void UAV_Control::stop()
{
  geometry_msgs::Twist vel;
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  UAV_Control::vel_pub.publish(vel);
  timer.stop();
}

// ============================================================================================
// scanCallback
//
// it is called when scanning is complete.
// it receives scan data based on the topic, sensor_msgs/LaserScan.
// the sensor data is stored in the static field.
// ============================================================================================
void UAV_Control::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  p_control->updateScanData(scan);
}
// ============================================================================================
// updateScanData
//
// it stores scan data into the UAV_Control object
// ============================================================================================
void UAV_Control::updateScanData(const sensor_msgs::LaserScan::ConstPtr& new_scan)
{
  boost::mutex::scoped_lock lock(UAV_Control::scan_mutex);
  double half_angle = (new_scan->angle_max - new_scan->angle_min)/2;
  int num = new_scan->ranges.size();
  int num_2 = new_scan->ranges.size()/2*(M_PI/2)/half_angle;
  int num_4 = new_scan->ranges.size()/2*(M_PI/4)/half_angle;
  if(new_scan->header.frame_id.compare("laser0_frame") == 0)
  {
    scan_h[0] = new_scan->ranges[num/2-num_2]; // 90 degrees to the right
    scan_h[1] = new_scan->ranges[num/2-num_4]; // 45 degrees to the right
    scan_h[2] = new_scan->ranges[num/2];       // center
    scan_h[3] = new_scan->ranges[num/2+num_4]; // 45 degrees to the left
    scan_h[4] = new_scan->ranges[num/2+num_2]; // 90 degrees to the left
  }
  else if(new_scan->header.frame_id.compare("laser1_frame") == 0)
  {
    scan_v[0] = new_scan->ranges[num/2-num_4]; // 45 degrees to the rear from the nadir
    scan_v[1] = new_scan->ranges[num/2];       // nadir
    scan_v[2] = new_scan->ranges[num/2+num_4]; // 45 degrees to the front from the nadir
  }
}
// ============================================================================================
// ============================================================================================

// ============================================================================================
// quit
//
// it is to be called when Ctrl-C is hit on the terminal.
// it kills the running control and releases the memory.
// ============================================================================================
void UAV_Control::quit(int sig)
{
  ROS_INFO("UAV Control: signal received, shutting down");
  UAV_Control::kill_control();
  ros::shutdown();
}



