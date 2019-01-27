// slam.cpp
// 190113
// Implementation of SLAM
//

#include <signal.h>

#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include "slam/slam_base.hpp"

// =============================================================================
// SLAM class
// =============================================================================
class SLAM : public SLAM_BASE
{
  ///\brief Creates a singleton of the class
  //        To make the class singleton, this function creates a new object if
  //        there is no existing one. otherwise, it returns the existing object.
  // \param[in] _enable True if the object must run at the beginning.
  // \return    The class object.
  public: static SLAM *create_slam(const bool _enable);

  ///\brief Releases the memotry after sending a command to stop the UAV
  public: static void kill_slam();

  ///\brief Callback function to receive a message to start operation.
  public: void OnStartMessage(
    const ros::MessageEvent<std_msgs::Bool const>& event);

  ///\brief Main part to process SLAM.
  protected: void proc() override;

  ///\brief Constructor. It is called by create_slam method.
  private: SLAM(const bool _enable);

  ///\brief Callback function to be called when Ctrl-C is hit on the terminal.
  //        it kills the running slam and releases the memory.
  private: static void quit(int);

  protected: static ros::Publisher vel_pub;

  protected: static ros::Subscriber enable_sub;

  private: static SLAM *p_slam;

  private: bool enable;
};

// =============================================================================
// definitions of static members in the class
// =============================================================================
SLAM* SLAM::p_slam = NULL;
ros::Publisher SLAM::vel_pub;
ros::Subscriber SLAM::enable_sub;

////////////////////////////////////////////////////////////////////////////////
SLAM *SLAM::create_slam(const bool _enable)
{
  if(SLAM::p_slam == NULL)
  {
    // create a new object
    SLAM::p_slam = new SLAM(_enable);

    // set up for signal handler
    // note: signal funciton must be called
    //       after the constructor is called where the node hander is created.
    signal(SIGINT,SLAM::quit);
  }
  return SLAM::p_slam;
}

////////////////////////////////////////////////////////////////////////////////
void SLAM::kill_slam()
{
  if(p_slam != NULL)
  {
    delete p_slam;
    p_slam = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
void SLAM::OnStartMessage(const ros::MessageEvent<std_msgs::Bool const>& event)
{
  const std_msgs::Bool::ConstPtr& flag = event.getMessage();
  this->enable = flag->data;
}

////////////////////////////////////////////////////////////////////////////////
void SLAM::proc()
{
  ROS_INFO("This message is from proc() of slam.cpp");

  // what I need:
  // ranging data (rng_x)
  // each sensor's location (org_x) w.r.t. the robot
  // the robot's location
  // m_octree

  // TODO:
  // for each sensor reading:
  // Translate the sensor's location
  // Calculate the detected point
  // update the octree map

  octomath::Vector3 r_position(
    r_pose.pose.position.x, r_pose.pose.position.y, r_pose.pose.position.z);
  octomath::Quaternion r_rotation(
    r_pose.pose.orientation.w, r_pose.pose.orientation.x,
    r_pose.pose.orientation.y, r_pose.pose.orientation.z);
  octomath::Pose6D pose_robot(r_position, r_rotation);

  for (int i = 0; i < 8; ++i)
  {
    octomath::Vector3 point_sensor(rng_h[i].range, 0, 0);

    octomath::Vector3 sensor_global = pose_robot.transform(pose_h[i].trans());
    octomath::Vector3 point_global
      = pose_robot.transform(pose_h[i].transform(point_sensor));

    m_octree->insertRay(sensor_global, point_global, 9.0);
  }

  for (int i = 0; i < 3; ++i)
  {
    octomath::Vector3 point_sensor(rng_u[i].range, 0, 0);

    octomath::Vector3 sensor_global = pose_robot.transform(pose_u[i].trans());
    octomath::Vector3 point_global
      = pose_robot.transform(pose_u[i].transform(point_sensor));

    m_octree->insertRay(sensor_global, point_global, 9.0);
  }

  for (int i = 0; i < 3; ++i)
  {
    octomath::Vector3 point_sensor(rng_d[i].range, 0, 0);

    octomath::Vector3 sensor_global = pose_robot.transform(pose_d[i].trans());
    octomath::Vector3 point_global
      = pose_robot.transform(pose_d[i].transform(point_sensor));

    m_octree->insertRay(sensor_global, point_global, 9.0);
  }

  octomap_msgs::Octomap map;
  map.header.frame_id = "base_link";
  map.header.stamp = ros::Time::now();
  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    map_pub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}



////////////////////////////////////////////////////////////////////////////////
SLAM::SLAM(const bool _enable): enable(_enable)
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  SLAM::vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  SLAM::enable_sub
    = n.subscribe("/start_slam", 1, &SLAM::OnStartMessage, this);
}

////////////////////////////////////////////////////////////////////////////////
void SLAM::quit(int /*sig*/)
{
  ROS_INFO("UAV Control: signal received, shutting down");
  SLAM::kill_slam();

  geometry_msgs::Twist vel;
  ROS_INFO("Command: STOP");
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = -1.0;//0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  SLAM::vel_pub.publish(vel);

  ros::shutdown();
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "main_slam");

  // enable the motors
  ros::NodeHandle n;
  ros::ServiceClient client
    = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  client.waitForExistence();
  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;
  client.call(srv);

  if (argc == 2 && std::string(argv[1]) == "wait")
    SLAM::create_slam(false);
  else
    SLAM::create_slam(true);

  ros::spin();

  SLAM::kill_slam();

  return(0);
}
