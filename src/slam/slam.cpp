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

  auto rostime = ros::Time::now();

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
  map.header.frame_id = "world";
  map.header.stamp = rostime;
  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    map_pub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");

  if (marker_counter >= 5)
  {
    m_octree->toMaxLikelihood();
    m_octree->prune();

    for (
      octomap::OcTree::iterator it = m_octree->begin(m_octree->getTreeDepth()),
      end = m_octree->end(); it != end; ++it)
    {
      if (m_octree->isNodeAtThreshold(*it))
      {
        double x = it.getX();
        double z = it.getZ();
        double y = it.getY();

        unsigned idx = it.getDepth();
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        if (m_octree->isNodeOccupied(*it))
        {
          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
        }
        else
        {
          freeNodesVis.markers[idx].points.push_back(cubeCenter);
        }
      }
    }

    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
    {
      double size = m_octree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = "world";
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "robot";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;

      occupiedNodesVis.markers[i].color = m_color_occupied;

      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    marker_occupied_pub.publish(occupiedNodesVis);

    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i)
    {
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = "world";
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "robot";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;

      freeNodesVis.markers[i].color = m_color_free;

      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    marker_free_pub.publish(freeNodesVis);

    marker_counter = 0;
  }
  else
  {
    marker_counter++;
  }

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
