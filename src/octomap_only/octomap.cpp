// octomap.cpp
// 190113
// Implementation of OCTOMAP
//

#include <cmath>

#include <signal.h>

#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include "octomap_only/octomap_base.hpp"

// =============================================================================
// OCTOMAP class
// =============================================================================
class OCTOMAP : public OCTOMAP_BASE
{
  ///\brief Creates a singleton of the class
  //        To make the class singleton, this function creates a new object if
  //        there is no existing one. otherwise, it returns the existing object.
  // \param[in] _enable True if the object must run at the beginning.
  // \return    The class object.
  public: static OCTOMAP *create_octomap(const bool _enable);

  ///\brief Releases the memotry after sending a command to stop the UAV
  public: static void kill_octomap();

  ///\brief Callback function to receive a message to start operation.
  public: void OnStartMessage(
    const ros::MessageEvent<std_msgs::Bool const>& event);

  ///\brief Main part to process OCTOMAP.
  protected: void proc() override;

  ///\brief Constructor. It is called by create_octomap method.
  private: OCTOMAP(const bool _enable);

  ///\brief Callback function to be called when Ctrl-C is hit on the terminal.
  //        it kills the running octomap and releases the memory.
  private: static void quit(int);

  protected: static ros::Publisher vel_pub;

  protected: static ros::Subscriber enable_sub;

  private: static OCTOMAP *p_octomap;

  private: bool enable;
};

// =============================================================================
// definitions of static members in the class
// =============================================================================
OCTOMAP* OCTOMAP::p_octomap = NULL;
ros::Publisher OCTOMAP::vel_pub;
ros::Subscriber OCTOMAP::enable_sub;

////////////////////////////////////////////////////////////////////////////////
OCTOMAP *OCTOMAP::create_octomap(const bool _enable)
{
  if(OCTOMAP::p_octomap == NULL)
  {
    // create a new object
    OCTOMAP::p_octomap = new OCTOMAP(_enable);

    // set up for signal handler
    // note: signal funciton must be called
    //       after the constructor is called where the node hander is created.
    signal(SIGINT,OCTOMAP::quit);
  }
  return OCTOMAP::p_octomap;
}

////////////////////////////////////////////////////////////////////////////////
void OCTOMAP::kill_octomap()
{
  if(p_octomap != NULL)
  {
    delete p_octomap;
    p_octomap = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
void OCTOMAP::OnStartMessage(const ros::MessageEvent<std_msgs::Bool const>& event)
{
  const std_msgs::Bool::ConstPtr& flag = event.getMessage();
  this->enable = flag->data;
}

////////////////////////////////////////////////////////////////////////////////
void OCTOMAP::proc()
{
  ROS_INFO("This message is from proc() of octomap.cpp");

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

          double cosR = std::cos(PI*z/10.0)*0.8+0.2;
          double cosG = std::cos(PI*(2.0/3.0+z/10.0))*0.8+0.2;
          double cosB = std::cos(PI*(4.0/3.0+z/10.0))*0.8+0.2;
          std_msgs::ColorRGBA clr;
          clr.r = (cosR > 0)? cosR: 0;
          clr.g = (cosG > 0)? cosG: 0;
          clr.b = (cosB > 0)? cosB: 0;
          clr.a = 0.5;
          occupiedNodesVis.markers[idx].colors.push_back(clr);
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

      //occupiedNodesVis.markers[i].color = m_color_occupied;

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
OCTOMAP::OCTOMAP(const bool _enable): enable(_enable)
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  OCTOMAP::vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  OCTOMAP::enable_sub
    = n.subscribe("/start_octomap", 1, &OCTOMAP::OnStartMessage, this);
}

////////////////////////////////////////////////////////////////////////////////
void OCTOMAP::quit(int /*sig*/)
{
  ROS_INFO("UAV Control: signal received, shutting down");
  OCTOMAP::kill_octomap();

  geometry_msgs::Twist vel;
  ROS_INFO("Command: STOP");
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = -1.0;//0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  OCTOMAP::vel_pub.publish(vel);

  ros::shutdown();
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "main_octomap");

  // enable the motors
  ros::NodeHandle n;
  ros::ServiceClient client
    = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  client.waitForExistence();
  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;
  client.call(srv);

  if (argc == 2 && std::string(argv[1]) == "wait")
    OCTOMAP::create_octomap(false);
  else
    OCTOMAP::create_octomap(true);

  ros::spin();

  OCTOMAP::kill_octomap();

  return(0);
}
