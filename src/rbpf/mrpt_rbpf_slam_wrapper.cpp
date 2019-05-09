#include "rbpf/mrpt_rbpf_slam_wrapper.h"

#include "rbpf/options.h"

#include "rbpf/CustomOctoMap.h"

namespace
{
bool isFileExists(const std::string& name)
{
  std::ifstream f(name.c_str());
  return f.good();
}
}  // namespace

namespace mrpt_rbpf_slam
{
PFslamWrapper::PFslamWrapper()
{
  mrpt_bridge::convert(ros::Time(0), timeLastUpdate_);
}

bool PFslamWrapper::getParams(const ros::NodeHandle& nh_p)
{
  ROS_INFO("READ PARAM FROM LAUNCH FILE");

  nh_p.getParam("ini_filename", ini_filename_);
  ROS_INFO("ini_filename: %s", ini_filename_.c_str());

  nh_p.param<std::string>("gtruthw_frame_id", gtruthw_frame_id_, "ground_truth/world");
  ROS_INFO("gtruthw_frame_id: %s", gtruthw_frame_id_.c_str());
  nh_p.param<std::string>("gtruthb_frame_id", gtruthb_frame_id_, "ground_truth/base_link");
  ROS_INFO("gtruthb_frame_id: %s", gtruthb_frame_id_.c_str());

  nh_p.param<std::string>("global_frame_id", global_frame_id_, "map");
  ROS_INFO("global_frame_id: %s", global_frame_id_.c_str());

  nh_p.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  ROS_INFO("odom_frame_id: %s", odom_frame_id_.c_str());

  nh_p.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  ROS_INFO("base_frame_id: %s", base_frame_id_.c_str());

  nh_p.param<std::string>("sensor_source", sensor_source_, "");
  ROS_INFO("sensor_source: %s", sensor_source_.c_str());

  nh_p.param("sensor_rate", rate_, 10.); // see the yaml file for more details.

  PFslam::Options options;
  if (!loadOptions(nh_p, options))
  {
    ROS_ERROR("Not able to read all parameters!");
    return false;
  }
  initSlam(std::move(options));
  return true;
}

bool PFslamWrapper::init(ros::NodeHandle& nh)
{
  // get parameters from ini file
  if (!isFileExists(ini_filename_))
  {
    ROS_ERROR_STREAM("CAN'T READ INI FILE" << ini_filename_);
    return false;
  }

  PFslam::readIniFile(ini_filename_);

  /// Create publishers///
  // publish grid map
  pub_map_ = nh.advertise<octomap_msgs::Octomap>("map", 1, true);
  // robot pose
  pub_particles_ = nh.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);

  // read sensor topics
  std::vector<std::string> lstSources;
  mrpt::system::tokenize(sensor_source_, " ,\t\n", lstSources);
  ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. "
                                      "\"scan\" or \"beacon\")");

  /// Create subscribers///
  sensorSub_.resize(lstSources.size());
  for (size_t i = 0; i < lstSources.size(); i++)
  {
    sensorSub_[i] = nh.subscribe(lstSources[i], 1, &PFslamWrapper::rangeCallback, this);
  }

  mapBuilder_ = mrpt::slam::CMetricMapBuilderRBPF(options_.rbpfMappingOptions_);

  // replace the map with the customized octomap
  for (auto& particle: mapBuilder_.mapPDF.m_particles)
  {
    auto map = mrpt::maps::CustomOctoMap::Create();
    auto org
      = mrpt::maps::COctoMap::Ptr(particle.d->mapTillNow.maps[0].get_ptr());

    // TODO: copy settings
    map->insertionOptions = org->insertionOptions;
    map->renderingOptions = org->renderingOptions;
    map->likelihoodOptions = org->likelihoodOptions;
    map->setResolution(org->getResolution());

    particle.d->mapTillNow.maps[0] = map;
  }


  // map visualization
  m_color_occupied.r = 1;
  m_color_occupied.g = 1;
  m_color_occupied.b = 0.3;
  m_color_occupied.a = 0.5;
  marker_occupied_pub
    = nh.advertise<visualization_msgs::MarkerArray>("map_marker_occupied", 1);
  marker_counter = 0;

  odometryOrgUnitialized_ = true;
  locationOrgUnitialized_ = true;
  gtruthLocOrgUnitialized_ = true;

  return true;
}

// ========================================================
bool PFslamWrapper::waitForTransform(mrpt::poses::CPose3D& des, const std::string& target_frame,
                                     const std::string& source_frame, const ros::Time& time,
                                     const ros::Duration& timeout, const ros::Duration& polling_sleep_duration)
{
  tf::StampedTransform transform;
  try
  {
    listenerTF_.waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
    listenerTF_.lookupTransform(target_frame, source_frame, time, transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Failed to get transform target_frame (%s) to source_frame (%s). TransformException: %s",
              target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
  mrpt_bridge::convert(transform, des);
  return true;
}

// ========================================================
void PFslamWrapper::rangeCallback(const sensor_msgs::Range& msg)
{
  std::lock_guard<std::mutex> lk(sensor_mutex);

  if (range_poses_.find(msg.header.frame_id) == range_poses_.end())
  {
    updateSensorPose(msg.header.frame_id);
  }

  if (sensor_buffer.count(msg.header.frame_id) == 0)
  {
    sensor_buffer[msg.header.frame_id] = std::shared_ptr<sensor_msgs::Range>();
  }
  sensor_buffer[msg.header.frame_id] = std::make_shared<sensor_msgs::Range>(msg);
}

// ========================================================
void PFslamWrapper::procSensoryData()
{
  using namespace mrpt::maps;
  using namespace mrpt::obs;

  // create a local point cloud observation w.r.t. the robot's ref frame.
  // as of 4/21, 2019, the version of MRPT for ROS is 1.5 and CObservationPointCloud is not available.
  // Temporarily, CObservation3DRangeScan is used.
  CObservation3DRangeScan::Ptr pc = CObservation3DRangeScan::Create();

  {
    std::lock_guard<std::mutex> lk(sensor_mutex);

    if (sensor_buffer.size() < sensorSub_.size())
      return;

    // CObservationPointCloud::Ptr pc = CObservationPointCloud::Create();
    for (auto& pair: sensor_buffer)
    {
      auto p_msg = pair.second;

      currentTime_ = p_msg->header.stamp;

      mrpt_bridge::convert(p_msg->header.stamp, pc->timestamp);

      mrpt::poses::CPoint3D dtpoint
        = range_poses_[p_msg->header.frame_id]
        + mrpt::poses::CPoint3D(p_msg->range, 0, 0);

      pc->points3D_x.push_back(dtpoint.x());
      pc->points3D_y.push_back(dtpoint.y());
      pc->points3D_z.push_back(dtpoint.z());
      pc->hasPoints3D = true;
      // pc->pointcloud->insertPointFast(dtpoint.x(), dtpoint.y(), dtpoint.z());
    }
    sensor_buffer.clear();
  }

  sensory_frame_ = CSensoryFrame::Create();
  CObservation::Ptr obs = CObservation::Ptr(pc);
  sensory_frame_->insert(obs);

  if (this->waitForTransform(odometry_, odom_frame_id_, base_frame_id_, currentTime_, ros::Duration(1)))
  {
    // store the initial odometry location.
    if (odometryOrgUnitialized_)
    {
      odometryOrg_ = odometry_;
      odometryOrgUnitialized_ = false;
    }
  }
  if (this->waitForTransform(gtruthLoc_, gtruthw_frame_id_, gtruthb_frame_id_, currentTime_, ros::Duration(1)))
  {
    // store the initial gtruthLoc location.
    if (gtruthLocOrgUnitialized_)
    {
      gtruthLocOrg_ = gtruthLoc_;
      gtruthLocOrgUnitialized_ = false;
    }
  }

  if (observation(sensory_frame_, odometry_))
  {
    timeLastUpdate_ = sensory_frame_->getObservationByIndex(0)->timestamp;

    tictac_.Tic();
    ROS_INFO("================= processActionObservation start ====================");
    mapBuilder_.processActionObservation(*action_, *sensory_frame_);
    ROS_INFO("================= processActionObservation end ====================");
    t_exec_ = tictac_.Tac();
    ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec_);
  }
  publishMapPose();
  publishTF();
  publishVisMap();
}
// =============================================================
void PFslamWrapper::publishMapPose()
{
  metric_map_ = mapBuilder_.mapPDF.getCurrentMostLikelyMetricMap();
  mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);
  if (metric_map_->maps.size())
  {
    auto octomap = mrpt::maps::COctoMap::Ptr(metric_map_->maps[0].get_ptr());
    octomap::OcTree &m_octree = octomap->getOctomap<octomap::OcTree>();
    // publish map
    octomap_msgs::Octomap msg;
    if (octomap_msgs::fullMapToMsg(m_octree, msg))
      pub_map_.publish(msg);
    else
      ROS_ERROR("Error serializing OctoMap");
  }

  // publish pose
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = global_frame_id_;
  poseArray.header.stamp = ros::Time::now();
  poseArray.poses.resize(curPDF.particlesCount());
  for (size_t i = 0; i < curPDF.particlesCount(); i++)
  {
    const auto p = mrpt::poses::CPose3D(curPDF.getParticlePose(i));
    mrpt_bridge::convert(p, poseArray.poses[i]);
  }
  pub_particles_.publish(poseArray);

  /*
  ROS_INFO_STREAM("# of particles: " << mapBuilder_.mapPDF.m_particles.size());
  for (int i = 0; i < mapBuilder_.mapPDF.m_particles.size(); ++i)
    ROS_INFO_STREAM("m_particles[" << i << "]: " << mapBuilder_.mapPDF.m_particles[i].log_w);
  */
}

void PFslamWrapper::updateSensorPose(const std::string& frame_id)
{
  mrpt::poses::CPose3D pose;
  tf::StampedTransform transform;
  try
  {
    listenerTF_.lookupTransform(base_frame_id_, frame_id, ros::Time(0), transform);

    tf::Vector3 translation = transform.getOrigin();
    tf::Quaternion quat = transform.getRotation();
    pose.x() = translation.x();
    pose.y() = translation.y();
    pose.z() = translation.z();
    tf::Matrix3x3 Rsrc(quat);
    mrpt::math::CMatrixDouble33 Rdes;
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++)
        Rdes(r, c) = Rsrc.getRow(r)[c];
    pose.setRotationMatrix(Rdes);
    range_poses_[frame_id] = pose;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void PFslamWrapper::publishTF()
{
  // Most of this code was copy and pase from ros::amcl
  mrpt::poses::CPose3D robotPose;
  mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);

  curPDF.getMean(robotPose);

  location_ = robotPose;
  // store the initial location location.
  if (locationOrgUnitialized_)
  {
    locationOrg_ = location_;
    locationOrgUnitialized_ = false;
  }

  tf::Stamped<tf::Pose> odom_to_map;
  tf::Transform tmp_tf;
  ros::Time stamp;
  mrpt_bridge::convert(timeLastUpdate_, stamp);
  mrpt_bridge::convert(robotPose, tmp_tf);

  try
  {
    tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), stamp, base_frame_id_);
    listenerTF_.transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Failed to subtract global_frame (%s) from odom_frame (%s). TransformException: %s",
              global_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
    return;
  }

  tf::Transform latest_tf_ =
      tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used

  ros::Duration transform_tolerance_(0.5);
  ros::Time transform_expiration = (stamp + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
  tf_broadcaster_.sendTransform(tmp_tf_stamped);
}

void PFslamWrapper::publishVisMap()
{
  // publish it oly once at each <rate_> updates.
  if (marker_counter < rate_)
  {
    ++marker_counter;
    return;
  }
  marker_counter = 0;

  metric_map_ = mapBuilder_.mapPDF.getCurrentMostLikelyMetricMap();
  auto octomap = mrpt::maps::COctoMap::Ptr(metric_map_->maps[0].get_ptr());
  octomap::OcTree &m_octree = octomap->getOctomap<octomap::OcTree>();
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_octree.getTreeDepth()+1);
  for (
    octomap::OcTree::iterator it = m_octree.begin(m_octree.getTreeDepth()),
    end = m_octree.end(); it != end; ++it)
  {
    if (m_octree.isNodeAtThreshold(*it))
    {
      double x = it.getX();
      double z = it.getZ();
      double y = it.getY();

      unsigned idx = it.getDepth();
      geometry_msgs::Point cubeCenter;
      cubeCenter.x = x;
      cubeCenter.y = y;
      cubeCenter.z = z;

      if (m_octree.isNodeOccupied(*it))
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
    }
  }

  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
  {
    double size = m_octree.getNodeSize(i);

    occupiedNodesVis.markers[i].header.frame_id = "map";
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
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
  if (occupiedNodesVis.markers.size() > 0)
    marker_occupied_pub.publish(occupiedNodesVis);
}
}  // namespace mrpt_rbpf_slam
