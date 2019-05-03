#pragma once
#include <ros/console.h>

#include <mrpt/random.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#else
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#endif
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/version.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement3D.h>

#include <mrpt/obs/CObservation3DRangeScan.h> // may replace with the following for MRPT v1.9+
//#include <mrpt/obs/CObservationPointCloud.h>

#include <mrpt/obs/CSensoryFrame.h>

namespace mrpt_rbpf_slam
{
/**
 * @brief The PFslam class provides Rao-Blackwellized Particle filter SLAM from
 * MRPT libraries.
 */
class PFslam
{
public:
  struct Options
  {
    mrpt::obs::CActionRobotMovement3D::TMotionModelOptions motion_model_options_;  ///< used with odom value motion
                                                                                   ///< noise
    mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions rbpfMappingOptions_;   ///< options for SLAM from ini file
    bool CAMERA_3DSCENE_FOLLOWS_ROBOT_;
    bool SHOW_PROGRESS_IN_WINDOW_;
    int SHOW_PROGRESS_IN_WINDOW_DELAY_MS_;
    int PROGRESS_WINDOW_WIDTH_, PROGRESS_WINDOW_HEIGHT_;
    std::string simplemap_path_prefix;
  } options_;

  PFslam() = default;
  virtual ~PFslam();

  /**
   * @brief Read ini file
   *
   * @param[in] ini_filename the name of the ini file to read
   */
  void readIniFile(const std::string& ini_filename);

  /**
   * @brief initialize the SLAM
   */
  void initSlam(Options options);

  /**
   * @brief Calculate the actions from odometry model for current observation
   *
   * @param[in] sensory_frame  current observation
   * @param[in] odometry raw odometry
   */
  bool observation(const mrpt::obs::CSensoryFrame::ConstPtr sensory_frame,
                   const mrpt::poses::CPose3D& odometry);

protected:
  mrpt::slam::CMetricMapBuilderRBPF mapBuilder_;  ///< map builder
  mrpt::obs::CActionCollection::Ptr action_;      ///< actions
  mrpt::obs::CSensoryFrame::Ptr sensory_frame_;   ///< observations

  mrpt::poses::CPose3D odomLastObservation_;  ///< last observation of odometry
  bool odomLastPoseUninitalized_;
  bool use_motion_model_default_options_;     ///< used default odom_params
  mrpt::system::TTimeStamp timeLastUpdate_;   ///< last update of the pose and map

  const mrpt::maps::CMultiMetricMap* metric_map_;  ///< receive map after iteration of SLAM to metric map
  mrpt::poses::CPose3DPDFParticles curPDF;         ///< current robot pose
};
}  // namespace mrpt_rbpf_slam
