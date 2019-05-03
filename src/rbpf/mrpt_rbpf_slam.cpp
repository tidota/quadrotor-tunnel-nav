/*
 *  File: mrpt_slam.cpp
 *  Author: Vladislav Tananaev
 *
 *
 */
#include "rbpf/mrpt_rbpf_slam.h"
#include <mrpt/version.h>
#include <mrpt_bridge/utils.h>

// For calculation of motion model
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/random.h>

namespace mrpt_rbpf_slam
{
PFslam::~PFslam()
{
  try
  {
    std::string sOutMap = "mrpt_rbpfslam_";
    mrpt::system::TTimeParts parts;
    mrpt::system::timestampToParts(mrpt::system::now(), parts, true);
    sOutMap += mrpt::format("%04u-%02u-%02u_%02uh%02um%02us", (unsigned int)parts.year, (unsigned int)parts.month,
                            (unsigned int)parts.day, (unsigned int)parts.hour, (unsigned int)parts.minute,
                            (unsigned int)parts.second);
    sOutMap += ".simplemap";

    sOutMap = mrpt::system::fileNameStripInvalidChars(sOutMap);
    if (mrpt::system::directoryExists(options_.simplemap_path_prefix))
    {
      sOutMap = options_.simplemap_path_prefix + sOutMap;
    }
    else
    {
      std::cerr << "Folder " << options_.simplemap_path_prefix
                << " doesn't exist, cannot save simplemap there! Default '~/.ros/' path will be used.\n";
    }
    std::cout << "Saving built map " << sOutMap << "\n";
    mapBuilder_.saveCurrentMapToFile(sOutMap);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}

void PFslam::readIniFile(const std::string& ini_filename)
{
#if MRPT_VERSION >= 0x199
  mrpt::config::CConfigFile iniFile(ini_filename);
#else
  mrpt::utils::CConfigFile iniFile(ini_filename);
#endif
  options_.rbpfMappingOptions_.loadFromConfigFile(iniFile, "MappingApplication");
  options_.rbpfMappingOptions_.dumpToConsole();
}

bool PFslam::observation(const mrpt::obs::CSensoryFrame::ConstPtr sensory_frame,
                         const mrpt::poses::CPose3D& odometry)
{
  action_ = mrpt::obs::CActionCollection::Create();
  mrpt::obs::CActionRobotMovement3D odom_move;
  odom_move.timestamp = sensory_frame->getObservationByIndex(0)->timestamp;

  if (odomLastPoseUninitalized_)
  {
    // the location is estimated based on incremental offset from the previous
    // pose. By initializing the first pose with only x, y, z, the initial pose
    // can be taken into considertation on the estimation.
    odomLastObservation_ = mrpt::poses::CPose3D(
      odometry.x(), odometry.y(), odometry.z(), 0, 0, 0);
    odomLastPoseUninitalized_ = false;
    return false;
  }

  mrpt::poses::CPose3D incOdoPose = odometry - odomLastObservation_;
  odomLastObservation_ = odometry;

  // Just apply Gaussian
  //odom_move.computeFromOdometry_model6DOF(incOdoPose, options_.motion_model_options_);
  {
    using namespace mrpt::poses;
    using namespace mrpt::random;

    CPose3DPDFParticles		*aux;
    static CPose3D nullPose(0,0,0,0,0,0);

    mrpt::poses::CPose3DPDFPtr poseChangeTemp = CPose3DPDFParticles::Create();
    aux = static_cast<CPose3DPDFParticles*>( poseChangeTemp.pointer() );

    // Set the number of particles:
    aux->resetDeterministic(nullPose, 300);

    // Draw samples:
    for (size_t i = 0; i < 300; i++)
    {
      //const double trn = incOdoPose.norm();
      const double lim = 0.15;
      double ampx = incOdoPose.x() * 1.5; //(trn * 1.5 < 0.15)? (trn) * 1.5: 0.15;
      ampx = (-lim < ampx && ampx < lim)? ampx: lim;
      double ampy = incOdoPose.y() * 1.5; //(trn * 1.5 < 0.15)? (trn) * 1.5: 0.15;
      ampy = (-lim < ampy && ampy < lim)? ampy: lim;
      double ampz = incOdoPose.z() * 1.5; //(trn * 1.5 < 0.15)? (trn) * 1.5: 0.15;
      ampz = (-lim < ampz && ampz < lim)? ampz: lim;
      aux->m_particles[i].d->x(
        incOdoPose.x() + ampx * randomGenerator.drawGaussian1D_normalized());
        //+ ampx * randomGenerator.drawGaussian1D_normalized());
      aux->m_particles[i].d->y(
        incOdoPose.y() + ampy * randomGenerator.drawGaussian1D_normalized());
        //+ ampy * randomGenerator.drawGaussian1D_normalized());
      aux->m_particles[i].d->z(
        incOdoPose.z() + ampz * randomGenerator.drawGaussian1D_normalized());
        //+ ampz * randomGenerator.drawGaussian1D_normalized());
      aux->m_particles[i].d->setYawPitchRoll(
        incOdoPose.yaw() + 0.001 * randomGenerator.drawGaussian1D_normalized(),
        incOdoPose.pitch() + 0.001 * randomGenerator.drawGaussian1D_normalized(),
        incOdoPose.roll() + 0.001 * randomGenerator.drawGaussian1D_normalized());
      aux->m_particles[i].d->normalizeAngles();
    }

    odom_move.poseChange.copyFrom(*poseChangeTemp);
  }

  action_->insert(odom_move);

  /*
  auto pose = odom_move.poseChange.getPoseMean();
  ROS_INFO_STREAM("odom_move: norm = " << pose.norm()
  << ", pitch = " << (pose.pitch() * 180.0 / 3.15159265)
  << ", roll = " << (pose.roll() * 180.0 / 3.15159265)
  << ", yaw = " << (pose.yaw() * 180.0 / 3.15159265));
  */
  return true;
}

void PFslam::initSlam(PFslam::Options options)
{
  log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  mapBuilder_.setVerbosityLevel(mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl(ros_logger->getLevel()));
  mapBuilder_.logging_enable_console_output = false;
#if MRPT_VERSION < 0x199
  mapBuilder_.logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback_mrpt_15));
#else
  mapBuilder_.logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback));
#endif
  mapBuilder_.options.enableMapUpdating = true;
  mapBuilder_.options.debugForceInsertion = false;

#if MRPT_VERSION >= 0x199
  mrpt::random::getRandomGenerator().randomize();
#else
  mrpt::random::randomGenerator.randomize();
#endif

  options_ = std::move(options);

  odomLastPoseUninitalized_ = true;
}
}  // namespace mrpt_rbpf_slam
