/*
 *  File: mrpt_slam.cpp
 *  Author: Vladislav Tananaev
 *
 *
 */
#include "rbpf/mrpt_rbpf_slam.h"
#include <mrpt/version.h>
#include <mrpt_bridge/utils.h>

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

void PFslam::observation(const mrpt::obs::CSensoryFrame::ConstPtr sensory_frame,
                         const mrpt::poses::CPose3D& odometry)
{
  action_ = mrpt::obs::CActionCollection::Create();
  mrpt::obs::CActionRobotMovement3D odom_move;
  odom_move.timestamp = sensory_frame->getObservationByIndex(0)->timestamp;

  if (odomLastObservation_.empty())
  {
    odomLastObservation_ = odometry;
  }

  mrpt::poses::CPose3D incOdoPose = odometry - odomLastObservation_;
  odomLastObservation_ = odometry;
  odom_move.computeFromOdometry(incOdoPose, options_.motion_model_options_);
  action_->insert(odom_move);
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
}
}  // namespace mrpt_rbpf_slam
