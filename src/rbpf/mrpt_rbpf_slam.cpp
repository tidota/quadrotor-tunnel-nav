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

  // Just apply Gaussian
  //odom_move.computeFromOdometry_model6DOF(incOdoPose, options_.motion_model_options_);
  {
    CPose3DPDFParticles* aux;
    const mrpt::math::TPose3D nullPose(0, 0, 0, 0, 0, 0);

    mrpt::poses::CPose3DPDF::Ptr poseChangeTemp =
        mrpt::make_aligned_shared<CPose3DPDFParticles>();
    aux = dynamic_cast<CPose3DPDFParticles*>(poseChangeTemp.get());

    // Set the number of particles:
    aux->resetDeterministic(nullPose, o.mm6DOFModel.nParticlesCount);
    // Draw samples:
    for (size_t i = 0; i < o.mm6DOFModel.nParticlesCount; i++)
    {
       float Ayaw1_draw =
           Ayaw1 + (o.mm6DOFModel.a1 * Ayaw1 + o.mm6DOFModel.a2 * Atrans) *
                       getRandomGenerator().drawGaussian1D_normalized();
       float Apitch1_draw =
           Apitch1 + (o.mm6DOFModel.a3 * odometryIncrement.z()) *
                         getRandomGenerator().drawGaussian1D_normalized();
       float Atrans_draw =
           Atrans + (o.mm6DOFModel.a4 * Atrans + o.mm6DOFModel.a5 * Ayaw2 +
                     o.mm6DOFModel.a6 * (Aroll + Apitch2)) *
                        getRandomGenerator().drawGaussian1D_normalized();

       float Aroll_draw =
           Aroll + (o.mm6DOFModel.a7 * Aroll) *
                       getRandomGenerator().drawGaussian1D_normalized();
       float Apitch2_draw =
           Apitch2 + (o.mm6DOFModel.a8 * Apitch2) *
                         getRandomGenerator().drawGaussian1D_normalized();
       float Ayaw2_draw =
           Ayaw2 + (o.mm6DOFModel.a9 * Ayaw2 + o.mm6DOFModel.a10 * Atrans) *
                       getRandomGenerator().drawGaussian1D_normalized();

       // Output:
       aux->m_particles[i].d.x =
           Atrans_draw * sin(Apitch1_draw) * cos(Ayaw1_draw) +
           motionModelConfiguration.mm6DOFModel.additional_std_XYZ *
               getRandomGenerator().drawGaussian1D_normalized();
       aux->m_particles[i].d.y =
           Atrans_draw * sin(Apitch1_draw) * sin(Ayaw1_draw) +
           motionModelConfiguration.mm6DOFModel.additional_std_XYZ *
               getRandomGenerator().drawGaussian1D_normalized();
       aux->m_particles[i].d.z =
           Atrans_draw * cos(Apitch1_draw) +
           motionModelConfiguration.mm6DOFModel.additional_std_XYZ *
               getRandomGenerator().drawGaussian1D_normalized();

       double new_yaw =
           Ayaw1_draw + Ayaw2_draw +
           motionModelConfiguration.mm6DOFModel.additional_std_angle *
               getRandomGenerator().drawGaussian1D_normalized();
       double new_pitch =
           Apitch1_draw + Apitch2_draw +
           motionModelConfiguration.mm6DOFModel.additional_std_angle *
               getRandomGenerator().drawGaussian1D_normalized();
       double new_roll =
           Aroll_draw +
           motionModelConfiguration.mm6DOFModel.additional_std_angle *
               getRandomGenerator().drawGaussian1D_normalized();

       aux->m_particles[i].d.yaw = new_yaw;
       aux->m_particles[i].d.pitch = new_pitch;
       aux->m_particles[i].d.roll = new_roll;
    }

    poseChange.copyFrom(*poseChangeTemp);
  }

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
