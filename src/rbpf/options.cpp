/*
 * File: options.cpp
 * Author: Vladislav Tananaev
 */

#include "rbpf/options.h"
#include <set>

namespace mrpt_rbpf_slam
{
namespace
{
using namespace mrpt::obs;

bool load6DOFModelParameters(const ros::NodeHandle& nh,
                                 CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel& sixdof_model)
{
  bool success = true;
  ros::NodeHandle sub_nh(nh, "6dof_motion_model_options");
  auto getParam = [&success, &sub_nh](const std::string param, auto& val) {
    success = success && sub_nh.getParam(param, val);
  };

  getParam("a1", sixdof_model.a1);
  getParam("a2", sixdof_model.a2);
  getParam("a3", sixdof_model.a3);
  getParam("a4", sixdof_model.a4);
  getParam("a5", sixdof_model.a5);
  getParam("a6", sixdof_model.a6);
  getParam("a7", sixdof_model.a7);
  getParam("a8", sixdof_model.a8);
  getParam("a9", sixdof_model.a9);
  getParam("a10", sixdof_model.a10);
  getParam("additional_std_XYZ", sixdof_model.additional_std_XYZ);
  getParam("additional_std_angle", sixdof_model.additional_std_angle);
  return success;
}

bool loadMotionModelParameters(const ros::NodeHandle& nh,
                               CActionRobotMovement3D::TMotionModelOptions& motion_model_options)
{
  static const std::map<std::string, CActionRobotMovement3D::TDrawSampleMotionModel> motion_models = {
    { "gaussian", CActionRobotMovement3D::mmGaussian },
    { "6dof", CActionRobotMovement3D::mm6DOF }
  };

  bool success = true;
  ros::NodeHandle sub_nh(nh, "motion_model");
  auto getParam = [&success, &sub_nh](const std::string param, auto& val) {
    success = success && sub_nh.getParam(param, val);
  };

  std::string model_type;
  getParam("type", model_type);
  if (!motion_models.count(model_type))
  {
    ROS_ERROR_STREAM("Specified motion model " << model_type << " is not supported.");
    return false;
  }
  motion_model_options.modelSelection = motion_models.at(model_type);
  success = success && load6DOFModelParameters(sub_nh, motion_model_options.mm6DOFModel);
  return success;
}

bool loadVisualizationOptions(const ros::NodeHandle& nh, PFslam::Options& options)
{
  bool success = true;
  ros::NodeHandle sub_nh(nh, "mrpt_visualization_options");
  success = success && sub_nh.getParam("width", options.PROGRESS_WINDOW_WIDTH_);
  success = success && sub_nh.getParam("height", options.PROGRESS_WINDOW_HEIGHT_);
  success = success && sub_nh.getParam("window_update_delay", options.SHOW_PROGRESS_IN_WINDOW_DELAY_MS_);
  success = success && sub_nh.getParam("show_window", options.SHOW_PROGRESS_IN_WINDOW_);
  success = success && sub_nh.getParam("camera_follow_robot", options.CAMERA_3DSCENE_FOLLOWS_ROBOT_);
  return success;
}
}  // namespace

bool loadOptions(const ros::NodeHandle& nh, PFslam::Options& options)
{
  bool success = true;
  success = success && loadMotionModelParameters(nh, options.motion_model_options_);
  success = success && loadVisualizationOptions(nh, options);
  success = success && nh.getParam("simplemap_save_folder", options.simplemap_path_prefix);
  return success;
}

}  // namespace mrpt_rbpf_slam
