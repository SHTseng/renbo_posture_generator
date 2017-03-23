#include <renbo_grasp_generator/posture_generator.h>

#include <eigen_conversions/eigen_msg.h>

namespace posture_generator
{

PostureGenerator::PostureGenerator(const std::string& file_path):
  nh_("~")
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  wb_jmg_ = robot_model_loader_->getModel()->getJointModelGroup("whole_body_fixed");

  wb_joint_names_ = wb_jmg_->getJointModelNames();

  cm_ = std::shared_ptr<renbo_grasp_planning::CapabilityMap>(new renbo_grasp_planning::CapabilityMap(file_path));

  rstate_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("renbo_robot_state", 1);
}

PostureGenerator::~PostureGenerator()
{

}

bool PostureGenerator::generate_whole_body_posture(renbo_msgs::generate_whole_body_posture::Request& req,
                                                   renbo_msgs::generate_whole_body_posture::Response& res)
{
  Eigen::Affine3d eigen_eef_pose;
  tf::poseMsgToEigen(req.right_eef_pose, eigen_eef_pose);

  std::vector<double> wb_config = cm_->generateNearestConfig(eigen_eef_pose);

  robot_state::RobotState rstate(robot_model_loader_->getModel());
  rstate.setVariablePositions(wb_joint_names_, wb_config);

  moveit_msgs::DisplayRobotState rstate_msg;
  robot_state::robotStateToRobotStateMsg(rstate, rstate_msg.state);
  rstate_pub_.publish(rstate_msg);

  res.solved_config = wb_config;
  res.success = 1;

  return true;
}

}
