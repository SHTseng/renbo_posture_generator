#ifndef POSTURE_GENERATOR_H
#define POSTURE_GENERATOR_H

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>

#include <renbo_msgs/generate_whole_body_posture.h>

#include <renbo_grasp_generator/capability_map.h>

namespace posture_generator
{
class PostureGenerator
{
public:

  PostureGenerator(const std::string& file_path);

  ~PostureGenerator();

  bool generate_whole_body_posture(renbo_msgs::generate_whole_body_posture::Request &req,
                                   renbo_msgs::generate_whole_body_posture::Response &res);
private:

  ros::NodeHandle nh_;

  ros::Publisher rstate_pub_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  const robot_state::JointModelGroup* wb_jmg_;

  std::vector<std::string> wb_joint_names_;

  std::shared_ptr<renbo_grasp_planning::CapabilityMap> cm_;
};
}

#endif // POSTURE_GENERATOR_H
