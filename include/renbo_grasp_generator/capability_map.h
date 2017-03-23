#ifndef CAPABILITY_MAP_H_
#define CAPABILITY_MAP_H_

#include <ros/package.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/macros/console_colors.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <robot_state_publisher/robot_state_publisher.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <fstream>

namespace renbo_grasp_planning
{

typedef std::vector<double> config;

class CapabilityMap
{
public:

  CapabilityMap(const std::string& database_file_path);

  ~CapabilityMap();

  std::vector<double> generateNearestConfig(const Eigen::Affine3d& eef_pose);

  bool buildCapabilityMap(const std::string& package_path);

  void publishTCPPoint();

  void displayReachabilityMap();

  void displayVoxelGridMap();

  void displayConfigNumMap();

  void displayColorLevel(int intervel[]);

private:

  void initialize();

  bool sortEEtoVoxel(std::string database_path);

  int findEEVoxel(const Eigen::Affine3d& eef_pose);

  std::vector<int> queryVoxelConfig(const int& voxel_index) const;

  std::vector< std::vector<int>> readConfigVoxelTable() const;

  EigenSTL::vector_Vector3d buildVoxelGridMap();

  float computeManipulability(Eigen::MatrixXd jacobian);

  Eigen::MatrixXd computeJacobian(const robot_state::RobotState state);

  double computeAffineDist(Eigen::Affine3d affine_a, Eigen::Affine3d affine_b);

  void voxelizeWorkSpace();

  int XYZtoIndex(int x, int y, int z);

  Eigen::Affine3d getTCPPose(const config& config_);

  config sampleConfig();

  void addChildren(const KDL::SegmentMap::const_iterator segment);

  bool loadDBConfig();

  bool loadConfigVoxelTable(const std::string& file_path);

  std_msgs::ColorRGBA getColor(float r, float g, float b, float a);

  std::vector<float> getRGBColorBar(float level);

  ros::NodeHandle nh_;

  EigenSTL::vector_Vector3d capability_map_;

  Eigen::MatrixXd ws_bound_;

  double grid_size_;

  int ws_x_length_;

  int ws_y_length_;

  int ws_z_length_;

  float max_manipulability_;

  float min_manipulability_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  const robot_state::JointModelGroup* wb_jmg_;

  const robot_state::JointModelGroup* right_arm_torso_jmg_;

  const robot_state::JointModelGroup* right_leg_jmg_;

  std::vector<std::string> wb_joint_names_;

  boost::shared_ptr<const urdf::ModelInterface> urdf_;

  KDL::Tree kdl_tree_;

  KDL::Chain kdl_chain_right_;

  KDL::Chain kdl_chain_left_;

  std::map<std::string, robot_state_publisher::SegmentPair> segments_;

  std::vector< std::vector<double> > db_configs_;

  std::vector< std::vector<int>> config_voxel_table_;

  std::string ds_config_file_path_;

  std::string voxel_config_file_path_;

  std::string base_name_;

  std::string eef_name_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  random_numbers::RandomNumberGenerator rng_;

};

}



#endif
