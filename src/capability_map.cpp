#include <renbo_grasp_generator/capability_map.h>

namespace renbo_grasp_planning
{

CapabilityMap::CapabilityMap(const std::string& database_file_path):
  nh_("~"),
  base_name_("r_sole"),
  eef_name_("r_gripper"),
  grid_size_(0.05),
  ds_config_file_path_(database_file_path)
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  wb_jmg_ = robot_model_loader_->getModel()->getJointModelGroup("whole_body_fixed");

  wb_joint_names_ = wb_jmg_->getJointModelNames();

  right_arm_torso_jmg_ = robot_model_loader_->getModel()->getJointModelGroup("right_arm_torso");

  right_leg_jmg_ = robot_model_loader_->getModel()->getJointModelGroup("right_leg");

  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_name_, "rviz_visual_markers"));
  visual_tools_->enableBatchPublishing();

  max_manipulability_ = std::numeric_limits<double>::min();

  min_manipulability_ = std::numeric_limits<double>::max();

  ws_bound_.resize(3, 2);
  ws_bound_(0, 0) = -1;
  ws_bound_(1, 0) = -1;
  ws_bound_(2, 0) = 0.15005;
  ws_bound_(0, 1) = 1;
  ws_bound_(1, 1) = 1;
  ws_bound_(2, 1) = 1.9;

  initialize();
}

CapabilityMap::~CapabilityMap()
{

}

void CapabilityMap::initialize()
{
  if (!loadDBConfig())
  {
    ROS_ERROR("CapabilityMap: load database config fail");
  }

  std::string path = ros::package::getPath("renbo_grasp_generator").append("/database/config_voxel_table.dat");
  if (!loadConfigVoxelTable(path))
  {
    ROS_ERROR("CapabilityMap: load config voxel table fail");
  }

  capability_map_ = buildVoxelGridMap();

}

std::vector<double> CapabilityMap::generateNearestConfig(const Eigen::Affine3d& eef_pose)
{
  std::vector<double> nearest_config(wb_jmg_->getVariableCount());
  robot_state::RobotState rstate(robot_model_loader_->getModel());

  int correspond_voxel_index = findEEVoxel(eef_pose);
  std::vector<int> voxel_config_indexs = queryVoxelConfig(correspond_voxel_index);
  if (voxel_config_indexs.size() == 0)
  {
    ROS_ERROR("no configuration in the corresponding voxel");
    return nearest_config;
  }

  double min_dist = std::numeric_limits<double>::max();
  int min_index = 0;
  for (int i = 0; i < voxel_config_indexs.size(); i++)
  {
    rstate.setVariablePositions(wb_joint_names_, db_configs_[voxel_config_indexs[i]]);
    Eigen::Affine3d db_eef_pose = rstate.getGlobalLinkTransform("r_gripper");
    Eigen::Affine3d waist_pose = rstate.getGlobalLinkTransform("waist");

//    double dist;
//    ROS_INFO_STREAM(waist_pose.translation());
//    double dist = computeAffineDist(db_eef_pose, eef_pose);
    Eigen::Matrix3d waist_rot = waist_pose.linear();
    double dist = (waist_pose.translation().x())*(waist_pose.translation().x())+waist_pose.translation().y()*waist_pose.translation().y();

    Eigen::Vector3d waist_euler = waist_rot.eulerAngles(0, 1, 2);
    dist += waist_euler(0)*waist_euler(0)+waist_euler(1)*waist_euler(1)+waist_euler(2)*waist_euler(2);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_index = voxel_config_indexs[i];
    }
  }

  nearest_config = db_configs_[min_index];

  rstate.setVariablePositions(wb_joint_names_, nearest_config);
  bool ik_ret = rstate.setFromIK(right_arm_torso_jmg_, eef_pose, 1, 0);
  if (!ik_ret)
  {
    ROS_INFO("solve ik fail");
  }

  std::vector<double> approx_config(wb_jmg_->getVariableCount());
  rstate.copyJointGroupPositions(wb_jmg_, approx_config);

  return approx_config;
}

bool CapabilityMap::buildCapabilityMap(const std::string& package_path)
{
  bool ret;
  ret = sortEEtoVoxel(package_path);
  return ret;
}

void CapabilityMap::publishTCPPoint()
{
  config sampled_config = sampleConfig();
  Eigen::Affine3d tcp_pose = getTCPPose(sampled_config);

  robot_state::RobotState state(robot_model_loader_->getModel());
  state.setVariablePositions(wb_joint_names_, sampled_config);
  state.update();

  Eigen::MatrixXd jacobian;
  jacobian = computeJacobian(state);

  float manipulability = computeManipulability(jacobian);

  if(manipulability > max_manipulability_)
    max_manipulability_ = manipulability;

  if(manipulability < min_manipulability_)
    min_manipulability_ = manipulability;

//  ROS_INFO_STREAM("manipulability: " << manipulability);

  geometry_msgs::Vector3 scale;
  scale.x = 0.025;
  scale.y = 0.025;
  scale.z = 0.025;
  float level = manipulability/4.3;
  std::vector<float> color_bar = getRGBColorBar(level);

  visual_tools_->publishSphere(tcp_pose, getColor(color_bar[0], color_bar[1], color_bar[2], 1.0), scale);
  visual_tools_->trigger();
}

void CapabilityMap::displayReachabilityMap()
{
  std::vector<float> reachabilities(capability_map_.size());
  robot_state::RobotState state(robot_model_loader_->getModel());

  ros::Time start = ros::Time::now();
  for (int i = 0; i < 40000; i++)
  {
    Eigen::Affine3d ee_pose = getTCPPose(db_configs_[i]);

    int index = findEEVoxel(ee_pose);
    state.setVariablePositions(wb_joint_names_, db_configs_[i]);
    state.update();
    Eigen::MatrixXd jacobian = computeJacobian(state);
    float manipulability = computeManipulability(jacobian);

    if (manipulability > reachabilities[index]) // assign the biggest manipuability index to the grid
      reachabilities[index] = manipulability;
  }

  float max = std::numeric_limits<float>::min();
  float min = std::numeric_limits<float>::max();
  std::vector<double> level;

  for (int i = 0; i < reachabilities.size(); i++)
  {
    if (reachabilities[i] == 0)
      continue;

    if (reachabilities[i] > max)
      max = reachabilities[i];
    if (reachabilities[i] < min)
      min = reachabilities[i];
  }

  ros::Duration duration = ros::Time::now() - start;

  geometry_msgs::Vector3 scale;
  scale.x = grid_size_;
  scale.y = grid_size_;
  scale.z = grid_size_;

  double interval = max - min;
  for (int i = 0; i < reachabilities.size(); i++)
  {
    level.push_back((reachabilities[i]-min)/interval);
  }

  for (int i = 1; i < ws_x_length_; i++)
  {
    for (int j = 30; j < 50; j++)
    {
      for (int k = 0; k < ws_z_length_; k++)
      {
        int index = XYZtoIndex(i, j, k);

        if (reachabilities[index] == 0)
          continue;

        Eigen::Affine3d pose;
        pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
        pose.translation() << capability_map_[index](0), capability_map_[index](1), capability_map_[index](2);

        std::vector<float> color_bar = getRGBColorBar(level[index]);
        visual_tools_->publishSphere(pose, getColor(color_bar[0], color_bar[1], color_bar[2], 1.0), scale);
      }
    }
  }

//  for (int i = XYZtoIndex(8, 0, 0); i < XYZtoIndex(12, ws_y_length_, ws_z_length_); i++)
//  {
//    if (reachabilities[i] == 0)
//      continue;

//    Eigen::Affine3d pose;
//    pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
//    pose.translation() << capability_map_[i](0), capability_map_[i](1), capability_map_[i](2);

//    std::vector<float> color_bar = getRGBColorBar(level[i]);
//    visual_tools_->publishSphere(pose, getColor(color_bar[0], color_bar[1], color_bar[2], 1.0), scale);
//  }

  visual_tools_->trigger();
  ros::Duration(1.0);

  ROS_INFO_STREAM("maximum node contains " << max << " minimum node contains " << min);
  ROS_INFO_STREAM("process cost " << duration << " secs");

}

void CapabilityMap::displayVoxelGridMap()
{
  EigenSTL::vector_Vector3d capability_map = buildVoxelGridMap();
  ROS_INFO_STREAM("voxel size " << capability_map.size());

  for (int i = 0; i < capability_map.size(); i++)
  {
    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    pose.translation() << capability_map[i](0), capability_map[i](1), capability_map[i](2);

    visual_tools_->publishWireframeCuboid(pose, grid_size_, grid_size_, grid_size_, rviz_visual_tools::RED);
  }

  visual_tools_->trigger();

  config sampled_config = sampleConfig();
  Eigen::Affine3d tcp_pose = getTCPPose(sampled_config);

  ROS_INFO_STREAM("eef pose:\n" << tcp_pose.translation());

  int ee_voxel_index = findEEVoxel(tcp_pose);
  Eigen::Vector3d ee_voxel = capability_map_[ee_voxel_index];
  Eigen::Affine3d pose_voxel;
  pose_voxel = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  pose_voxel.translation() << ee_voxel(0), ee_voxel(1), ee_voxel(2);

  geometry_msgs::Vector3 scale;
  scale.x = grid_size_;
  scale.y = grid_size_;
  scale.z = grid_size_;
  visual_tools_->publishSphere(tcp_pose, getColor(0.0, 255.0, 0.0, 1.0), scale);
  visual_tools_->publishWireframeCuboid(pose_voxel, grid_size_, grid_size_, grid_size_, rviz_visual_tools::LIME_GREEN);
  visual_tools_->trigger();

}

void CapabilityMap::displayConfigNumMap()
{
  std::vector<int> config_count(capability_map_.size());

  ros::Time start = ros::Time::now();
  for (int i = 0; i < 40000; i++)
  {
    Eigen::Affine3d ee_pose = getTCPPose(db_configs_[i]);

    if (ee_pose.translation().y() > -0.25 || ee_pose.translation().y() < -0.3)
      continue;

    int index = findEEVoxel(ee_pose);
//    Eigen::Vector3d grid = capability_map_[index];
    config_count[index]++;

    if (i%1000 == 0)
      ROS_INFO_STREAM("current count: " << i);
  }

  int max = 0;
  int min = 100000;

  for (int i = 0; i < config_count.size(); i++)
  {
//    if (config_count[i] == 0)
//      continue;

    if (config_count[i] > max)
      max = config_count[i];
    if (config_count[i] < min)
      min = config_count[i];
  }

  ros::Duration duration = ros::Time::now() - start;

  double interval = max - min;
//  ROS_INFO_STREAM("interval " << interval);
  geometry_msgs::Vector3 scale;
  scale.x = 0.05;
  scale.y = 0.05;
  scale.z = 0.05;
  for (int i = 0; i < config_count.size(); i++)
  {
    if (config_count[i] == 0)
      continue;

    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    pose.translation() << capability_map_[i](0), capability_map_[i](1), capability_map_[i](2);

    double level = (config_count[i]-min)/interval;
//    ROS_INFO_STREAM("level " << level);
    std::vector<float> color_bar = getRGBColorBar(level);

    if (level <= 0.5)
    {
      visual_tools_->publishSphere(pose, getColor(color_bar[0], color_bar[1], color_bar[2], 1.0), scale);
    }
    else
    {
      visual_tools_->publishSphere(pose, getColor(color_bar[0], color_bar[1], color_bar[2], 1.0), scale);
    }
  }

  visual_tools_->trigger();
  ros::Duration(2.0);

  ROS_INFO_STREAM("maximum node contains " << max << " minimum node contains " << min);
  ROS_INFO_STREAM("process cost " << duration << " secs");

}

void CapabilityMap::displayColorLevel(int intervel[])
{
  geometry_msgs::Vector3 scale;
  scale.x = 0.01;
  scale.y = 0.01;
  scale.z = 0.01;

  Eigen::Affine3d pose;
  for (int i = 0; i < 1000; i++)
  {
    float level = (float)i/1000.0;
    std::vector<float> color_bar = getRGBColorBar(level);
    pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    pose.translation().x() = i*0.001;

    visual_tools_->publishSphere(pose, getColor(color_bar[0], color_bar[1], color_bar[2], 1.0), scale);
  }

  visual_tools_->trigger();

  ros::Duration(0.5).sleep();
}



int CapabilityMap::findEEVoxel(const Eigen::Affine3d &eef_pose)
{
  int x = std::ceil((eef_pose.translation().x()-ws_bound_(0, 0))/grid_size_);
  int y = std::ceil((eef_pose.translation().y()-ws_bound_(1, 0))/grid_size_);
  int z = std::ceil((eef_pose.translation().z()-ws_bound_(2, 0))/grid_size_);

  int index = (x-1)*(ws_z_length_)*(ws_y_length_)+ (ws_z_length_)*(y-1)+z-1;

  bool verbose = false;
  if (verbose)
  {
    ROS_INFO_STREAM("eef grid coordinate: (" << x << "," << y << "," << z << ")");
    ROS_INFO_STREAM("legnth of workspace: x=" << ws_x_length_ << " y=" << ws_y_length_ << " z=" << ws_z_length_);
    ROS_INFO_STREAM("result index: " << index);
  }

  return index;
}

bool CapabilityMap::sortEEtoVoxel(std::string database_path)
{
  robot_state::RobotState rstate(robot_model_loader_->getModel());
  rstate.setToDefaultValues();
  rstate.update();

  std::vector< std::vector<int> > config_voxel_table;
  config_voxel_table.resize((ws_x_length_+1)*(ws_y_length_+1)*(ws_z_length_+1));

  if (db_configs_.size() == 0)
  {
    ROS_ERROR("double support config not loaded");
    return false;
  }

  for (int i = 0; i < db_configs_.size()-1; i++)
  {
    rstate.setVariablePositions(wb_joint_names_, db_configs_[i]);
    Eigen::Affine3d eef_pose = rstate.getGlobalLinkTransform(eef_name_);
    int voxel_index = findEEVoxel(eef_pose);
    config_voxel_table[voxel_index].push_back(i);
  }

  ROS_INFO_STREAM(database_path);
  std::ofstream output_file(database_path.c_str(), std::ios_base::out|std::ios_base::trunc);
  if (!output_file)
  {
    ROS_ERROR("open config voxel table fail");
    return false;
  }

  for (int i = 0; i < config_voxel_table.size(); i ++)
  {
    for (int j = 0; j < config_voxel_table[i].size(); j++)
    {
      output_file << config_voxel_table[i][j] << " ";
    }
    output_file << "\n";
  }
  output_file.close();
  return true;
}

std::vector<int> CapabilityMap::queryVoxelConfig(const int& voxel_index) const
{
  return config_voxel_table_[voxel_index];
}

EigenSTL::vector_Vector3d CapabilityMap::buildVoxelGridMap()
{
  EigenSTL::vector_Vector3d map;
  double ws_x =  ws_bound_(0, 1) - ws_bound_(0, 0);
  double ws_y =  ws_bound_(1, 1) - ws_bound_(1, 0);
  double ws_z =  ws_bound_(2, 1) - ws_bound_(2, 0);

  ws_x_length_ = std::floor(ws_x/grid_size_);
  ws_y_length_ = std::floor(ws_y/grid_size_);
  ws_z_length_ = std::floor(ws_z/grid_size_);

  Eigen::Vector3d voxel;
  for (int i = 0; i < ws_x_length_; i++)
  {
    for (int j = 0; j < ws_y_length_; j++)
    {
      for (int k = 0; k < ws_z_length_; k++)
      {
        voxel << ws_bound_(0, 0)+i*grid_size_+grid_size_/2, ws_bound_(1, 0)+j*grid_size_+grid_size_/2, k*grid_size_+(grid_size_/2)+0.15505;
        map.push_back(voxel);
      }
    }
  }

  return map;
}

float CapabilityMap::computeManipulability(Eigen::MatrixXd jacobian)
{
  jacobian *= jacobian.transpose();
  float manipulability = std::sqrt(jacobian.determinant());

  return manipulability;
}


Eigen::MatrixXd CapabilityMap::computeJacobian(const robot_state::RobotState state)
{
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd upper_jacobian, lower_jacobian;

  state.getJacobian(right_arm_torso_jmg_, state.getLinkModel(right_arm_torso_jmg_->getLinkModelNames().back()),
                    reference_point_position, upper_jacobian);

  state.getJacobian(right_leg_jmg_, state.getLinkModel("waist"),
                    reference_point_position, lower_jacobian);

//  ROS_INFO_STREAM("upper body Jacobian:\n " << upper_jacobian);
//  ROS_INFO_STREAM("lower body Jacobian:\n " << lower_jacobian);

  Eigen::MatrixXd jacaobian(lower_jacobian.rows(), lower_jacobian.cols()+upper_jacobian.cols());
  jacaobian << lower_jacobian, upper_jacobian;

  return jacaobian;
}

double CapabilityMap::computeAffineDist(Eigen::Affine3d affine_a, Eigen::Affine3d affine_b)
{
  double dist_square = 0.0;
  double dist_norm = 0.0;

  Eigen::Vector3d trans_a = affine_a.translation();
  Eigen::Vector3d trans_b = affine_b.translation();
  dist_square += (trans_a(0)-trans_b(0))*(trans_a(0)-trans_b(0));
  dist_square += (trans_a(1)-trans_b(1))*(trans_a(1)-trans_b(1));
  dist_square += (trans_a(2)-trans_b(2))*(trans_a(2)-trans_b(2));

  Eigen::Matrix3d rot_a = affine_a.linear();
  Eigen::Matrix3d rot_b = affine_b.linear();

  Eigen::Vector3d eular_a = rot_a.eulerAngles(0, 1, 2);
  Eigen::Vector3d eular_b = rot_b.eulerAngles(0, 1, 2);

  dist_square += (eular_a(0)-eular_b(0))*(eular_a(0)-eular_b(0));
  dist_square += (eular_a(1)-eular_b(1))*(eular_a(1)-eular_b(1));
  dist_square += (eular_a(2)-eular_b(2))*(eular_a(2)-eular_b(2));

  dist_norm = std::sqrt(dist_square);
  return dist_norm;
}

int CapabilityMap::XYZtoIndex(int x, int y, int z)
{
  int index = (x-1)*(ws_z_length_)*(ws_y_length_)+ (ws_z_length_)*(y-1)+z-1;
  return index;
}

Eigen::Affine3d CapabilityMap::getTCPPose(const config& config_)
{
  robot_state::RobotState state(robot_model_loader_->getModel());
  state.setVariablePositions(wb_joint_names_, config_);

  Eigen::Affine3d eef_pose;
  eef_pose = state.getGlobalLinkTransform("r_gripper");

  return eef_pose;
}

config CapabilityMap::sampleConfig()
{
  config rnd_config;

  int lb = 0, ub = db_configs_.size() - 1;
  int rnd_idx = floor(rng_.uniformReal(lb, ub));

  rnd_config = db_configs_[rnd_idx];

  return rnd_config;
}

void CapabilityMap::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = segment->second.segment.getName();
  const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;

  for (unsigned int i = 0; i < children.size(); i++)
  {
    const KDL::Segment& child = children[i]->second.segment;
    robot_state_publisher::SegmentPair s(children[i]->second.segment, root, child.getName());

    if (child.getJoint().getType() == KDL::Joint::None)
    {
      // skip over fixed:
      //      segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Tree initialization: Skipping fixed segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    else
    {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Tree initialization: Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}

bool CapabilityMap::loadDBConfig()
{
  std::string line;
  std::vector<double> config;
  double q_in;

  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_CYAN << "capability map: loading double support database..." << MOVEIT_CONSOLE_COLOR_RESET);

  std::ifstream db_config_file;
  db_config_file.open(ds_config_file_path_.c_str());
  if (!db_config_file)
  {
    ROS_ERROR("capability map: open database fail");
    return false;
  }

  db_configs_.clear();

  ros::Time start = ros::Time::now();
  int db_config_count = 0;
  while (db_config_file.good())
  {
    std::getline(db_config_file, line);
    std::istringstream in(line);

    while (in >> q_in)
    {
      config.push_back(q_in);
    }

    db_configs_.push_back(config);
    config.clear();

    db_config_count++;

    if (db_config_file.eof())
    {
      break;
    }
  }

  ros::Time end = ros::Time::now();
  ros::Duration dura = end - start;

  ROS_INFO_STREAM_NAMED("capability map", "Load total " << db_config_count << " configs" << " and cost " << dura << " sec");

  db_config_file.close();
  return true;
}

bool CapabilityMap::loadConfigVoxelTable(const std::string& file_path)
{
  std::ifstream table_file(file_path.c_str());
  if (!table_file)
  {
    ROS_ERROR("capability map: load config voxel table fail");
    return false;
  }

  config_voxel_table_.clear();
  std::string line;
  std::vector<int> voxel_indexs;
  while (table_file.good())
  {
    std::getline(table_file, line);
    std::istringstream in(line);
    int index;
    while (in >> index)
    {
      voxel_indexs.push_back(index);
    }

    config_voxel_table_.push_back(voxel_indexs);
    voxel_indexs.clear();

    if (table_file.eof())
    {
      break;
    }
  }

  table_file.close();
  return true;
}

std_msgs::ColorRGBA CapabilityMap::getColor(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;

  color.r = r/255;
  color.g = g/255;
  color.b = b/255;
  color.a = a;

  return color;
}

std::vector<float> CapabilityMap::getRGBColorBar(float level)
{
  std::vector<float> color_bar(3);

  if (level < 0.5)
  {
    if (level < 0.25)
    {
      color_bar[0] = 0.0;
      color_bar[1] = 4*level*255.0;
      color_bar[2] = 255.0;
    }
    else
    {
      color_bar[0] = 0.0;
      color_bar[1] = 255.0;
      color_bar[2] = 4*(0.5-level)*255.0;
    }
  }
  else
  {
    if (level < 0.75)
    {
      color_bar[0] = 4*(level-0.5)*255.0;
      color_bar[1] = 255.0;
      color_bar[2] = 0.0;
    }
    else
    {
      color_bar[0] = 255.0;
      color_bar[1] = 4*(1.0-level)*255.0;
      color_bar[2] = 0.0;
    }
  }

  return color_bar;
}

} // end namespace
