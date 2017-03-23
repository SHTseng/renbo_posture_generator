#include <renbo_grasp_generator/capability_map.h>

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "build_capability_map");

  std::string data_file_path = ros::package::getPath("renbo_whole_body_plan");
  data_file_path.append("/database/ds_dataset.dat");
  std::string table_file_path = ros::package::getPath("renbo_grasp_generator");
  table_file_path.append("/database/config_voxel_table.dat");

  std::shared_ptr<renbo_grasp_planning::CapabilityMap> cm_;
  cm_ = std::shared_ptr<renbo_grasp_planning::CapabilityMap>(new renbo_grasp_planning::CapabilityMap(data_file_path));

  bool ret = cm_->buildCapabilityMap(table_file_path);
  if (!ret)
  {
    ROS_ERROR("build capability map fail");
  }

  ros::shutdown();
  return 0;
}
