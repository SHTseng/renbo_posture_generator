#include <ros/ros.h>
#include <ros/package.h>

#include <renbo_grasp_generator/capability_map.h>

using namespace renbo_grasp_planning;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_capability_map");

  std::string file_path = ros::package::getPath("renbo_whole_body_plan");
  file_path.append("/database/ds_dataset.dat");

  std::shared_ptr<renbo_grasp_planning::CapabilityMap> cm_;
  cm_ = std::shared_ptr<renbo_grasp_planning::CapabilityMap>(new renbo_grasp_planning::CapabilityMap(file_path));

  int interval[] = {0, 100};


  ros::shutdown();

  return 0;
}
