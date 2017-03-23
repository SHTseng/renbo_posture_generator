#include <ros/ros.h>

#include <renbo_grasp_generator/posture_generator.h>

using namespace posture_generator;

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "posture_generator_node");

  ros::NodeHandle nh;

  std::string data_file_path = ros::package::getPath("renbo_whole_body_plan");
  data_file_path.append("/database/ds_dataset.dat");

  PostureGenerator posture_generator_(data_file_path);
  ros::ServiceServer generate_whole_body_posture = nh.advertiseService("capability_map/generate_wb_posture",
                                                                       &PostureGenerator::generate_whole_body_posture, &posture_generator_);

  ros::spin();

  return 0;
}
