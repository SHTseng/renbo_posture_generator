#ifndef GRASP_PLANNING_H_
#define GRASP_PLANNING_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include "geometric_shapes/shape_operations.h"

#include <shape_msgs/Mesh.h>

#include <eigen_conversions/eigen_msg.h>

namespace renbo_grasp_planning
{
class GraspPlanning
{
public:

  GraspPlanning();

  ~GraspPlanning();

  void displayGripper();

private:

  shape_msgs::Mesh importMesh();

  std_msgs::ColorRGBA getColor(float r, float g, float b, float a);

  ros::NodeHandle nh_;

  ros::Publisher gripper_marker_pub_;
};

}

#endif
