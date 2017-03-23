#include <ros/ros.h>
#include <ros/package.h>

#include <string>

#include <visualization_msgs/Marker.h>

#include <eigen_conversions/eigen_msg.h>

#include <renbo_grasp_generator/grasp_planning.h>

std_msgs::ColorRGBA getColor(float r, float g, float b, float a);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "renbo_grasp");

  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base","/rviz_visual_markers"));

  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
  ros::Duration(0.5).sleep();

  Eigen::Affine3d gripper_pose;
  gripper_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
  visual_tools_->publishMesh(gripper_pose, "package://renbo_description/meshes/arm/r_gripper.stl", rviz_visual_tools::WHITE, rviz_visual_tools::XXXXSMALL);
  visual_tools_->trigger();

  ros::Duration(1.0).sleep();

//  Eigen::Affine3d plane_pose;
//  plane_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
//  plane_pose.translation() = Eigen::Vector3d(0.0, 0.2, 0.0);

//  std_msgs::ColorRGBA plane_color;
//  plane_color.r = 0.0;
//  plane_color.g = 1.0;
//  plane_color.b = 0.5;
//  plane_color.a = 0.5;
//  visual_tools_->publishXYPlane(plane_pose, rviz_visual_tools::YELLOW, 0.05);
//  visual_tools_->trigger();

//  ros::Duration(1.0).sleep();

//  Eigen::Affine3d cuboid_pose;
//  cuboid_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
//  cuboid_pose.translation() = Eigen::Vector3d(0.2, 0.2, 0.06);

//  visual_tools_->publishCuboid(cuboid_pose, 0.05, 0.05, 0.12);

  Eigen::Affine3d cylinder_pose;
  cylinder_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  cylinder_pose.translation() = Eigen::Vector3d(0.2, 0.4, 0.06);

  geometry_msgs::Pose cylinder_pose_msgs;
  tf::poseEigenToMsg(cylinder_pose, cylinder_pose_msgs);

  visual_tools_->publishCylinder(cylinder_pose_msgs, getColor(176.0, 196.0, 222.0, 0.8), 0.12, 0.05);

  Eigen::Affine3d grid_pose;
  grid_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  for (int i = 0; i < 5; i++)
  {
    grid_pose.translation() = Eigen::Vector3d(0.4+i*0.02, 0.2, 0.005);
    visual_tools_->publishWireframeCuboid(grid_pose, 0.02, 0.02, 0.02, rviz_visual_tools::LIME_GREEN);
    visual_tools_->trigger();
  }

  Eigen::Affine3d cuboid_plane_pose;
  cuboid_plane_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  cuboid_plane_pose.translation() = Eigen::Vector3d(0.166, 0.0258, 0.0);
  visual_tools_->publishCuboid(cuboid_plane_pose, 0.029, 0.0001, 0.019, getColor(0.0, 191.0, 255.0, 1.0));

//  cuboid_plane_pose.translation() = Eigen::Vector3d(0.166, 0.0258, 0.05);
//  visual_tools_->publishCuboid(cuboid_plane_pose, 0.029, 0.0001, 0.019, getColor(0.0, 191.0, 255.0, 0.5));

  cuboid_plane_pose.translation() = Eigen::Vector3d(0.181, 0.026, 0.0);
  visual_tools_->publishCuboid(cuboid_plane_pose, 0.059, 0.0001, 0.05, getColor(205.0, 201.0, 201.0, 0.5));

  cuboid_plane_pose.translation() = Eigen::Vector3d(0.166, -0.0258, 0.0);
  visual_tools_->publishCuboid(cuboid_plane_pose, 0.029, 0.0001, 0.019, getColor(0.0, 191.0, 255.0, 1.0));

//  cuboid_plane_pose.translation() = Eigen::Vector3d(0.166, -0.0258, 0.05);
//  visual_tools_->publishCuboid(cuboid_plane_pose, 0.029, 0.0001, 0.019, getColor(0.0, 191.0, 255.0, 0.5));

  cuboid_plane_pose.translation() = Eigen::Vector3d(0.181, -0.026, 0.0);
  visual_tools_->publishCuboid(cuboid_plane_pose, 0.059, 0.0001, 0.05, getColor(205.0, 201.0, 201.0, 0.5));

//  cuboid_plane_pose.translation() = Eigen::Vector3d(0.122, 0.0, 0.0);
//  visual_tools_->publishCuboid(cuboid_plane_pose, 0.0001, 0.035, 0.033, getColor(191.0, 239.0, 255.0, 1.0));
  Eigen::Affine3d tcp_pose;
  tcp_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  tcp_pose.translation() = Eigen::Vector3d(0.166, 0.0, 0.0);
  visual_tools_->publishSphere(tcp_pose, rviz_visual_tools::RED, rviz_visual_tools::XXSMALL);

  visual_tools_->trigger();

  ros::shutdown();

  return 0;
}

std_msgs::ColorRGBA getColor(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;
  color.r = r/255.0;
  color.g = g/255.0;
  color.b = b/255.0;
  color.a = a;

  return color;
}

