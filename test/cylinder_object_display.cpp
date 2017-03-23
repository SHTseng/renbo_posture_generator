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

  float grid_size = 0.02;
  Eigen::Vector3d object_size(0.04, 0.04, 0.08);

  Eigen::Affine3d gripper_pose;
  gripper_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
  gripper_pose.translation() = Eigen::Vector3d(-0.155, 0.02, 0.01+grid_size);
  visual_tools_->publishMesh(gripper_pose, "package://renbo_description/meshes/arm/r_gripper.stl", rviz_visual_tools::WHITE, rviz_visual_tools::XXXXSMALL);

  gripper_pose *= Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY());
  gripper_pose.translation() = Eigen::Vector3d(0*std::cos(-M_PI/4)-grid_size*std::sin(-M_PI/4),
                                               0*std::sin(-M_PI/4)+grid_size*std::cos(-M_PI/4),
                                               0.156+4*grid_size);
  visual_tools_->publishMesh(gripper_pose, "package://renbo_description/meshes/arm/r_gripper.stl", rviz_visual_tools::WHITE, rviz_visual_tools::XXXXSMALL);

  visual_tools_->trigger();
  ros::Duration(1.0).sleep();


  Eigen::Affine3d cylinder_pose;
  cylinder_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  cylinder_pose.translation() = Eigen::Vector3d(0.02, 0.02, 0.04);

  geometry_msgs::Pose cylinder_pose_msgs;
  tf::poseEigenToMsg(cylinder_pose, cylinder_pose_msgs);

  visual_tools_->publishCylinder(cylinder_pose_msgs, getColor(176.0, 196.0, 222.0, 0.8), 0.08, 0.04);

  Eigen::Affine3d cuboid_pose;
  cuboid_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  cuboid_pose.translation() = Eigen::Vector3d(object_size(0)/2+0.3, object_size(1)/2, object_size(2)/2);

//  visual_tools_->publishCuboid(cuboid_pose, object_size(0), object_size(1), object_size(2), getColor(176.0, 196.0, 222.0, 0.7));

  Eigen::Affine3d grid_pose;
  grid_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  EigenSTL::vector_Vector3d grid_poses;
  Eigen::Vector3d grid_pose_vec;
  for (int k = 0; k < object_size(2)/grid_size-1; k++)
  {
    for (int i = 0; i < object_size(1)/grid_size-1; i++)
    {
      for (int j = 0; j < object_size(0)/grid_size-1; j++)
      {
        grid_pose_vec = Eigen::Vector3d(grid_size/2 + i*grid_size, grid_size/2 + j*grid_size, grid_size/2 + k*grid_size);
        grid_poses.push_back(grid_pose_vec);
        grid_pose.translation() = grid_pose_vec;
        visual_tools_->publishWireframeCuboid(grid_pose, grid_size, grid_size, grid_size, rviz_visual_tools::LIME_GREEN);
      }
    }
  }

  for (int k = 0; k < object_size(2)/grid_size-1; k++)
  {
    for (int i = 0; i < object_size(1)/grid_size-1; i++)
    {
      for (int j = 0; j < object_size(0)/grid_size-1; j++)
      {
        grid_pose_vec = Eigen::Vector3d((grid_size/2 + i*grid_size)*std::cos(M_PI/4)-(grid_size/2 + j*grid_size)*std::sin(M_PI/4)+0.02,
                                        (grid_size/2 + i*grid_size)*std::sin(M_PI/4)+(grid_size/2 + j*grid_size)*std::cos(M_PI/4)-0.0077,
                                        grid_size/2 + k*grid_size); //+0.02/std::sqrt(2)
        grid_poses.push_back(grid_pose_vec);
        grid_pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ());
        grid_pose.translation() = grid_pose_vec;
        visual_tools_->publishWireframeCuboid(grid_pose, grid_size, grid_size, grid_size, rviz_visual_tools::LIME_GREEN);
      }
    }
  }


  Eigen::Affine3d cuboid_plane_pose;
  cuboid_plane_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  cuboid_plane_pose.translation() = Eigen::Vector3d(0.166, 0.0258, 0.0);
//  visual_tools_->publishCuboid(cuboid_plane_pose, 0.029, 0.0001, 0.019, getColor(0.0, 191.s0, 255.0, 1.0));

  cuboid_plane_pose.translation() = Eigen::Vector3d(0.166, -0.0258, 0.0);
//  visual_tools_->publishCuboid(cuboid_plane_pose, 0.029, 0.0001, 0.019, getColor(0.0, 191.0, 255.0, 1.0));

  visual_tools_->trigger();


  Eigen::Vector3d tmp = grid_poses[0];
  cuboid_plane_pose.translation() = Eigen::Vector3d(tmp(0), tmp(1) - grid_size/2-0.001, tmp(2)+grid_size);
  visual_tools_->publishCuboid(cuboid_plane_pose, 0.029, 0.0001, 0.019, getColor(0.0, 191.0, 255.0, 1.0));
  cuboid_plane_pose.translation() = Eigen::Vector3d(tmp(0), tmp(1) + 3*grid_size/2+0.001, tmp(2)+grid_size);
  visual_tools_->publishCuboid(cuboid_plane_pose, 0.029, 0.0001, 0.019, getColor(0.0, 191.0, 255.0, 1.0));

  cuboid_plane_pose.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()));
  cuboid_plane_pose.translation() = Eigen::Vector3d((tmp(0)+grid_size/2)*std::cos(M_PI/4)- (tmp(1) - grid_size/2-0.001-grid_size)*std::sin(M_PI/4),
                                                    (tmp(0)+grid_size/2)*std::sin(M_PI/4)+ (tmp(1) - grid_size/2-0.001-grid_size)*std::cos(M_PI/4),
                                                    tmp(2)+3*grid_size);
  visual_tools_->publishCuboid(cuboid_plane_pose, 0.019, 0.0001, 0.029, getColor(0.0, 191.0, 255.0, 1.0));

  cuboid_plane_pose.translation() = Eigen::Vector3d((tmp(0)+grid_size/2)*std::cos(M_PI/4)-(3*grid_size/2+0.001-grid_size/2)*std::sin(M_PI/4),
                                                    (tmp(0)+grid_size/2)*std::sin(M_PI/4)+(3*grid_size/2+0.001-grid_size/2)*std::cos(M_PI/4),
                                                    tmp(2)+3*grid_size);
  visual_tools_->publishCuboid(cuboid_plane_pose, 0.019, 0.0001, 0.029, getColor(0.0, 191.0, 255.0, 1.0));

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

