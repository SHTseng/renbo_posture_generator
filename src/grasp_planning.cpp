#include <renbo_grasp_generator/grasp_planning.h>

namespace renbo_grasp_planning
{

GraspPlanning::GraspPlanning():
nh_("~")
{
  gripper_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("gripper_marker", 10);
}

GraspPlanning::~GraspPlanning()
{

}

void GraspPlanning::displayGripper()
{
//  shape_msgs::Mesh gripper_mesh = importMesh();

  visualization_msgs::Marker gripper_marker;
  gripper_marker.header.frame_id = "base";
  gripper_marker.header.stamp = ros::Time::now();
  gripper_marker.id = 0;
  gripper_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  gripper_marker.action = visualization_msgs::Marker::ADD;

  gripper_marker.pose.position.x = 0.0;
  gripper_marker.pose.position.y = 0.0;
  gripper_marker.pose.position.z = 0.0;
  gripper_marker.pose.orientation.x = 0.0;
  gripper_marker.pose.orientation.y = 0.0;
  gripper_marker.pose.orientation.z = 0.0;
  gripper_marker.pose.orientation.w = 1.0;
  gripper_marker.scale.x = 1;
  gripper_marker.scale.y = 1;
  gripper_marker.scale.z = 1;
  gripper_marker.color = getColor(192.0, 192.0, 192.0, 1.0);

  std::string file_path(ros::package::getPath("renbo_description"));
  file_path.append("/meshes/arm/r_gripper.stl");

  gripper_marker.mesh_resource = file_path;
  gripper_marker.mesh_use_embedded_materials = true;

  gripper_marker_pub_.publish(gripper_marker);

  ROS_INFO("Published gripper mesh to scene");
  ros::spinOnce();

}

shape_msgs::Mesh GraspPlanning::importMesh()
{
  std::string file_path(ros::package::getPath("renbo_description"));

  file_path.append("/meshes/arm/r_gripper.stl");
  shapes::Mesh* source_mesh = shapes::createMeshFromResource(file_path);

  ROS_INFO_STREAM("Loaded mesh from source");

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(source_mesh, mesh_msg);
  delete source_mesh;

  shape_msgs::Mesh mesh;
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  return mesh;
}

std_msgs::ColorRGBA GraspPlanning::getColor(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;

  color.r = r/255;
  color.g = g/255;
  color.b = b/255;
  color.a = a;

  return color;
}

}
