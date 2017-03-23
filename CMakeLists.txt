cmake_minimum_required(VERSION 2.8.3)
project(renbo_grasp_generator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  rviz_visual_tools
  roscpp
  roslib
  renbo_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    renbo_grasp
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    rviz_visual_tools
    roscpp
    roslib
    renbo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
    ${catkin_LIBRARY_DIRS}
)

set(renbo_grasp_lib_files
  src/grasp_planning.cpp
  src/capability_map.cpp
  src/posture_generator.cpp

  include/renbo_grasp_generator/grasp_planning.h
  include/renbo_grasp_generator/capability_map.h
  include/renbo_grasp_generator/posture_generator.h
)

add_library(${PROJECT_NAME} ${renbo_grasp_lib_files})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(posture_generator_node src/posture_generator_node.cpp)
target_link_libraries(posture_generator_node ${PROJECT_NAME})

add_executable(build_capability_map util/build_capability_map.cpp)
target_link_libraries(build_capability_map ${PROJECT_NAME})

add_executable(gripper_display test/gripper_display.cpp)
target_link_libraries(gripper_display ${PROJECT_NAME})

add_executable(object_grid test/object_grid.cpp)
target_link_libraries(object_grid ${PROJECT_NAME})

add_executable(cylinder_object_display test/cylinder_object_display.cpp)
target_link_libraries(cylinder_object_display ${PROJECT_NAME})

add_executable(test_colorbar test/test_colorbar.cpp)
target_link_libraries(test_colorbar ${PROJECT_NAME})

add_executable(test_capability_map test/test_capability_map.cpp)
target_link_libraries(test_capability_map ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
