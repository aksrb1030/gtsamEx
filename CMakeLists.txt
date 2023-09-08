cmake_minimum_required(VERSION 2.8.3)
project(gtsam_ex)

# Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  jsk_rviz_plugins
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
)

generate_dynamic_reconfigure_options(
  cfg/gtsamConfig.cfg
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)

# message("====" ${Sophus_INCLUDE_DIRS})
include_directories(
  src
  include
  /usr/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}

  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake


  )
# include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${CERES_INCLUDE_DIRS})

# # include_directories(${CHOLMOD_INCLUDE_DIR})
# include_directories(${Sophus_INCLUDE_DIRS})
# include_directories(${GTSAM_INCLUDE_DIR})

# include_directories(common/flags)

# message("" ${G2O_LIBS})



catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs dynamic_reconfigure
  DEPENDS EIGEN3 PCL GTSAM LIBRARIES include
)

add_executable(${PROJECT_NAME}_poseGraph src/pose_graph_gtsam.cpp src/visualizer.cpp)
target_link_libraries(${PROJECT_NAME}_poseGraph ${catkin_LIBRARIES} ${Sophus_LIBRARIES} gtsam)
add_dependencies(${PROJECT_NAME}_poseGraph ${PROJECT_NAME}_gencfg)

# add_executable(gtsam_ex src/pose_graph_gtsam.cpp)
# target_link_libraries(gtsam_ex ${catkin_LIBRARIES} ${Sophus_LIBRARIES} gtsam)