cmake_minimum_required(VERSION 3.0.2)
set (CMAKE_CXX_STANDARD 14)
project(open3d_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
)

#find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

add_message_files(
  FILES
  PointsImages.msg
  PointsImagesFront.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
#  LIBRARIES open3d_test
CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
DEPENDS Open3D
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp")

###########
## Build ##
###########

link_directories(${Open3D_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR}
  #/usr/local/include/open3d/3rdparty/
  ${Open3D_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES} ${Open3D_LIBRARIES})

add_executable(open3d_test_interpolate src/interpolate.cpp)

add_executable(open3d_test_adapter src/adapter.cpp)

add_executable(open3d_test_remove_noise_node src/remove_noise_node.cpp)
