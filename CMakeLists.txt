cmake_minimum_required(VERSION 3.0.2)
project(open3d_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

add_message_files(
  FILES
  PointsImages.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES open3d_test
CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
DEPENDS EIGEN3 Open3D
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp")

add_executable(open3d_test_hoge src/hoge.cpp)
target_link_libraries(open3d_test_hoge ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
