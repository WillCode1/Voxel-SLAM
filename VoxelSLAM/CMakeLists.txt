cmake_minimum_required(VERSION 3.5)
project(voxel_slam)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  include
)

if(ROS_EDITION STREQUAL "ROS1")
  add_definitions(-DROS1)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    pcl_conversions
    pcl_ros
    livox_ros_driver
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs 
    DEPENDS EIGEN3 PCL GTSAM
    INCLUDE_DIRS
  )

  add_executable(voxelslam src/voxelslam.cpp src/BTC.cpp)
  target_link_libraries(voxelslam ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

  install(TARGETS voxelslam
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch config rviz_cfg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
else(ROS_EDITION STREQUAL "ROS2")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(livox_ros_driver2 REQUIRED)

  include_directories(
    ${pcl_conversions_INCLUDE_DIRS})

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()

  add_executable(voxelslam src/voxelslam.cpp src/BTC.cpp)
  target_link_libraries(voxelslam ${PCL_LIBRARIES} gtsam)
  ament_target_dependencies(voxelslam rclcpp sensor_msgs nav_msgs visualization_msgs tf2_ros livox_ros_driver2)

  install(TARGETS voxelslam
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY launch config rviz_cfg
    DESTINATION share/${PROJECT_NAME})
endif()

