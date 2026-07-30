cmake_minimum_required(VERSION 3.5)
project(voxelslam_pointcloud2)

if(ROS_EDITION STREQUAL "ROS1")
  add_definitions(-DROS1)

  find_package(catkin REQUIRED COMPONENTS rviz)
  catkin_package()
  include_directories(${catkin_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})

  set(CMAKE_AUTOMOC ON)

  if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
  else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
  endif()

  set(SRC_FILES
    src/voxelslam_pc2.cpp
  )

  add_library(${PROJECT_NAME} ${SRC_FILES})

  target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

  install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  # install(DIRECTORY media/
  #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

else(ROS_EDITION STREQUAL "ROS2")
  find_package(ament_cmake REQUIRED)
  find_package(rviz_common REQUIRED)
  find_package(rviz_rendering REQUIRED)
  find_package(rviz_default_plugins REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

  set(SRC_FILES
    src/voxelslam_pc2.cpp
  )

  add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

  set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
  )

  target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
    sensor_msgs::sensor_msgs
  )

  target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_rendering_INCLUDE_DIRS}
    ${rviz_default_plugins_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(
    FILES plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
  )

  # install(
  #   DIRECTORY media/
  #   DESTINATION share/${PROJECT_NAME}/media
  # )

  pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(
    rviz_common
    rviz_rendering
    sensor_msgs
  )
  ament_export_include_directories(include)

  ament_package()

endif()

