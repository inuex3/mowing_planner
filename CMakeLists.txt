cmake_minimum_required(VERSION 2.8.3)
project(mowing_planner)

find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mowing_planner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
)

add_library(mowing_planner src/mowing_planner.cpp)
add_dependencies(mowing_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mowing_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS mowing_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


