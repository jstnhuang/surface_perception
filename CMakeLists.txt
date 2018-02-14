cmake_minimum_required(VERSION 2.8.3)
project(surface_perception)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    surface_perception_segmentation
    surface_perception_shape_extraction
    surface_perception_surface_finder
    surface_perception_surface_history_recorder
    surface_perception_visualization
  CATKIN_DEPENDS
    geometry_msgs
    pcl_conversions
    roscpp
    rospy
    sensor_msgs
    visualization_msgs
  DEPENDS
    Eigen
    PCL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(surface_perception_segmentation src/segmentation.cpp)
add_dependencies(surface_perception_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(surface_perception_segmentation
  surface_perception_shape_extraction
  surface_perception_surface_finder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_library(surface_perception_shape_extraction src/shape_extraction.cpp)
add_dependencies(surface_perception_shape_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(surface_perception_shape_extraction
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_library(surface_perception_surface_finder src/surface_finder.cpp)
add_dependencies(surface_perception_surface_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(surface_perception_surface_finder
  surface_perception_surface_history_recorder
  ${catkin_LIBRARIES}
)

add_library(surface_perception_surface_history_recorder src/surface_history_recorder.cpp)
add_dependencies(surface_perception_surface_history_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(surface_perception_surface_history_recorder
  ${catkin_LIBRARIES}
)

add_library(surface_perception_visualization src/visualization.cpp)
add_dependencies(surface_perception_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(surface_perception_visualization
  ${catkin_LIBRARIES})

add_executable(surface_perception_demo src/demo_main.cpp)
set_target_properties(surface_perception_demo PROPERTIES OUTPUT_NAME demo PREFIX "")
target_link_libraries(surface_perception_demo
  ${catkin_LIBRARIES}
  surface_perception_segmentation
  surface_perception_visualization)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS
  surface_perception_demo
  surface_perception_segmentation
  surface_perception_shape_extraction
  surface_perception_surface_finder
  surface_perception_surface_history_recorder
  surface_perception_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_surface_perception.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
