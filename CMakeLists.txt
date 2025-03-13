cmake_minimum_required(VERSION 3.10)
project(pcl_pkg)

## Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-unused-function)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    roslint
    camera_info_manager
    image_transport
    ddynamic_reconfigure
    tf2_ros
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

# Declare the message files to be built
add_message_files(FILES
  FisheyeImages.msg
  OrientationStamped.msg
  Plane.msg
  Planes.msg
  Lost.msg
  PoseStampedConfidence.msg
)

# Declare the service files to be built
add_service_files(FILES
  GetPose.srv
  GetPoseAt.srv
  GetOrientation.srv
  GetOrientationAt.srv
  GetDevices.srv
  LoadMapAndSwithcCslam.srv
  SaveMapAndSwitchCslam.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    camera_info_manager
    image_transport
    ddynamic_reconfigure
    tf2_ros
)

###########
## Build ##
###########

# Find XVSDK if necessary
if (XVSDK_INCLUDE_DIRS AND XVSDK_LIBRARIES)
  set(XVSDK_FOUND TRUE)
else()
  set(XVSDK_FOUND FALSE)
endif()
if (XVSDK_FOUND)
  message(STATUS " - xvsdk includes: ${XVSDK_INCLUDE_DIRS}")
  message(STATUS " - xvsdk libraries: ${XVSDK_LIBRARIES}")
else()
  message(FATAL_ERROR "xvsdk not provided")
endif()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${XVSDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare cpp executables and link libraries
add_executable(pcl_test src/pcl_test.cpp)
add_executable(pcl_save src/pcl_save.cpp)
add_executable(slam_path src/slam_path.cpp)
add_executable(slam_path2 src/slam_path2.cpp)
add_executable(tof_point src/tof_point.cpp)
add_executable(tof_point_rgbd src/tof_point_rgbd.cpp)
add_executable(tof_slam src/tof_slam.cpp)
add_executable(tof_slam_filter src/tof_slam_filter.cpp)
add_executable(tof_slam_filter2 src/tof_slam_filter2.cpp)
add_executable(tof_slam_filter3 src/tof_slam_filter3.cpp)
add_executable(ex1 src/ex1.cpp)
add_executable(ex2 src/ex2.cpp)
add_executable(ex3 src/ex3.cpp)
add_executable(tof_point_color src/tof_point_color.cpp)
add_executable(tof_slam_color src/tof_slam_color.cpp)
add_executable(tof_slam_filter_save src/tof_slam_filter_save.cpp)
add_executable(slam_start src/slam_start.cpp)


# Link the libraries for each executable
target_link_libraries(pcl_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(pcl_save
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(slam_path
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(slam_path2
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_point
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_point_rgbd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_slam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_slam_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_slam_filter2
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_slam_filter3
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(ex1
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(ex2
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(ex3
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_point_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_slam_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(tof_slam_filter_save
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(slam_start
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS pcl_test pcl_save tof_point
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(${CATKIN_ENABLE_TESTING})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test
#   test/test_xv_sdk.cpp)
# target_link_libraries(${PROJECT_NAME}-test ${XVSDK_LIBRARIES})
endif()

##########################
## Static code analysis ##
##########################

roslint_cpp()
