cmake_minimum_required(VERSION 3.02)
project(gps_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14") 
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  tf
  serial
  sleipnir_msgs
  geodesy
)

## pkg-config libraries
find_package(PkgConfig REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# ## Build the library
add_library(${PROJECT_NAME} src/gps_driver.cpp src/imu_driver.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## gps节点1： NavSatFix
add_executable(${PROJECT_NAME}_stardard_node src/gps_driver_stardard_node.cpp)
target_link_libraries(${PROJECT_NAME}_stardard_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## gps节点2： sensorgps
add_executable(${PROJECT_NAME}_node src/gps_driver_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


## gps节点1： sensorgps to NavSatFix
add_executable(sensorgps2NavSatFix src/sensorgps2NavSatFix.cpp)
target_link_libraries(sensorgps2NavSatFix
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_stardard_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS sensorgps2NavSatFix ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
