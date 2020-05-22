cmake_minimum_required(VERSION 2.8.12)
project(multi_mav_manager)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  kr_mav_manager
  std_msgs
  std_srvs
  cmake_modules
  message_generation)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  include/capt/hungarian
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

add_service_files(
  DIRECTORY srv
  FILES
    Formation.srv
    RawPosFormation.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    include/capt/hungarian
  LIBRARIES kr_mav_manager
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    kr_mav_manager
    std_msgs
    std_srvs
    message_runtime
  DEPENDS EIGEN3)

add_library(capt include/capt/hungarian/hungarian.cpp)

add_executable(multi_mav_services src/multi_mav_services.cpp src/mav_manager_interface.cpp)
target_link_libraries(multi_mav_services capt ${catkin_LIBRARIES})

install(TARGETS
    multi_mav_services
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)