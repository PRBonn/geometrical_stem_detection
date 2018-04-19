cmake_minimum_required(VERSION 2.8.3)
project(piros_vegsec)

# handle debug vs release mode
# if no option is defined, default to release mode (optimizations, no debug)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("Build type: ${CMAKE_BUILD_TYPE}") # print mode

# set the flags for each mode
set(CMAKE_CXX_FLAGS "-Wall")  # complain about everything!
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0") # don't optimize, roduce debug info (for gdb)
set(CMAKE_CXX_FLAGS_RELEASE "-O3") # optimize, no debug

if(${CMAKE_BUILD_TYPE} MATCHES Debug)
  set(CMAKE_CXX_FLAGS "-DDEBUG_MODE ${CMAKE_CXX_FLAGS}")
endif()

## Use C++11
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
)

find_package(OpenCV)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
   src/vegsec.cpp
)

## Declare cpp executables
add_executable(piros_vegsec_node
  src/vegsec_handler.cpp
  src/vegsec_node.cpp
)


## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(piros_vegsec_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
  DIRECTORY doc launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}_test test/vegsec_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_core)
endif()
