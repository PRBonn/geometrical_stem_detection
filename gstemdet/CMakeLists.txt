cmake_minimum_required(VERSION 2.8.3)
project(gstemdet)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)
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
  LIBRARIES stemdet
  CATKIN_DEPENDS
  DEPENDS YAML_CPP OpenCV
)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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
  add_definitions(-DDEBUG_MODE)
  message("Debug Mode")
endif()

if(${CMAKE_BUILD_TYPE} MATCHES Timer)
  add_definitions(-DTIMER)
  message("Timing Mode")
endif()

## Use C++11
add_definitions(-std=c++11)
# set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
endif()

## Include header and src files
include_directories(include)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()
add_subdirectory(src)


#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
