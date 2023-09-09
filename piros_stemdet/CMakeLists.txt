cmake_minimum_required(VERSION 2.8.3)
project(piros_stemdet)

## Use C++11
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    message_generation
    image_transport
    cv_bridge
    gstemdet
)

## find OpenCV packages
find_package(OpenCV 4 REQUIRED)

# adding custom messages
add_message_files(DIRECTORY msg FILES Stem.msg Stems.msg)
generate_messages(DEPENDENCIES std_msgs)

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
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    message_runtime
    image_transport
    cv_bridge
    gstemdet
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
)

add_executable(stemecho_node
  src/stemecho/stemecho_node.cpp
  src/stemecho/stemecho.cpp
)

add_executable(stemdetection_node
  src/stemdetection/stemdetection_node.cpp
  src/stemdetection/stemdetection.cpp
)

## Specify libraries to link executable targets against
# target_link_libraries(${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

target_link_libraries(stemecho_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(stemdetection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(stemdetection_node piros_stemdet_generate_messages_cpp)
add_dependencies(stemecho_node piros_stemdet_generate_messages_cpp)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS stemecho_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include
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

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
endif()
