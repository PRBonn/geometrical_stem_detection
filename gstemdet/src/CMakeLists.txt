add_library(stemdet
  plant.cpp
  leaf.cpp
  stem.cpp
  algorithm.cpp
  tools.cpp
)

add_executable(example
  example.cpp
)

target_link_libraries(stemdet
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS})

target_link_libraries(example
  stemdet
  ${Boost_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(
  TARGETS example stemdet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
