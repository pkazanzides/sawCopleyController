cmake_minimum_required (VERSION 3.10)
project (saw_copley_controller_config VERSION 0.0.1)

find_package (catkin QUIET)
if (catkin_FOUND)
  catkin_package()
endif ()

install (
  DIRECTORY "${saw_copley_controller_config_SOURCE_DIR}"
  DESTINATION share/sawCopleyController
  COMPONENT mtsCopleyController-Share
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "package.xml" EXCLUDE)
