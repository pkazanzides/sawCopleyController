#
# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawCopleyConsole VERSION 2.1.0)

# List cisst libraries needed
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  find_package (sawCopleyController
    HINTS ${CMAKE_BINARY_DIR})

  if (sawCopleyController_FOUND)

    include_directories (${sawCopleyController_INCLUDE_DIR})
    link_directories (${sawCopleyController_LIBRARY_DIR})

    add_executable (sawCopleyConsole main.cpp)

    # link with the cisst libraries
    cisst_target_link_libraries (sawCopleyConsole ${REQUIRED_CISST_LIBRARIES})

    # link with sawCopleyController library
    target_link_libraries (sawCopleyConsole ${sawCopleyController_LIBRARIES})

    set_target_properties (sawCopleyConsole PROPERTIES
      COMPONENT sawCopleyController-Examples
      FOLDER "sawCopleyController")

    install (TARGETS sawCopleyConsole
      COMPONENT sawCopleyController-Examples
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

  else (sawCopleyController_FOUND)
    message ("Information: sawCopleyConsole will not be compiled, it requires sawCopleyController")
  endif (sawCopleyController_FOUND)
else (cisst_FOUND_AS_REQUIRED)
  message ("Information: sawCopleyConsole will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
