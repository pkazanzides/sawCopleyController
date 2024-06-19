#
# (C) Copyright 2016-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawCopleyController VERSION 0.1.0)

# So we can use Findgclib.cmake
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message( FATAL_ERROR "sawCopleyController requires cisst to be built with JSON" )
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()

  set (sawCopleyController_INCLUDE_DIR
    "${sawCopleyController_SOURCE_DIR}/include"
    "${sawCopleyController_BINARY_DIR}/include")
  set (sawCopleyController_HEADER_DIR "${sawCopleyController_SOURCE_DIR}/include/sawCopleyController")
  set (sawCopleyController_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawCopleyController_LIBRARIES sawCopleyController)

  include_directories (BEFORE ${sawCopleyController_INCLUDE_DIR})

  # add all config files for this component
  cisst_add_config_files (sawCopleyController)

  # create data type using the data generator
  cisst_data_generator (sawCopleyController
      "${sawCopleyController_BINARY_DIR}/include"
      "sawCopleyController/"
      code/sawCopleyControllerConfig.cdg)

    set (sawCopleyController_HEADER_FILES
      "${sawCopleyController_HEADER_DIR}/mtsCopleyController.h"
      "${sawCopleyController_HEADER_DIR}/sawCopleyControllerExport.h"
      ${sawCopleyController_CISST_DG_HDRS})

    set (sawCopleyController_SOURCE_FILES
      code/mtsCopleyController.cpp
      ${sawCopleyController_CISST_DG_SRCS})

    add_library (
      sawCopleyController
      ${IS_SHARED}
      ${sawCopleyController_HEADER_FILES}
      ${sawCopleyController_SOURCE_FILES})

    set_target_properties (
      sawCopleyController PROPERTIES
      VERSION ${sawCopleyController_VERSION}
      FOLDER "sawCopleyController")

    cisst_target_link_libraries (
      sawCopleyController
      ${REQUIRED_CISST_LIBRARIES})

    # Install target for headers and library
    install (
      DIRECTORY
      "${sawCopleyController_SOURCE_DIR}/include/sawCopleyController"
      "${sawCopleyController_BINARY_DIR}/include/sawCopleyController"
      DESTINATION include
      COMPONENT sawCopleyController-dev)

    install (
      TARGETS sawCopleyController
      COMPONENT sawCopleyController
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
