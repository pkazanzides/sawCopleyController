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

cmake_minimum_required(VERSION 3.10)
project (sawCopleyControllerCore VERSION 0.0.1)

# to locate cisst macros
find_package (cisst REQUIRED cisstCommon)
include (${CISST_USE_FILE})

cisst_cpack_settings (
  VENDOR "JHU"
  MAINTAINER "pkaz@jhu.edu")

add_subdirectory (components)

set (sawCopleyController_DIR "${sawCopleyControllerCore_BINARY_DIR}/components")
add_subdirectory (examples)
add_subdirectory (share)

include (CPack)
cpack_add_component (sawCopleyController)
cpack_add_component (sawCopleyController-dev
                     DEPENDS sawCopleyController)
cpack_add_component (sawCopleyController-Examples
                     DEPENDS sawCopleyController)
cpack_add_component (sawCopleyController-Share
                     DEPENDS sawCopleyController sawCopleyController-Share)
