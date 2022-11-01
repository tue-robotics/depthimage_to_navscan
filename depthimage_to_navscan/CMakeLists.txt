cmake_minimum_required(VERSION 3.0.2)
project(depthimage_to_navscan)

find_package(catkin REQUIRED COMPONENTS
    geolib2
    image_geometry
    roscpp
)

## System dependencies are found with CMake's conventions
#TODO // add explicit dependency on OpenCV
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depthimage_to_navscan
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 SYSTEM
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(depthimage_to_navscan
   src/depthimage_to_navscan.cpp
)
target_link_libraries(depthimage_to_navscan ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ------------------------------------------------------------------------------------------------
#                                           TESTS
# ------------------------------------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_depthimage_to_navscan test/test_depthimage_to_navscan.cpp)
    target_link_libraries(test_depthimage_to_navscan depthimage_to_navscan ${catkin_LIBRARIES})
endif ()
