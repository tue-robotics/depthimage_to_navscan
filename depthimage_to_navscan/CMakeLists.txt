cmake_minimum_required(VERSION 3.5)
project(depthimage_to_navscan)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geolib2
  image_geometry
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geolib2 image_geometry
  DEPENDS OpenCV
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/depthimage_to_navscan.cpp
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test(-W2)

  catkin_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endif ()
