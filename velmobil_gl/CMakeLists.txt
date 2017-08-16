cmake_minimum_required(VERSION 2.8.3)

project(velmobil_gl)
find_package(Boost COMPONENTS boost_random property_tree)
find_package(OROCOS-RTT REQUIRED)
find_package(catkin REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules tf)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  DEPENDS rtt 
  CATKIN_DEPENDS rtt_ros 
)

add_library(intensity_map_lib SHARED src/intensity_map_lib.cpp)
target_link_libraries(intensity_map_lib ${catkin_LIBRARIES} ${BOOST_LIBRARIES} )

orocos_component(${PROJECT_NAME} src/velmobil_gl.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BOOST_LIBRARIES} ${OpenCV_LIBRARIES} intensity_map_lib)


install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/intensity_map_lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(TARGETS intensity_map_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

orocos_generate_package()

