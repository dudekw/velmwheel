cmake_minimum_required(VERSION 2.8.3)

project(velmwheel_laser_driver)

find_package(catkin REQUIRED COMPONENTS 
		rtt_ros 
		cmake_modules
		lms1xx)
find_package(OROCOS-RTT REQUIRED)
find_package(lms1xx)
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()
set (CMAKE_CXX_FLAGS "-fPIC")

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

catkin_package(
  DEPENDS rtt 
  CATKIN_DEPENDS rtt_ros 
)

orocos_component(${PROJECT_NAME} src/VelmWheel_laser_driver.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${lms1xx_LIBRARIES})

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE)


orocos_generate_package()
