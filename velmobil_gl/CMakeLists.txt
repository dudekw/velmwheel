cmake_minimum_required(VERSION 2.8.3)

project(velmobil_gl)
find_package(Boost COMPONENTS boost_random property_tree)
find_package(OROCOS-RTT REQUIRED)
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules tf)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  DEPENDS rtt 
  CATKIN_DEPENDS rtt_ros 
)


orocos_component(${PROJECT_NAME} src/velmobil_gl.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE)

orocos_generate_package()

