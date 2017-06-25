cmake_minimum_required(VERSION 2.8.3)

project(velmobil_gl)
find_package(Boost COMPONENTS boost_random)
find_package(OROCOS-RTT REQUIRED)
find_package(catkin REQUIRED
	    )


find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules tf)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

catkin_package(
  DEPENDS rtt 
  CATKIN_DEPENDS rtt_ros 
)


orocos_component(${PROJECT_NAME} velmobil_gl.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} orocos-bfl ${BOOST_LIBRARIES})

orocos_generate_package()

