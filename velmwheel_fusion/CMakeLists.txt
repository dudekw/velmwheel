cmake_minimum_required(VERSION 2.8.3)

project(velmwheel_fusion)
find_package(Boost COMPONENTS boost_random)
find_package(OROCOS-RTT REQUIRED)
find_package(catkin REQUIRED COMPONENTS
		robot_localization
	    )

include_directories(~/biblioteki/include/bfl)
set(BFL_LIBRARIES ~/biblioteki/lib/liborocos-bfl.so)

find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules tf)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

catkin_package(
  DEPENDS rtt 
  CATKIN_DEPENDS rtt_ros 
)




  #ADD_EXECUTABLE( test_nonlinear_kalman src/test_nonlinear_kalman.cpp src/nonlinearanalyticconditionalgaussianmobile.cpp src/mobile_robot.cpp) 

  #TARGET_LINK_LIBRARIES( test_nonlinear_kalman orocos-bfl ${BOOST_LIBRARIES} ${BFL_LIBRARIES})

  INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src/wrappers/matrix ${CMAKE_SOURCE_DIR}/src ${RNG_INCLUDE} ${MATRIX_INCLUDE} ${robot_localization_INCLUDE_DIRS} )
 
 #INSTALL(TARGETS test_nonlinear_kalman
 #         DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/bfl 
 #         PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

orocos_component(${PROJECT_NAME} src/VelmWheel_fusion.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} orocos-bfl ${BOOST_LIBRARIES} ${BFL_LIBRARIES} ${robot_localization_LIBRARIES})

orocos_generate_package()

