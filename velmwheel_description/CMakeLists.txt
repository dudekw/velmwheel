cmake_minimum_required(VERSION 2.8.3)

project(velmwheel_description)

find_package(catkin REQUIRED urdf)

catkin_package(CATKIN_DEPENDS xacro urdf)
include_directories(${catkin_INCLUDE_DIRS} )
# add_executable(parser src/parser.cpp)
# target_link_libraries(parser ${catkin_LIBRARIES})

foreach(dir urdf)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

