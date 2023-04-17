cmake_minimum_required(VERSION 2.8.3)
project(map_stack)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  nav_msgs
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  nav_msgs 
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(map_stack src/map_stack.cpp)
add_dependencies(map_stack ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_stack ${catkin_LIBRARIES})
