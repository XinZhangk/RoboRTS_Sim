project(roborts_sim)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  )

find_package(Threads REQUIRED)

catkin_package()
add_executable(
  sim_node
  sim_node.cpp 
  sim_node.h
)


target_link_libraries(sim_node PUBLIC
        ${catkin_LIBRARIES}
)

target_include_directories(sim_node PUBLIC
        ${catkin_INCLUDE_DIRS}
)

