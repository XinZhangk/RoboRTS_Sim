project(roborts_sim)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  actionlib
  roborts_msgs
  std_msgs
  )

find_package(Threads REQUIRED)

catkin_package()

add_executable(
  sim_node
  sim_node.cpp 
  sim_node.h
  sim_map.h
  sim_map.cpp
)

add_dependencies(sim_node
  roborts_msgs_generate_messages_cpp)

target_link_libraries(sim_node PUBLIC
        ${catkin_LIBRARIES}
)

target_include_directories(sim_node PUBLIC
        ${catkin_INCLUDE_DIRS}
)

add_executable(
  sim_cmd_node
  sim_cmd_node.cpp 
)

add_dependencies(sim_cmd_node
  roborts_msgs_generate_messages_cpp)

target_link_libraries(sim_cmd_node PUBLIC
        ${catkin_LIBRARIES}
)

target_include_directories(sim_cmd_node PUBLIC
        ${catkin_INCLUDE_DIRS}
)


