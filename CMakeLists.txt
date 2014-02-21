cmake_minimum_required(VERSION 2.8.3)
project(robo_surrogate)

find_package(orocos_kdl)

find_package(catkin REQUIRED 
  COMPONENTS
    actionlib
    control_msgs
    geometry_msgs
    sensor_msgs
    roscpp
    tf
    tf_conversions
    urdf
    kdl_parser
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    control_msgs
    geometry_msgs
    sensor_msgs
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(head_pointer
  src/head_pointer.cpp
  )
target_link_libraries(head_pointer
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(head_pointer
  sensor_msgs_generate_messages_cpp
)

add_executable(arm_mover
  src/arm_mover.cpp
  )
target_link_libraries(arm_mover
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(arm_mover
  sensor_msgs_generate_messages_cpp
)

add_executable(incremental_arm_mover
  src/incremental_arm_mover.cpp
  )
target_link_libraries(incremental_arm_mover
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
)
add_dependencies(incremental_arm_mover
  sensor_msgs_generate_messages_cpp
)


add_executable(arm_pre_mover
  src/arm_pre_mover.cpp
  )
target_link_libraries(arm_pre_mover
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
)
add_dependencies(arm_pre_mover
  sensor_msgs_generate_messages_cpp
)


install(TARGETS head_pointer arm_mover incremental_arm_mover
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
