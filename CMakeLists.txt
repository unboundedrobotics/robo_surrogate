cmake_minimum_required(VERSION 2.8.3)
project(robo_surrogate)

find_package(catkin REQUIRED 
  COMPONENTS
    actionlib
    control_msgs
    geometry_msgs
    sensor_msgs
    roscpp
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

include_directories(${catkin_INCLUDE_DIRS})

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

install(TARGETS head_pointer arm_mover
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
