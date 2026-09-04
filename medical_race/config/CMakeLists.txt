cmake_minimum_required(VERSION 3.0.2)
project(medical_race)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  roborts_msgs
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
  std_srvs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    roborts_msgs
    roscpp
    rospy
    sensor_msgs
    serial
    std_msgs
    std_srvs
    tf
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/medical_competition_master.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  README_competition_master.md
  CHANGELOG_STATE_MACHINE.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
