cmake_minimum_required(VERSION 2.8.3)
project(rudpt_evaluation)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vikit_py
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/helpers/set_eval_times.py
  scripts/helpers/fix_timestamp.py
  scripts/helpers/path_writer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)