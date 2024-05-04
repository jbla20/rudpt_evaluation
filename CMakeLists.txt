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
  scripts/svo/svo_pose_to_txt.py
  scripts/orb_slam3/orb_slam3_pose_to_txt.py
  scripts/vins/vins_path_to_txt.py
  scripts/helpers/set_eval_times.py
  scripts/helpers/fix_timestamp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)