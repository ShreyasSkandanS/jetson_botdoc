cmake_minimum_required(VERSION 2.8.3)
project(jetson_botdoc)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation rospy std_msgs)

catkin_python_setup()

add_message_files(
  FILES
  healthReportData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/docConfig.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_generation message_runtime rospy std_msgs
)

catkin_install_python(PROGRAMS src/monitor.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
