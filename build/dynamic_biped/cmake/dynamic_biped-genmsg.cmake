# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dynamic_biped: 13 messages, 7 services")

set(MSG_I_FLAGS "-Idynamic_biped:/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dynamic_biped_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg" "geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg" "geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv" ""
)

get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv" NAME_WE)
add_custom_target(_dynamic_biped_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_biped" "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)

### Generating Services
_generate_srv_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_cpp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
)

### Generating Module File
_generate_module_cpp(dynamic_biped
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dynamic_biped_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dynamic_biped_generate_messages dynamic_biped_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_cpp _dynamic_biped_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_biped_gencpp)
add_dependencies(dynamic_biped_gencpp dynamic_biped_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_biped_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_msg_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)

### Generating Services
_generate_srv_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_srv_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_srv_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_srv_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_srv_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_srv_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)
_generate_srv_eus(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
)

### Generating Module File
_generate_module_eus(dynamic_biped
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dynamic_biped_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dynamic_biped_generate_messages dynamic_biped_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_eus _dynamic_biped_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_biped_geneus)
add_dependencies(dynamic_biped_geneus dynamic_biped_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_biped_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_msg_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)

### Generating Services
_generate_srv_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)
_generate_srv_lisp(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
)

### Generating Module File
_generate_module_lisp(dynamic_biped
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dynamic_biped_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dynamic_biped_generate_messages dynamic_biped_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_lisp _dynamic_biped_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_biped_genlisp)
add_dependencies(dynamic_biped_genlisp dynamic_biped_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_biped_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_msg_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)

### Generating Services
_generate_srv_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_srv_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_srv_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_srv_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_srv_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_srv_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)
_generate_srv_nodejs(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
)

### Generating Module File
_generate_module_nodejs(dynamic_biped
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dynamic_biped_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dynamic_biped_generate_messages dynamic_biped_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_nodejs _dynamic_biped_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_biped_gennodejs)
add_dependencies(dynamic_biped_gennodejs dynamic_biped_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_biped_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_msg_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)

### Generating Services
_generate_srv_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_srv_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_srv_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_srv_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_srv_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_srv_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)
_generate_srv_py(dynamic_biped
  "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
)

### Generating Module File
_generate_module_py(dynamic_biped
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dynamic_biped_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dynamic_biped_generate_messages dynamic_biped_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv" NAME_WE)
add_dependencies(dynamic_biped_generate_messages_py _dynamic_biped_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_biped_genpy)
add_dependencies(dynamic_biped_genpy dynamic_biped_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_biped_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_biped
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dynamic_biped_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dynamic_biped_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_biped
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dynamic_biped_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(dynamic_biped_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_biped
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dynamic_biped_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dynamic_biped_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_biped
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dynamic_biped_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(dynamic_biped_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_biped
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dynamic_biped_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dynamic_biped_generate_messages_py geometry_msgs_generate_messages_py)
endif()
