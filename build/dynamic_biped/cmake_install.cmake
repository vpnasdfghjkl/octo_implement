# Install script for directory: /home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rebot801/LIuXin/ICCUB_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/msg" TYPE FILE FILES
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/walkCommand.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/ECJointMotordata.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotQVTau.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotTorsoState.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotPhase.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmInfo.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotArmQVVD.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHandPosition.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/robotHeadMotionData.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotation.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/QuaternionArray.msg"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/msg/handRotationEular.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/srv" TYPE FILE FILES
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangePhases.srv"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvClearPositionCMD.srv"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvchangeCtlMode.srv"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeArmCtrlMode.srv"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/changeAMBACCtrlMode.srv"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/srvChangeJoller.srv"
    "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/srv/controlEndHand.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/cmake" TYPE FILE FILES "/home/rebot801/LIuXin/ICCUB_ws/build/dynamic_biped/catkin_generated/installspace/dynamic_biped-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/rebot801/LIuXin/ICCUB_ws/devel/include/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/rebot801/LIuXin/ICCUB_ws/devel/share/roseus/ros/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/rebot801/LIuXin/ICCUB_ws/devel/share/common-lisp/ros/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/rebot801/LIuXin/ICCUB_ws/devel/share/gennodejs/ros/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/rebot801/LIuXin/ICCUB_ws/devel/lib/python3/dist-packages/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/rebot801/LIuXin/ICCUB_ws/devel/lib/python3/dist-packages/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rebot801/LIuXin/ICCUB_ws/build/dynamic_biped/catkin_generated/installspace/dynamic_biped.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/cmake" TYPE FILE FILES "/home/rebot801/LIuXin/ICCUB_ws/build/dynamic_biped/catkin_generated/installspace/dynamic_biped-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/cmake" TYPE FILE FILES
    "/home/rebot801/LIuXin/ICCUB_ws/build/dynamic_biped/catkin_generated/installspace/dynamic_bipedConfig.cmake"
    "/home/rebot801/LIuXin/ICCUB_ws/build/dynamic_biped/catkin_generated/installspace/dynamic_bipedConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped" TYPE FILE FILES "/home/rebot801/LIuXin/ICCUB_ws/src/dynamic_biped/package.xml")
endif()

