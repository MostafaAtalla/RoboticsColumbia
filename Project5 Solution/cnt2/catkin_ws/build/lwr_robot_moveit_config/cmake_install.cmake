# Install script for directory: /home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot_moveit_config

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/install")
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

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/build/lwr_robot_moveit_config/catkin_generated/installspace/lwr_robot_moveit_config.pc")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_robot_moveit_config/cmake" TYPE FILE FILES
    "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/build/lwr_robot_moveit_config/catkin_generated/installspace/lwr_robot_moveit_configConfig.cmake"
    "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/build/lwr_robot_moveit_config/catkin_generated/installspace/lwr_robot_moveit_configConfig-version.cmake"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_robot_moveit_config" TYPE FILE FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot_moveit_config/package.xml")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_robot_moveit_config" TYPE DIRECTORY FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot_moveit_config/launch")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_robot_moveit_config" TYPE DIRECTORY FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot_moveit_config/config")
endif()

