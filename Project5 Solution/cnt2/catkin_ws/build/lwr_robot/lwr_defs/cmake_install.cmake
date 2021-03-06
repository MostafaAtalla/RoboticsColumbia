# Install script for directory: /home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot/lwr_defs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/build/lwr_robot/lwr_defs/catkin_generated/installspace/lwr_defs.pc")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_defs/cmake" TYPE FILE FILES
    "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/build/lwr_robot/lwr_defs/catkin_generated/installspace/lwr_defsConfig.cmake"
    "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/build/lwr_robot/lwr_defs/catkin_generated/installspace/lwr_defsConfig-version.cmake"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_defs" TYPE FILE FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot/lwr_defs/package.xml")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_defs/calibration" TYPE DIRECTORY FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot/lwr_defs/calibration/")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_defs/defs" TYPE DIRECTORY FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot/lwr_defs/defs/")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_defs/meshes" TYPE DIRECTORY FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot/lwr_defs/meshes/")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_defs/robots" TYPE DIRECTORY FILES "/home/ccc_45345959f5_44723/asn51553_5/asn51554_1/work/catkin_ws/src/lwr_robot/lwr_defs/robots/")
endif()

