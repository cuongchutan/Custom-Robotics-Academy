# Install script for directory: /home/cuong/jderobot_ws/src/PX4-Autopilot/src/drivers/optical_flow

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/cuong/jderobot_ws/src/PX4-Autopilot/build/px4_fmu-v4_default/src/drivers/optical_flow/paw3902/cmake_install.cmake")
  include("/home/cuong/jderobot_ws/src/PX4-Autopilot/build/px4_fmu-v4_default/src/drivers/optical_flow/pmw3901/cmake_install.cmake")
  include("/home/cuong/jderobot_ws/src/PX4-Autopilot/build/px4_fmu-v4_default/src/drivers/optical_flow/px4flow/cmake_install.cmake")
  include("/home/cuong/jderobot_ws/src/PX4-Autopilot/build/px4_fmu-v4_default/src/drivers/optical_flow/thoneflow/cmake_install.cmake")

endif()

