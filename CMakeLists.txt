# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-corbaserver.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-corbaserver is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/idl.cmake)

SET(PROJECT_NAME agimus-hpp)
SET(PROJECT_DESCRIPTION "Package to link HPP to the Agimus framework")
SET(PROJECT_URL "")

OPTION(BUILD_HPP_PLUGIN ON "")
OPTION(BUILD_ROS_INTERFACE ON "")

LIST(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

SETUP_PROJECT()
FINDPYTHON(REQUIRED)

# Activate hpp-util logging if requested
SET (HPP_BENCHMARK FALSE CACHE BOOL "trigger hpp-util benchmark output")
IF (HPP_BENCHMARK)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
ENDIF()

IF(BUILD_ROS_INTERFACE)
  find_package(catkin REQUIRED COMPONENTS
    dynamic_graph_bridge_msgs
    geometry_msgs
    sensor_msgs
    agimus_sot_msgs
    std_msgs
    rospy
    )

  catkin_package()

  ADD_REQUIRED_DEPENDENCY("agimus_sot_msgs")
ELSE(BUILD_ROS_INTERFACE)
  ADD_REQUIRED_DEPENDENCY("geometry_msgs")
ENDIF(BUILD_ROS_INTERFACE)

IF(BUILD_HPP_PLUGIN)
  ADD_REQUIRED_DEPENDENCY("roscpp")
  ADD_REQUIRED_DEPENDENCY("dynamic_graph_bridge_msgs")
endif(BUILD_HPP_PLUGIN)

ADD_REQUIRED_DEPENDENCY("omniORB4")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver")
ADD_REQUIRED_DEPENDENCY("hpp-manipulation-corba")

ADD_SUBDIRECTORY(src)

IF(BUILD_ROS_INTERFACE)
  INSTALL(PROGRAMS
    scripts/hpp_node.py
    scripts/estimation.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
  INSTALL(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
ENDIF(BUILD_ROS_INTERFACE)

SETUP_PROJECT_FINALIZE()
