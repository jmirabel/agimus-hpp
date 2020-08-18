# Copyright (c) 2018, 2019, 2020 CNRS and Airbus S.A.S
# Author: Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAME agimus-hpp)
SET(PROJECT_DESCRIPTION "Package to link HPP to the Agimus framework")
SET(PROJECT_URL "")

SET(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/idl.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

OPTION(BUILD_HPP_PLUGIN ON "")
OPTION(BUILD_ROS_INTERFACE ON "")

LIST(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

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

  add_project_dependency("agimus_sot_msgs")
ELSE(BUILD_ROS_INTERFACE)
  ADD_REQUIRED_DEPENDENCY("geometry_msgs")
ENDIF(BUILD_ROS_INTERFACE)

IF(BUILD_HPP_PLUGIN)
  ADD_REQUIRED_DEPENDENCY("roscpp")
  add_project_dependency("dynamic_graph_bridge_msgs")
endif(BUILD_HPP_PLUGIN)

ADD_REQUIRED_DEPENDENCY("omniORB4")
ADD_PROJECT_DEPENDENCY("hpp-corbaserver")
ADD_PROJECT_DEPENDENCY("hpp-manipulation-corba")

ADD_SUBDIRECTORY(src)

IF(BUILD_ROS_INTERFACE)
  INSTALL(PROGRAMS
    scripts/hpp_node.py
    scripts/estimation.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
  INSTALL(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
ENDIF(BUILD_ROS_INTERFACE)
