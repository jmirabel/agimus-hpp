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
SET(LIBRARY_NAME ${PROJECT_NAME})

INCLUDE(${HPP_CORBASERVER_CMAKE_PLUGIN})
INCLUDE(${HPP_CORBASERVER_OMNIIDL_CMAKE})
OMNIIDL_INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/idl ${HPP_CORBASERVER_DATAROOTDIR}/idl
  )
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)

MAKE_DIRECTORY(${CMAKE_BINARY_DIR}/src/agimusHpp_idl)
FOREACH(IDL server discretization)
  GENERATE_IDL_CPP (agimusHpp_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/agimusHpp_idl
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/agimusHpp_idl
    ENABLE_DOCSTRING
    STUBS agimusHpp_stubs
    ARGUMENTS
    -Wbextern=common:hpp_stubs
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    )
ENDFOREACH()

GENERATE_IDL_CPP_IMPL (agimusHpp_idl/discretization ${CMAKE_SOURCE_DIR}/idl/agimusHpp_idl
  ARGUMENTS
  -Wbguard_prefix=hpp_agimus_idl
  -Wbinc_prefix=agimusHpp_idl
  HH_SUFFIX -idl.hh)

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/idl/agimusHpp_idl
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/hpp)

INSTALL(
  DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/agimusHpp_idl
  DESTINATION ${PYTHON_SITELIB})
INSTALL(
  DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/agimusHpp_stubs
  DESTINATION ${PYTHON_SITELIB})

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS} ${ALL_IDL_CPP_IMPL_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

IF(BUILD_HPP_PLUGIN)
  HPP_ADD_SERVER_PLUGIN(agimus-hpp
    SOURCES
    server.cc
    discretization.cc
    ${ALL_IDL_CPP_STUBS}
    ${ALL_IDL_CPP_IMPL_STUBS}
    LINK_DEPENDENCIES PUBLIC hpp-corbaserver::hpp-corbaserver
    PKG_CONFIG_DEPENDENCIES omniORB4 roscpp dynamic_graph_bridge_msgs)

  ADD_DEPENDENCIES (agimus-hpp generate_idl_cpp generate_idl_python)
ELSE(BUILD_HPP_PLUGIN)
  ADD_CUSTOM_TARGET(generate_idl ALL)
  # No need to generate C++ bindings
  ADD_DEPENDENCIES (generate_idl generate_idl_python)
ENDIF(BUILD_HPP_PLUGIN)

# Install Python files

IF(BUILD_ROS_INTERFACE)
  SET (PYTHON_FILE
      estimation.py
      trajectory_publisher.py
      planning_request_adapter.py
      client.py
      ros_tools.py
      tools.py
      __init__.py)

  FOREACH(F ${PYTHON_FILE})
    PYTHON_INSTALL_ON_SITE("agimus_hpp" ${F})
  ENDFOREACH()

  SET (PYTHON_FILE
      planning_request_adapter.py
      __init__.py)

  FOREACH(F ${PYTHON_FILE})
    PYTHON_INSTALL_ON_SITE("agimus_hpp/manipulation" ${F})
  ENDFOREACH()
ENDIF(BUILD_ROS_INTERFACE)

SET (PYTHON_FILE
    client.py
    __init__.py)
FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("hpp/agimus" ${F})
ENDFOREACH()
