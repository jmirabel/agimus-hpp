SET(LIBRARY_NAME ${PROJECT_NAME})

INCLUDE(${HPP_CORBASERVER_CMAKE_PLUGIN})
INCLUDE(${HPP_CORBASERVER_DATAROOTDIR}/hpp-corbaserver/cmake/omniidl.cmake)
OMNIIDL_INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/idl ${HPP_CORBASERVER_DATAROOTDIR}/idl
  )
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)

MAKE_DIRECTORY(${CMAKE_BINARY_DIR}/src/hpp/agimus_idl)
FOREACH(IDL server discretization)
  GENERATE_IDL_CPP (hpp/agimus_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
    ENABLE_DOCSTRING
    STUBS hpp_stubs.agimus
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=paths:hpp_stubs.core
    )
ENDFOREACH()

GENERATE_IDL_CPP_IMPL (hpp/agimus_idl/discretization ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
  ARGUMENTS
  -Wbguard_prefix=hpp_agimus_idl
  -Wbinc_prefix=hpp/agimus_idl
  HH_SUFFIX -idl.hh)

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/hpp)

INSTALL(
  DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/agimus_idl
  DESTINATION ${PYTHON_SITELIB}/hpp_idl/hpp/)
INSTALL(
  DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/agimus
  DESTINATION ${PYTHON_SITELIB}/hpp_stubs)

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS} ${ALL_IDL_CPP_IMPL_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

IF(BUILD_HPP_PLUGIN)
  HPP_ADD_SERVER_PLUGIN(agimus-hpp
    SOURCES
    server.cc
    discretization.cc
    ${ALL_IDL_CPP_STUBS}
    ${ALL_IDL_CPP_IMPL_STUBS}
    PKG_CONFIG_DEPENDENCIES omniORB4 hpp-corbaserver)

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
