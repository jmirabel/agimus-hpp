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
