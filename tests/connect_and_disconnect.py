# To run this test, you must launch both hppcorbaserver and roscore

from hpp.corbaserver import loadServerPlugin
from hpp.agimus import Client as AgimusClient
import rosgraph, sys

verbose = "--verbose" in sys.argv

master = rosgraph.Master("")
name = "hpp"

loadServerPlugin('corbaserver', "agimus-hpp.so")

def get_node_uri():
    try:
        return master.lookupNode(name)
    except rosgraph.MasterError as e:
        return None

cl = AgimusClient()
discretization = cl.server.getDiscretization()

if verbose:
    print("HPP uri before initialization: " + str(get_node_uri()))
assert get_node_uri() == None, "HPP was not correctly disconnected from ROS"

success = discretization.initializeRosNode(name, False)
if verbose:
    print("started: " + str(success))
    print("HPP uri after initialization: " + str(get_node_uri()))
assert success, "Failed to initialize ROS node"
assert get_node_uri() != None, "HPP ROS node could not be found"

discretization.shutdownRos()
if verbose:
    print("HPP uri after shutdown: " + str(get_node_uri()))
assert get_node_uri() == None, "HPP could not be disconnected from ROS"
