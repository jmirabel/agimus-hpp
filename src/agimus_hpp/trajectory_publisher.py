#!/usr/bin/env python
import rospy, hpp.corbaserver
import numpy as np
from .client import HppClient
from agimus_sot_msgs.msg import *
from agimus_sot_msgs.srv import *
import ros_tools
from .tools import *
from dynamic_graph_bridge_msgs.msg import Vector
from geometry_msgs.msg import Vector3, Transform
from std_msgs.msg import UInt32, Empty
import std_srvs.srv

## Samples and publishes a path from HPP into several topics
##
## References to be published are:
## \li robot configuration and velocity (by default, can be parameterized via a service),
## \li COM position and velocity (requested via a service),
## \li joint position and velocity (requested via a service),
## \li link position and velocity (requested via a service),
## \li frame position and velocity (requested via a service).
##
## Connection with HPP is handle throw agimus_hpp.client.HppClient.
class HppOutputQueue(HppClient):
    ## Subscribed topics
    subscribersDict = {
            "hpp": {
                "target": {
                    "read_path": [ UInt32, "read" ],
                    "read_subpath": [ ReadSubPath, "readSub" ],
                    "publish": [ Empty, "publish" ]
                    },
                },
            }
    ## Published topics (prefixed by "/hpp/target")
    publishersDist = {
            "read_path_done": [ UInt32, 1 ],
            "publish_done": [ Empty, 1 ]
            }
    ## Provided services
    servicesDict = {
            "hpp": {
                "target": {
                    "set_joint_names": [ SetJointNames, "setJointNames", ],
                    "reset_topics": [ std_srvs.srv.Empty, "resetTopics", ],
                    "add_center_of_mass": [ SetString, "addCenterOfMass", ],
                    "add_operational_frame": [ SetString, "addOperationalFrame", ],
                    "add_center_of_mass_velocity": [ SetString, "addCenterOfMassVelocity", ],
                    "add_operational_frame_velocity": [ SetString, "addOperationalFrameVelocity", ],

                    "publish_first": [ std_srvs.srv.Trigger, "publishFirst", ],
                    "get_queue_size": [ GetInt, "getQueueSize", ],
                    }
                }
            }

    def __init__ (self):
        self.discretization = None
        rospy.on_shutdown (self._ros_shutdown)

        super(HppOutputQueue, self).__init__ ()

        ## Publication frequency
        self.dt = rospy.get_param ("/sot_controller/dt")
        self.frequency = 1. / self.dt # Hz

        self.resetTopics ()
        self.setJointNames (SetJointNamesRequest(self.hpp().robot.getJointNames()))

        self.subscribers = ros_tools.createSubscribers (self, "", self.subscribersDict)
        self.services = ros_tools.createServices (self, "", self.servicesDict)
        self.pubs = ros_tools.createPublishers ("/hpp/target", self.publishersDist)

        self.times = None

    def _connect (self):
        super(HppOutputQueue, self)._connect ()
        from hpp.corbaserver.tools import loadServerPlugin
        loadServerPlugin (self.context, "agimus-hpp.so")
        from hpp.agimus import Client
        self._agimus = Client(context=self.context)
        if self.discretization is None:
            self.discretization = self._agimus.server.getDiscretization()
            self.discretization.initializeRosNode ("hpp_server", True)
        else:
            try:
                self.discretization.initializeRosNode ("hpp_server", True)
            except:
                self.discretization = self._agimus.server.getDiscretization()
                self.discretization.initializeRosNode ("hpp_server", True)

    def _ros_shutdown(self):
        if self.discretization is not None:
            self.discretization.shutdownRos()
            self.hpptools().deleteServantFromObject (self.discretization)

    def resetTopics (self, msg = None):
        self.hpp()
        self.discretization.resetTopics()
        rospy.loginfo("Reset topics")
        if msg is not None:
            return std_srvs.srv.EmptyResponse()

    def addCenterOfMass (self, req):
        from hpp_idl.hpp.agimus_idl import Discretization
        try:
            hpp = self.hpp()
            comcomp = hpp.robot.getCenterOfMassComputation (req.value)
            self.discretization.addCenterOfMass (req.value, comcomp, Discretization.Position)
            self.hpptools().deleteServantFromObject (comcomp)
        except Exception as e:
            rospy.logerr("Could not add COM position: " + str(e))
            return False
        rospy.loginfo("Add COM position topic " + req.value)
        return True

    def addCenterOfMassVelocity (self, req):
        from hpp_idl.hpp.agimus_idl import Discretization
        try:
            hpp = self.hpp()
            comcomp = hpp.robot.getCenterOfMassComputation (req.value)
            self.discretization.addCenterOfMass (req.value, comcomp, Discretization.Derivative)
            self.hpptools().deleteServantFromObject (comcomp)
        except Exception as e:
            rospy.logerr("Could not add COM velocity: " + str(e))
            return False
        rospy.loginfo("Add COM velocity topic " + req.value)
        return True

    def addOperationalFrame (self, req):
        from hpp_idl.hpp.agimus_idl import Discretization
        try:
            self.discretization.addOperationalFrame (req.value, Discretization.Position)
        except Exception as e:
            rospy.logerr("Could not add operational frame pose: " + str(e))
            return False
        rospy.loginfo("Add operational frame pose topic " + req.value)
        return True

    def addOperationalFrameVelocity (self, req):
        from hpp_idl.hpp.agimus_idl import Discretization
        try:
            self.discretization.addOperationalFrame (req.value, Discretization.Derivative)
        except Exception as e:
            rospy.logerr("Could not add operational frame velocity: " + str(e))
            return False
        rospy.loginfo("Add operational frame velocity topic " + req.value)
        return True

    def setJointNames (self, req):
        try:
            hpp = self.hpp()
            # TODO at the moment, the root joint is considered to be always added.
            names = [ n for n in req.names if "root_joint" not in n ]
            self.discretization.setJointNames (names)
        except Exception as e:
            rospy.logerr("Could not set joint names: " + str(e))
            return False
        return True

    def _read (self, pathId, start, L):
        from math import ceil, floor, sqrt
        N = int(ceil(abs(L) * self.frequency))
        rospy.loginfo("Prepare sampling of path {} (t in [ {}, {} ]) into {} points".format(pathId, start, start + L, N+1))

        times = (-1 if L < 0 else 1 ) *np.array(range(N+1), dtype=float) / self.frequency
        times[-1] = L
        times += start

        hpp = self.hpp()
        path = hpp.problem.getPath(pathId)
        self.discretization.setPath (path)
        self.hpptools().deleteServantFromObject (path)

        self.times = times

    def read (self, msg):
        pathId = msg.data
        hpp = self.hpp()
        L = hpp.problem.pathLength(pathId)
        self._read (pathId, 0, L)

    def readSub (self, msg):
        self._read (msg.id, msg.start, msg.length)

    def publishFirst(self, trigger):
        count = 1000
        rate = rospy.Rate (count)
        if self.times is None:
            rospy.logwarn ("First message not ready yet. Keep trying during one second.")
        while self.times is None and count > 0:
            rate.sleep()
            count -= 1
        if self.times is None:
            rospy.logerr("Could not print first message")
            return False, "First message not ready yet. Did you call read_path ?"

        self.discretization.compute (self.times[0])
        return True, ""

    def publish(self, empty):
        rospy.loginfo("Start publishing path (size is {})".format(len(self.times)))
        # The queue in SOT should have about 100ms of points
        n = 0
        advance = 0.150 * self.frequency # Begin with 150ms of points
        nstar = min(advance, len(self.times))
        start = rospy.Time.now()
        rate = rospy.Rate (100) # Send 10ms every 10ms
        computation_time = rospy.Duration()
        now = rospy.Time.now()
        while n < len(self.times):
            if n < nstar:
                prev = rospy.Time.now()
                self.discretization.compute (self.times[n])
                now = rospy.Time.now()
                computation_time += now - prev
                n += 1
            else:
                rate.sleep()
                now = rospy.Time.now()
            t = (now - start).to_sec()
            nstar = min(advance + t * self.frequency, len(self.times))

        avg = computation_time.to_sec()/n
        if self.dt <= avg:
            rospy.logwarn("The average sampling time of the reference trajectory ({}) is higher than the execution time ({}). Consider subsampling or preprocessing.".format(avg, self.dt))
        self.times = None
        self.pubs["publish_done"].publish(Empty())
        rospy.loginfo("Finish publishing queue ({})".format(n))

    ## \todo rename this service in get_number_of_points.
    #        This information could also be returned by read and readSub.
    def getQueueSize (self, empty):
        return len(self.times)
