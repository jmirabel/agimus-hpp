#!/usr/bin/env python
import rospy, hpp.corbaserver
import numpy as np
from .client import HppClient
from agimus_sot_msgs.msg import *
from agimus_sot_msgs.srv import *
import ros_tools
from .tools import *
import Queue
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

    class Topic (object):
        def __init__ (self, reader, topicPub, MsgType, data = None):
            self.reader = reader
            self.pub = rospy.Publisher("/hpp/target/" + topicPub, MsgType, latch=False, queue_size=1000)
            self.MsgType = MsgType
            self.data = data

        def init (self, hpp):
            pass

        def read (self, hpp):
            return self.reader(hpp, self.data)

        def publish (self, msg):
            self.pub.publish(msg)
    class ConstantTopic (object):
        def __init__ (self, value, topicPub, MsgType):
            self.pub = rospy.Publisher("/hpp/target/" + topicPub, MsgType, latch=False, queue_size=1000)
            self.msg = value

        def init (self, hpp):
            pass

        def read (self, hpp):
            return None

        def publish (self, msg):
            assert msg==None
            self.pub.publish(self.msg)

    def __init__ (self):
        super(HppOutputQueue, self).__init__ ()

        ## Publication frequency
        self.dt = rospy.get_param ("/sot_controller/dt")
        self.frequency = 1. / self.dt # Hz
        ## Queue size should be adapted according to the queue size in SoT
        self.queue_size = 1024
        self.queue = Queue.Queue (self.queue_size)

        self.setJointNames (SetJointNamesRequest(self._hpp().robot.getJointNames()))

        self.subscribers = ros_tools.createSubscribers (self, "", self.subscribersDict)
        self.services = ros_tools.createServices (self, "", self.servicesDict)
        self.pubs = ros_tools.createPublishers ("/hpp/target", self.publishersDist)
        self.reading = False
        self.firstMsgs = None

        self.resetTopics ()

    def resetTopics (self, msg = None):
        self.topics = [
                self.Topic (self._readConfigAtParam  , "position", Vector),
                self.Topic (self._readVelocityAtParam, "velocity", Vector),
                ]
        hpp = self._hpp()
        self.topics[0].init(hpp)
        self.topics[1].init(hpp)
        rospy.loginfo("Reset topics")
        if msg is not None:
            return std_srvs.srv.EmptyResponse()

    def addCenterOfMass (self, req):
        # TODO check that com exists
        comName = req.value
        n = "com"
        if comName != "":
            n += "/" + comName
        self.topics.append (
                self.Topic (self._readCenterOfMass, n, Vector3, data = comName),
                )
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n)
        return SetStringResponse(True)

    def addCenterOfMassVelocity (self, req):
        # TODO check that com exists
        comName = req.value
        n = "velocity/com"
        if comName != "":
            n += "/" + comName
        self.topics.append (
                self.Topic (self._readCenterOfMassVelocity, n, Vector3, data = comName),
                )
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n)
        return SetStringResponse(True)

    def _getFrameType (self, n):
        _hpp = self._hpp()
        try:
            _hpp.robot.getJointPosition (n)
            return "joint"
        # except hpp.Error:
        except:
            pass
        try:
            _hpp.robot.getLinkPosition (n)
            return "link"
        # except hpp.Error:
        except:
            pass
        try:
            _hpp.obstacle.getObstaclePosition (n)
            return "obstacle"
        # except hpp.Error:
        except:
            pass
        raise ValueError ("Unknown operational frame type of " + n)

    def addOperationalFrame (self, req):
        # TODO check that frame exists
        n = "op_frame/" + req.value
        try:
            frameType = self._getFrameType (req.value)
        except ValueError as e:
            rospy.logerr("Could not add operational frame: " + str(e))
            return SetStringResponse(False)
        if frameType == "joint":
            self.topics.append (self.Topic (self._readJointPosition, n, Transform, data = req.value))
        elif frameType == "link":
            self.topics.append (self.Topic (self._readLinkPosition, n, Transform, data = req.value))
        elif frameType == "obstacle":
            # TODO There should be a way for the node who requests this
            # to know the value is constant.
            _hpp = self._hpp()
            pos = _hpp.obstacle.getObstaclePosition (req.value)
            self.topics.append (self.ConstantTopic (listToTransform(pos), n, Transform))
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n + " " + frameType)
        return SetStringResponse(True)

    def addOperationalFrameVelocity (self, req):
        # TODO check that frame exists
        n = "velocity/op_frame/" + req.value
        try:
            frameType = self._getFrameType (req.value)
        except ValueError as e:
            rospy.logerr("Could not add operational frame: " + str(e))
            return SetStringResponse(False)
        if frameType == "joint":
            self.topics.append (self.Topic (self._readJointVelocity, n, Vector, data = req.value))
        elif frameType == "link":
            self.topics.append (self.Topic (self._readLinkVelocity, n, Vector, data = req.value))
        elif frameType == "obstacle":
            # TODO There should be a way for the node who requests this
            # to know the value is constant.
            self.topics.append (self.ConstantTopic ([0,0,0,0,0,0], n, Vector))
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n + " " + frameType)
        return SetStringResponse(True)

    def setJointNames (self, req):
        try:
            hpp = self._hpp()
            jns = hpp.robot.getJointNames() + [None]
            # list of segments in [config, velocity]
            joint_selection = [ [], [] ]
            # rank in [config, velocity]
            rks = [0, 0]
            segments = None
            for jn in jns:
                szs = [hpp.robot.getJointConfigSize(jn), hpp.robot.getJointNumberDof(jn)] if jn is not None else [0,0]
                if jn in req.names:
                    if segments is None:
                        segments = [ [rks[0], rks[0] + szs[0]], [rks[1], rks[1] + szs[1]] ]
                    else:
                        for i in range(2): segments[i][1] += szs[i]
                else:
                    if segments is not None: # insert previous segments
                        joint_selection[0].append(segments[0])
                        joint_selection[1].append(segments[1])
                        segments = None
                for i in range(2): rks[i] += szs[i]
            self.jointNames = req.names
            self.joint_selection = joint_selection
        except:
            return SetJointNamesResponse(False)
        rospy.loginfo("Joint names set to " + str(self.jointNames))
        self.rootJointName = None
        self.rootJointSizes = 0
        for n in self.jointNames:
            if n.endswith("root_joint"):
                self.rootJointName = n
                self.rootJointSizes = ( hpp.robot.getJointConfigSize(n), hpp.robot.getJointNumberDof(n) )
                break
        return SetJointNamesResponse(True)

    def _readConfigAtParam (self, client, data):
        qin = client.robot.getCurrentConfig()
        qout = list()
        for segment in self.joint_selection[0]:
            qout.extend(qin[segment[0]:segment[1]])
        if self.rootJointName is not None:
            rootpos = client.robot.getJointPosition(self.rootJointName)
            # TODO although it is weird, the root joint may not be at
            # position 0
            qout[0:self.rootJointSizes[0]] = hppPoseToSotTransRPY (rootpos[0:7])
        return Vector(qout)

    def _readVelocityAtParam (self, client, data):
        vin = client.robot.getCurrentVelocity()
        vout = list()
        for segment in self.joint_selection[1]:
            vout.extend(vin[segment[0]:segment[1]])
        if self.rootJointName is not None:
            rootvel = client.robot.getJointVelocity(self.rootJointName)
            # TODO although it is weird, the root joint may not be at
            # position 0
            vout[0:self.rootJointSizes[1]] = rootvel
        return Vector(vout)

    def _readCenterOfMass (self, client, data):
        if data == "":
            v = client.robot.getCenterOfMass()
        else:
            v = client.robot.getPartialCom(data)
        return listToVector3(v)

    def _readCenterOfMassVelocity (self, client, data):
        if data == "":
            v = client.robot.getCenterOfMassVelocity()
        else:
            v = client.robot.getVelocityPartialCom(data)
        return listToVector3(v)

    def _readJointPosition (self, client, data):
        t = client.robot.getJointPosition(data)
        return listToTransform(t)

    def _readJointVelocity (self, client, data):
        t = client.robot.getJointVelocityInLocalFrame(data)
        return Vector(t)

    def _readLinkPosition (self, client, data):
        t = client.robot.getLinkPosition(data)
        return listToTransform(t)

    def _readLinkVelocity (self, client, data):
        rospy.logerr ("Link velocity cannot be obtained from HPP")
        return Vector()

    ## Compute the configuration and velocity of the robot along \c path
    #  at time \c time.
    # \param path the path,
    # \param time along the path,
    # \param timeShift unused. I forgot the meaning.
    # \note The derivatives are computed using \f$ \frac{q(t+dt) - q(t)}{dt} \f$
    # \todo At the moment, HPP does not return the correct path derivatives because
    #       class StraightPath uses hpp::pinocchio::RnxSOnLieGroupMap and not
    #       hpp::pinocchio::DefaultLieGroupMap.
    def readAt (self, path, time, timeShift = 0):
        hpp = self._hpp()
        qt, success = path.call(time)
        hpp.robot.setCurrentConfig( qt )
        #hpp.robot.setCurrentVelocity( path.derivative (time, 1))
        device = hpp.problem.getProblem().robot()
        if time+self.dt > path.length():
            qt_dt, success = path.call (path.length())
            dt = path.length() - time
            if dt == 0:
                vel = [ 0., ] * hpp.robot.getNumberDof()
            else:
                vel = [ v/dt for v in device.difference (qt_dt, qt) ]
        else:
            qt_dt, success = path.call (time+self.dt)
            vel = [ v/self.dt for v in device.difference (qt_dt, qt) ]
        hpp.robot.setCurrentVelocity( vel )
        msgs = []
        for topic in self.topics:
            msgs.append (topic.read(hpp))
        self.queue.put (msgs, True)
        return msgs

    def publishNext (self):
        msgs = self.queue.get(True)
        for topic, msg in zip(self.topics, msgs):
            topic.publish (msg)
        self.queue.task_done()

    def _read (self, pathId, start, L):
        from math import ceil, floor
        N = int(ceil(abs(L) * self.frequency))
        rospy.loginfo("Start reading path {} (t in [ {}, {} ]) into {} points".format(pathId, start, start + L, N+1))
        self.reading = True
        self.queue = Queue.Queue (self.queue_size)
        times = (-1 if L < 0 else 1 ) *np.array(range(N+1), dtype=float) / self.frequency
        times[-1] = L
        times += start
        self.firstMsgs = None
        hpp = self._hpp()
        path = hpp.problem.getPath(pathId)
        for t in times:
            msgs = self.readAt(path, t, timeShift = start)
            if self.firstMsgs is None: self.firstMsgs = msgs
        self.hpp_tools.deleteServantFromObject (path)
        self.pubs["read_path_done"].publish(UInt32(pathId))
        rospy.loginfo("Finish reading path {}".format(pathId))
        self.reading = False

    def read (self, msg):
        pathId = msg.data
        hpp = self._hpp()
        L = hpp.problem.pathLength(pathId)
        self._read (pathId, 0, L)

    def readSub (self, msg):
        self._read (msg.id, msg.start, msg.length)

    def publishFirst(self, trigger):
        count = 1000
        rate = rospy.Rate (count)
        if self.firstMsgs is None:
            rospy.logwarn ("First message not ready yet. Keep trying during one second.")
        while self.firstMsgs is None and count > 0:
            rate.sleep()
            count -= 1
        if self.firstMsgs is None:
            rospy.logerr("Could not print first message")
            return False, "First message not ready yet. Did you call read_path ?"

        for topic, msg in zip(self.topics, self.firstMsgs):
            topic.publish (msg)
        self.firstMsgs = None
        return True, ""

    def publish(self, empty):
        rospy.loginfo("Start publishing queue (size is {})".format(self.queue.qsize()))
        # The queue in SOT should have about 100ms of points
        n = 0
        advance = 0.150 * self.frequency # Begin with 150ms of points
        start = rospy.Time.now()
        # highrate = rospy.Rate (5 * self.frequency)
        rate = rospy.Rate (10) # Send 100ms every 100ms
        while not self.queue.empty() or self.reading:
            dt = (rospy.Time.now() - start).to_sec()
            nstar = advance + dt * self.frequency
            while n < nstar and not self.queue.empty():
                self.publishNext()
                n += 1
                # highrate.sleep()
            rate.sleep()
        self.pubs["publish_done"].publish(Empty())
        rospy.loginfo("Finish publishing queue ({})".format(n))

    def getQueueSize (self, empty):
        return self.queue.qsize()
