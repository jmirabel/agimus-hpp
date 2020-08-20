#!/usr/bin/env python
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
import rospy, hpp.corbaserver
import agimus_hpp.ros_tools as ros_tools
from tf import TransformListener
from .client import HppClient
from dynamic_graph_bridge_msgs.msg import Vector
from agimus_sot_msgs.msg import ProblemSolved, PlanningGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty, Bool
from math import cos, sin
from threading import Lock
from omniORB import CORBA
import traceback

def _setGaussianShooter (hpp, q, dev):
    hpp.robot.setCurrentConfig (q)
    hpp.problem.setParameter ("ConfigurationShooter/Gaussian/standardDeviation",
            CORBA.Any(CORBA.TC_double, dev))
    hpp.problem.selectConfigurationShooter ("Gaussian")

## Class that handles ROS motion planning requests and forward them to HPP.
#
# This class takes care of:
# - setting the initial configuration of the planning problem
#   according to a policy. See PlanningRequestAdapter.modes.
# - request the resolution of a planning problem.
#
# This class *does not* take care of:
# - initializing the problem (loading the robot, the environment,)
# - defining the goal.
#
# Connection with HPP is handle throw agimus_hpp.client.HppClient.
class PlanningRequestAdapter(HppClient):
    subscribersDict = {
            "motion_planning": {
                "set_goal" : [PlanningGoal, "set_goal" ],
                "request" : [Empty, "request" ],
                "param" : {
                    'init_position_mode': [ String, "init_position_mode" ],
                    'set_init_pose': [ PlanningGoal, "set_init_pose" ],
                    },
                },
            }
    publishersDict = {
            "motion_planning": {
                "problem_solved" : [ ProblemSolved, 1],
                },
            }
    ## Mode to set the initial configuration of the planning problem.
    # There are three modes:
    # - \c current: The current robot configuration, acquired from the PlanningRequestAdapter.topicStateFeedback.
    #               The base pose is computed using tf and ROS parameter
    #               \c /motion_planning/tf/world_frame_name
    # - \c estimated: The robot configuration, acquired from the PlanningRequestAdapter.topicEstimation
    # - \c uesr_defined: The value passed with topic \c /motion_planning/param/set_init_pose
    modes = [ "current", "estimated", "user_defined" ]

    def __init__ (self, topicStateFeedback):
        super(PlanningRequestAdapter, self).__init__ (connect=False)
        self.subscribers = ros_tools.createSubscribers (self, "/agimus", self.subscribersDict)
        self.publishers = ros_tools.createPublishers ("/agimus", self.publishersDict)

        self.topicStateFeedback = topicStateFeedback
        self.topicEstimation = "/agimus/estimation/semantic"
        self.q_init = None
        self.init_mode = "user_defined"
        self.get_current_state = None
        self.get_estimation = None
        self.tfListener = TransformListener()
        self.mutexSolve = Lock()
        if not rospy.has_param ("/motion_planning/tf/world_frame_name"):
            rospy.set_param ("/motion_planning/tf/world_frame_name", "world")
        self.robot_name = ""
        self.robot_base_frame = None

    def hpp (self, reconnect = True):
        hpp = super(PlanningRequestAdapter, self).hpp(reconnect)
        rjn = hpp.robot.getAllJointNames()[1]
        try:
          self.robot_name = rjn[:rjn.index('/')+1]
        except ValueError:
          self.robot_name = ""
        self.robot_base_frame = hpp.robot.getLinkNames(self.robot_name + "root_joint")[0]
        rootJointType = hpp.robot.getJointType(self.robot_name + "root_joint").lower()
        if rootJointType == "anchor":
            self.setRootJointConfig = lambda x : None
        elif rootJointType == "jointmodelfreeflyer":
            self.setRootJointConfig = lambda x : hpp.robot.setJointConfig(self.robot_name + "root_joint", x)
        elif rootJointType == "jointmodelplanar":
            self.setRootJointConfig = lambda x : hpp.robot.setJointConfig(self.robot_name + "root_joint", x[0:2] + [x[6]**2 - x[5]**2, 2 * x[5] * x[6]] )
        else:
            self.setRootJointConfig = lambda x : (_ for _ in ()).throw(Exception("Root joint type is not understood. It must be one of (anchor, freeflyer, anchor) and not " + str(rootJointType)))
        return hpp

    def _JointStateToConfig(self, placement, js_msg):
        hpp = self.hpp()
        if self.q_init is not None:
            hpp.robot.setCurrentConfig(self.q_init)
        self.setRootJointConfig(placement)
        for jn, q in zip(js_msg.name, js_msg.position):
            size = hpp.robot.getJointConfigSize(self.robot_name + jn)
            if size == 2:
                hpp.robot.setJointConfig(self.robot_name + jn, [cos(q), sin(q)])
            else:
                hpp.robot.setJointConfig(self.robot_name + jn, [q])
        return hpp.robot.getCurrentConfig()

    def set_goal (self, msg):
        hpp = self.hpp()
        q_goal = self._JointStateToConfig(msg.base_placement, msg.joint_state)
        hpp.problem.resetGoalConfigs()
        hpp.problem.addGoalConfig(q_goal)

    def request (self, msg):
        self.mutexSolve.acquire()
        try:
            if self.init_mode == "current":
                self.set_init_pose (PlanningGoal(self.last_placement, self.last_joint_state))
            elif self.init_mode == "estimated":
                self.q_init = self.estimated_config
            hpp = self.hpp()
            self._validate_configuration (self.q_init, collision = True)
            rospy.loginfo("init done")
            rospy.loginfo(str(self.q_init))
            hpp.problem.setInitialConfig(self.q_init)
            rospy.loginfo("configured")
            t = hpp.problem.solve()
            rospy.loginfo("solved")
            pid = hpp.problem.numberPaths() - 1
            time = t[0] * 3600 + t[1] * 60 + t[2] + t[3] * 1e-3
            # print "Solved in", t, ", path id", pid
            rospy.loginfo("Path ({}) to reach target found in {} seconds".format(pid, t))
            rospy.sleep(0.1)
            self.publishers["motion_planning"]["problem_solved"].publish (ProblemSolved(True, "success", pid))
        except Exception as e:
            rospy.loginfo (str(e))
            rospy.loginfo (traceback.format_exc())
            rospy.sleep(0.1)
            self.publishers["motion_planning"]["problem_solved"].publish (ProblemSolved(False, str(e), -1))
        finally:
            self.mutexSolve.release()

    def _validate_configuration (self, q, collision):
        hpp = self.hpp()
        if len(q) != hpp.robot.getConfigSize ():
            rospy.logerr ("Configuration size mismatch: got {0} expected {1}".format(len(q), hpp.robot.getConfigSize ()))
            return False
        if collision:
            valid, msg = hpp.robot.isConfigValid (q)
            if not valid:
                rospy.logerr ("Configuration is not valid: {0}".format(msg))
                return False
        # TODO in manipulation, check that it has a state.
        return True

    def estimation_acquisition (self, cfg):
        self.estimated_config = cfg.data

    def init_position_mode(self, msg):
        if msg.data in self.modes:
            if msg.data == self.init_mode: return
            self.init_mode = msg.data
            rospy.loginfo("Initial position mode: %s" % msg.data)
            if self.get_current_state is not None:
                self.get_current_state.unregister()
            self.get_current_state = None
            if self.get_estimation is not None:
                self.get_estimation.unregister()
            self.get_estimation = None
            if msg.data == "current":
                self.get_current_state = rospy.Subscriber (self.topicStateFeedback, JointState, self.get_joint_state)
            elif msg.data == "estimated":
                self.get_estimation = rospy.Subscriber (self.topicEstimation, Vector, self.estimation_acquisition)

    def get_joint_state (self, msg):
        self.last_joint_state = msg
        try:
            world_frame = rospy.get_param ("/motion_planning/tf/world_frame_name")
            base = "base_link"
            p, q = self.tfListener.lookupTransform(world_frame, base, rospy.Time(0))
            self.last_placement = p + q
        except Exception as e:
            rospy.loginfo( str(e) )
            pass

    def set_init_pose(self, msg):
        self.q_init = self._JointStateToConfig(msg.base_placement, msg.joint_state)
        self._set_init_pose (msg)

    def _set_init_pose (self, msg):
        """ To allow reimplementation """
        pass
