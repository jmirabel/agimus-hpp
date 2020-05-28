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
import rospy
from agimus_hpp.planning_request_adapter import PlanningRequestAdapter as Parent, _setGaussianShooter
from agimus_sot_msgs.srv import SetString

class PlanningRequestAdapter (Parent):
    servicesDict = {
            "motion_planning": {
                "manipulation" : {
                    'set_robot_prefix': [ SetString, "set_robot_prefix" ],
                    },
                },
            }

    def __init__ (self, topicStateFeedback):
        super(PlanningRequestAdapter, self).__init__(topicStateFeedback)

    def set_robot_prefix (self, req):
        self.robot_name = req.value
        return SetStringResponse(True)

    def _validate_configuration (self, q, collision):
        valid = super(PlanningRequestAdapter, self)._validate_configuration (q, collision)
        if not valid: return False

        from CORBA import UserException
        manip = self.manip ()
        try:
            state_id = manip.graph.getNode (q)
            rospy.loginfo ("Current estimated configuration is in {0}".format(state_id))
            return True
        except UserException as e:
            rospy.logerr ("Configuration is not valid: {0}".format(e))
            return False

    def _set_init_pose (self, msg):
        self.q_init = self.get_object_root_joints()
        # TODO: WHEN NEEDED Get the joint states of the objects.

    def get_object_root_joints(self):
        world_frame = rospy.get_param ("/motion_planning/tf/world_frame_name")
        hpp = self.hpp()
        # Get the name of the objects required by HPP
        root_joints = [el for el in hpp.robot.getAllJointNames() if "root_joint" in el]
        root_joints = [el.split("/")[0] for el in root_joints if not self.robot_name in el]

        for obj in root_joints:
            if self.tfListener.frameExists(obj):
                p, q = self.tfListener.lookupTransform(world_frame, obj, rospy.Time(0))
                hpp.robot.setJointConfig(obj + "/root_joint", p + q)
            else:
                print (obj + " is not published on the TF tree but is " +
                       "needed to plan the trajectory of the robot.")

        return hpp.robot.getCurrentConfig()
