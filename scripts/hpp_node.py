#!/usr/bin/env python

# Copyright 2018 CNRS - Airbus SAS
# Authors: Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## \file hpp_node.py
#
# This file starts a ROS node that retrieves paths planned by hppcorbaserver.
#
# Note that manipulation planning should be performed priorly by the
# user. This node only retrieves paths that have already been planned.

import sys, rospy, agimus_hpp.trajectory_publisher as tp

if "hpp-manipulation-server" in sys.argv:
    import agimus_hpp.manipulation.hpp_server_initializer as hsi
    import agimus_hpp.manipulation.planning_request_adapter as pra
    print "Launching manipulation client"
else:
    import agimus_hpp.hpp_server_initializer as hsi
    import agimus_hpp.planning_request_adapter as pra
    print "Launching default client"

rospy.init_node('hpp_server_connection')

_pra = pra.PlanningRequestAdapter ("/joint_states")
_hsi = hsi.HppServerInitializer()
_tp = tp.HppOutputQueue ()

rospy.spin()
