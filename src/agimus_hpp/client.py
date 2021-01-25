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
import rospy, CORBA
import hpp.corbaserver
import hpp.corbaserver.tools
import hpp.corbaserver.robot
import hpp.corbaserver.manipulation
import hpp.corbaserver.manipulation.robot
import hpp.gepetto
import hpp.gepetto.manipulation

## Handles connection with HPP servers
#
# It handles connection with hppcorbaserver
class HppClient(object):
    def __init__ (self, context = "corbaserver", connect = True):
        self.context = context
        if connect:
            self._connect()

    def tryConnect(self):
        try:
            self._connect()
            self._hppclient.problem.getAvailable("type")
            return True, ""
        except CORBA.TRANSIENT as err:
            return False, "Could not connect to HPP: " + str(err)

    def setHppUrl (self):
        self._connect()

    def _connect(self):
        self._hppclient = hpp.corbaserver.Client(context = self.context)
        self._hpptools = hpp.corbaserver.tools.Tools ()
        try:
            cl = hpp.corbaserver.manipulation.robot.CorbaClient (context = self.context)
            self._manipclient = cl.manipulation
            self.robot = hpp.corbaserver.manipulation.robot.Robot (client = cl)
            self.problemSolver = hpp.corbaserver.manipulation.ProblemSolver(self.robot)
        except Exception as e:
            rospy.logwarn("Could not connect to manipulation server: " + str(e))
            if hasattr(self, "_manipclient"): delattr(self, "_manipclient")
            self.robot = hpp.corbaserver.robot.Robot(client = self._hppclient)
            self.problemSolver = hpp.corbaserver.ProblemSolver(self.robot)
        rospy.loginfo("Connected to hpp")

    def _disconnect(self):
        delattr(self, "_hppclient")

    ## \deprecated Use HppClient.hpp instead.
    def _hpp (self, reconnect = True):
        return self.hpp (reconnect)

    ## Get the hppcorbaserver client.
    ## It handles reconnection if needed.
    def hpp (self, reconnect = True):
        if not hasattr(self, "_hppclient"):
            if reconnect:
                self._connect()
                reconnect = False
            else:
                rospy.loginfo("Not connected to hpp")
                raise RuntimeError("Not connected to hpp")
        try:
            self._hppclient.problem.getAvailable("type")
        except (CORBA.TRANSIENT, CORBA.COMM_FAILURE) as e:
            if reconnect:
                rospy.loginfo ("Connection with HPP lost. Trying to reconnect.")
                self._connect()
                return self.hpp(False)
            else: raise e
        return self._hppclient

    def hpptools (self, reconnect = True):
        return self._hpptools

    ## \deprecated Use HppClient.hpp instead.
    def _manip (self, reconnect = True):
        return self.manip (reconnect)

    ## Get the hppcorbaserver manipulation client.
    ## It handles reconnection if needed.
    def manip (self, reconnect = True):
        if not hasattr(self, "_manipclient"):
            raise Exception("No manip client")
        try:
            self._manipclient.problem.getAvailable("type")
        except (CORBA.TRANSIENT, CORBA.COMM_FAILURE) as e:
            if reconnect:
                rospy.loginfo ("Connection with HPP lost. Trying to reconnect.")
                self._connect()
                return self.manip(False)
            else: raise e
        return self._manipclient
