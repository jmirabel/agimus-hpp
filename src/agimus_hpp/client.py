import rospy, CORBA
import hpp.corbaserver
import hpp.corbaserver.tools
import hpp.corbaserver.robot
import hpp.corbaserver.manipulation
import hpp.corbaserver.manipulation.robot
import hpp.gepetto
import hpp.gepetto.manipulation
import ros_tools

## Handles connection with HPP servers
#
# It handles connection with hppcorbaserver
class HppClient(object):
    def __init__ (self, context = "corbaserver"):
        self.context = context
        self.setHppUrl()

    def setHppUrl (self):
        self._connect()

    def _connect(self):
        self._hppclient = hpp.corbaserver.Client(context = self.context)
        self.hpp_tools = hpp.corbaserver.tools.Tools ()
        try:
            cl = hpp.corbaserver.manipulation.robot.CorbaClient (context = self.context)
            self._manipclient = cl.manipulation
            self.robot = hpp.corbaserver.manipulation.robot.Robot (client = cl)
            self.problemSolver = hpp.corbaserver.manipulation.ProblemSolver(self.robot)
        except Exception, e:
            rospy.logwarn("Could not connect to manipulation server: " + str(e))
            if hasattr(self, "_manipclient"): delattr(self, "_manipclient")
            self.robot = hpp.corbaserver.robot.Robot(client = self._hppclient)
            self.problemSolver = hpp.corbaserver.ProblemSolver(self.robot)
        rospy.loginfo("Connected to hpp")

    ## \deprecated Use HppClient.hpp instead.
    def _hpp (self, reconnect = True):
        return self.hpp (reconnect)

    ## Get the hppcorbaserver client.
    ## It handles reconnection if needed.
    def hpp (self, reconnect = True):
        try:
            self._hppclient.problem.getAvailable("type")
        except (CORBA.TRANSIENT, CORBA.COMM_FAILURE) as e:
            if reconnect:
                rospy.loginfo ("Connection with HPP lost. Trying to reconnect.")
                self._connect()
                return self.hpp(False)
            else: raise e
        return self._hppclient

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
