/** \mainpage

  Communication between HPP and Agimus.

  This package implements a ROS node through script hpp_node.py. The script
  creates one instance of the 3 following classes:

  \li \link agimus_hpp.trajectory_publisher.HppOutputQueue HppOutputQueue
  \endlink
  \li \link agimus_hpp.estimation.Estimation  Estimation \endlink
  \li \link agimus_hpp.planning_request_adapter.PlanningRequestAdapter
  PlanningRequestAdapter \endlink

  \link agimus_hpp.trajectory_publisher.HppOutputQueue HppOutputQueue
  \endlink
  initializes a client to \c hppcorbaserver and samples reference trajectories
  of various tasks in topics. For instance
  \li \c /hpp/target/op_fame/talos/arm_left_7_joint for the reference position
      of joint \c arm_left_7_joint of Talos robot,
  \li \c /hpp/target/velocity/op_fame/talos/arm_left_7_joint for the reference
      velocity of joint \c arm_left_7_joint,
  \li \c /hpp/target/position for the reference value of the posture task,
  \li \c /hpp/target/velocity for the reference velocity of the posture,
  \li \c /hpp/target/com/talos for the reference trajectory of the center
      of mass,
  \li \c /hpp/target/velocity/com/talos for the reference velocity of the center of mass.

  \link agimus_hpp.estimation.Estimation  Estimation \endlink computes an
  estimation of the robot and object configurations from vision and encoders.

  \link agimus_hpp.planning_request_adapter.PlanningRequestAdapter
  PlanningRequestAdapter \endlink initializes a client to \c hppcorbaserver
  and send motion planning requests.
*/
