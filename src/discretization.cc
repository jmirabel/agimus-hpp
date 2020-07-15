// Copyright (c) 2018, 2019, 2020 CNRS and Airbus S.A.S
// Author: Joseph Mirabel
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <hpp/agimus/discretization.hh>

#include <pinocchio/algorithm/frames.hpp>
#include <hpp/util/timer.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <geometry_msgs/Transform.h>
#include <dynamic_graph_bridge_msgs/Vector.h>

namespace agimusHpp {
    using namespace hpp;
    namespace pinocchio = hpp::pinocchio;

    HPP_DEFINE_TIMECOUNTER(discretization);

    static const uint32_t queue_size = 1000;

    void Discretization::COM::compute (hpp::pinocchio::DeviceData& d)
    {
      switch (option) {
        case Position:
          com->compute (d, hpp::pinocchio::COM);
          break;
        case Derivative:
          com->compute (d, hpp::pinocchio::VELOCITY);
          break;
        case PositionAndDerivative:
          com->compute (d, hpp::pinocchio::COMPUTE_ALL);
          break;
      }
    }

    void Discretization::COM::initPublishers (const std::string& prefix,
        const std::string& name, ros::NodeHandle& nh)
    {
      if (option&Position)
        pubQ = nh.advertise <geometry_msgs::Vector3> (
            prefix + "com/" + name,
            queue_size, false);
      if (option&Derivative)
        pubV = nh.advertise <geometry_msgs::Vector3> (
            prefix + "velocity/com/" + name,
            queue_size, false);
    }

    void Discretization::FrameData::initPublishers (const std::string& prefix,
        const std::string& name, ros::NodeHandle& nh)
    {
      if (option&Position)
        pubQ = nh.advertise <geometry_msgs::Transform> (
            prefix + "op_frame/" + name,
            queue_size, false);
      if (option&Derivative)
        pubV = nh.advertise <dynamic_graph_bridge_msgs::Vector> (
            prefix + "velocity/op_frame/" + name,
            queue_size, false);
    }

    Discretization::~Discretization ()
    {
      shutdownRos();
    }

    void Discretization::compute (value_type time)
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!path_)
        throw std::logic_error ("Path is not set");
      HPP_START_TIMECOUNTER(discretization);

      q_.resize(device_->configSize());
      v_.resize(device_->numberDof ());

      bool success = path_->eval (q_, time);
      if (!success)
        throw std::runtime_error ("Could not evaluate the path");
      path_->derivative (v_, time, 1);

      hpp::pinocchio::DeviceSync device (device_);
      device.currentConfiguration(q_);
      device.currentVelocity     (v_);
      device.computeFramesForwardKinematics();

      dynamic_graph_bridge_msgs::Vector qmsgs;

      qmsgs.data.resize(qView_.nbIndices()+6);
      Eigen::Map<pinocchio::vector_t> (qmsgs.data.data()+6, qView_.nbIndices()) = qView_.rview(q_);
      { // Set root joint position
        // TODO at the moment, we must convert the quaternion into RPY values.
        const pinocchio::SE3& oMrj = device.data().oMi[1];
        Eigen::Map<pinocchio::vector3_t> (qmsgs.data.data()  ) = oMrj.translation();
        Eigen::Map<pinocchio::vector3_t> (qmsgs.data.data()+3) = oMrj.rotation().eulerAngles (2, 1, 0);
      }
      pubQ.publish (qmsgs);

      qmsgs.data.resize(vView_.nbIndices()+6);
      Eigen::Map<pinocchio::vector_t> (qmsgs.data.data()+6, vView_.nbIndices()) = vView_.rview(v_);
      { // TODO Set root joint velocity
        Eigen::Map<pinocchio::vector_t> (qmsgs.data.data(), 6).setZero();
      }
      pubV.publish (qmsgs);

      for (std::size_t i = 0; i < frames_.size(); ++i) {
        FrameData& frame = frames_[i];
        if (frame.option&Position)
        {
          const pinocchio::SE3& oMf = device.data().oMf[frame.index];
          geometry_msgs::Transform M;
          M.translation.x = oMf.translation()[0];
          M.translation.y = oMf.translation()[1];
          M.translation.z = oMf.translation()[2];
          pinocchio::SE3::Quaternion q (oMf.rotation());
          M.rotation.w = q.w();
          M.rotation.x = q.x();
          M.rotation.y = q.y();
          M.rotation.z = q.z();
          frame.pubQ.publish (M);
        }
        if (frame.option&Derivative)
        {
          dynamic_graph_bridge_msgs::Vector velocity;
          velocity.data.resize(6);
          Eigen::Map<Eigen::Matrix<pinocchio::value_type, 6, 1> > v (velocity.data.data());
          v = ::pinocchio::getFrameVelocity (device.model(), device.data(), frame.index).toVector();
          frame.pubV.publish (velocity);
        }
      }

      for (std::size_t i = 0; i < coms_.size(); ++i) {
        COM& com = coms_[i];
        com.compute(device.d());
        // publish it.
        if (com.option & Position) {
          pinocchio::vector3_t v (coms_[i].com->com (device.d()));
          geometry_msgs::Vector3 msg;
          msg.x = v[0];
          msg.y = v[1];
          msg.z = v[2];
          com.pubQ.publish (msg);
        }
        if (com.option & Derivative) {
          const pinocchio::vector3_t& v (coms_[i].com->jacobian (device.d()) * v_);
          geometry_msgs::Vector3 msg;
          msg.x = v[0];
          msg.y = v[1];
          msg.z = v[2];
          com.pubV.publish (msg);
        }
      }

      HPP_STOP_TIMECOUNTER(discretization);
      HPP_DISPLAY_LAST_TIMECOUNTER(discretization);
      HPP_DISPLAY_TIMECOUNTER(discretization);
    }

    bool Discretization::addCenterOfMass (const std::string& name,
        const CenterOfMassComputationPtr_t& c, ComputationOption option)
    {
      if (!handle_)
        throw std::logic_error ("Initialize ROS first");

      for (std::size_t i = 0; i < coms_.size(); ++i)
        if (coms_[i].com == c) {
          coms_[i].option = (ComputationOption)(coms_[i].option | option);
          coms_[i].initPublishers (topicPrefix_, name, *handle_);
          return true;
        }

      coms_.push_back(COM(c, option));
      coms_.back().initPublishers (topicPrefix_, name, *handle_);
      return true;
    }

    bool Discretization::addOperationalFrame (
        const std::string& name, ComputationOption option)
    {
      if (!handle_)
        throw std::logic_error ("Initialize ROS first");

      const pinocchio::Model& model = device_->model();
      if (!model.existFrame (name)) return false;

      pinocchio::FrameIndex index = model.getFrameId(name);
      for (std::size_t i = 0; i < frames_.size(); ++i)
        if (frames_[i].index == index) {
          frames_[i].option = (ComputationOption)(frames_[i].option | option);
          frames_[i].initPublishers (topicPrefix_, name, *handle_);
          return true;
        }

      frames_.push_back(FrameData(index, option));
      frames_.back().initPublishers (topicPrefix_, name, *handle_);
      return true;
    }

    void Discretization::resetTopics ()
    {
      frames_.clear();
      coms_.clear();
    }

    void Discretization::setJointNames (const std::vector<std::string>& names)
    {
      qView_ = Eigen::RowBlockIndices();
      vView_ = Eigen::RowBlockIndices();
      for (std::size_t i = 0; i < names.size(); ++i) {
        const std::string& name = names[i];
        core::JointPtr_t joint = device_->getJointByName(name);
        qView_.addRow(joint->rankInConfiguration(), joint->configSize());
        vView_.addRow(joint->rankInVelocity     (), joint->numberDof ());
      }
      qView_.updateRows<true,true,true>();
      vView_.updateRows<true,true,true>();
    }

    bool Discretization::initializeRosNode (const std::string& name, bool anonymous)
    {
      if (!ros::isInitialized()) {
        // Call ros init
        int option = ros::init_options::NoSigintHandler | (anonymous ? ros::init_options::AnonymousName : 0);
        int argc = 0;
        ros::init (argc, NULL, name, option);
      }
      bool ret = false;
      if (!handle_) {
        handle_ = new ros::NodeHandle();
        ret = true;
      }
      pubQ = handle_->advertise <dynamic_graph_bridge_msgs::Vector> (
          topicPrefix_ + "position",
          queue_size, false);
      pubV = handle_->advertise <dynamic_graph_bridge_msgs::Vector> (
          topicPrefix_ + "velocity",
          queue_size, false);
      return ret;
    }

    void Discretization::shutdownRos ()
    {
      if (!handle_) return;
      boost::mutex::scoped_lock lock(mutex_);
      resetTopics();
      pubQ.shutdown();
      pubV.shutdown();
      if (handle_) delete handle_;
      handle_ = NULL;
    }
} // namespace agimusHpp
