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

#ifndef HPP_AGIMUS_DISCRETIZATION_HH
#define HPP_AGIMUS_DISCRETIZATION_HH

#include <hpp/util/pointer.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/constraints/matrix-view.hh>
#include <hpp/core/path.hh>

#include <boost/thread/mutex.hpp>
#include <ros/node_handle.h>
#include <ros/init.h>
#include <ros/publisher.h>

namespace hpp {
  namespace agimus {
    typedef pinocchio::value_type value_type;
    typedef pinocchio::Configuration_t Configuration_t;
    typedef pinocchio::vector_t vector_t;
    typedef pinocchio::DevicePtr_t DevicePtr_t;
    typedef pinocchio::CenterOfMassComputationPtr_t CenterOfMassComputationPtr_t;
    typedef core::PathPtr_t PathPtr_t;

    HPP_PREDEF_CLASS(Discretization);
    typedef shared_ptr<Discretization> DiscretizationPtr_t;

    class Discretization
    {
      public:
        enum ComputationOption
        {   Position              = 1
          , Derivative            = 2
          , PositionAndDerivative = Position | Derivative
        };

        static DiscretizationPtr_t create (const DevicePtr_t device)
        {
          DiscretizationPtr_t ptr (new Discretization(device));
          ptr->init(ptr);
          return ptr;
        }

        /// \param time
        void compute (value_type time);

        inline bool addCenterOfMass (const std::string& name,
            const CenterOfMassComputationPtr_t& c, int option)
        {
          return addCenterOfMass (name, c, (ComputationOption) option);
        }

        bool addCenterOfMass (const std::string& name,
            const CenterOfMassComputationPtr_t& c, ComputationOption option);

        inline bool addOperationalFrame (const std::string& name, int option)
        {
          return addOperationalFrame (name, (ComputationOption) option);
        }

        bool addOperationalFrame (const std::string& name, ComputationOption option);

        void resetTopics ();

        /// \throw std::runtime_error if a joint is not found in the model.
        void setJointNames (const std::vector<std::string>& names);

        void path (const PathPtr_t& path)
        {
          path_ = path;
        }

        void topicPrefix (const std::string& tp)
        {
          topicPrefix_ = tp;
        }

        bool initializeRosNode (const std::string& name, bool anonymous);

        void shutdownRos ();

        ~Discretization();

      private:
        Discretization (const DevicePtr_t device)
          : device_ (device)
          , handle_ (NULL)
          , topicPrefix_ ("/hpp/target/")
        {}

        void init (const DiscretizationWkPtr_t)
        {}

        PathPtr_t path_;
        DevicePtr_t device_;
        ros::NodeHandle* handle_;
        boost::mutex mutex_;

        Configuration_t q_;
        vector_t v_;

        std::string topicPrefix_;

        Eigen::RowBlockIndices qView_, vView_;

        ros::Publisher pubQ, pubV;
        struct COM {
          CenterOfMassComputationPtr_t com;
          ComputationOption option;
          ros::Publisher pubQ, pubV;
          COM (CenterOfMassComputationPtr_t _com, ComputationOption _option)
            : com(_com), option(_option), pubQ(), pubV() {}
          void compute(::hpp::pinocchio::DeviceData& d);
          void initPublishers (const std::string& prefix, const std::string& name, ros::NodeHandle& nh);
        };
        std::vector<COM> coms_;
        struct FrameData {
          pinocchio::FrameIndex index;
          ComputationOption option;
          ros::Publisher pubQ, pubV;
          FrameData (pinocchio::FrameIndex _index, ComputationOption _option)
            : index(_index), option(_option), pubQ(), pubV() {}
          void initPublishers (const std::string& prefix, const std::string& name, ros::NodeHandle& nh);
        };
        std::vector<FrameData> frames_;
    };
  } // namespace agimus
} // namespace hpp

#endif // HPP_AGIMUS_DISCRETIZATION_HH
