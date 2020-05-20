// Copyright (C) 2019 CNRS-LAAS
// Author: Joseph Mirabel
//
// This file is part of the agimus-hpp.
//
// agimus-hpp is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// agimus-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with agimus-hpp.  If not, see
// <http://www.gnu.org/licenses/>.

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
    typedef boost::shared_ptr<Discretization> DiscretizationPtr_t;

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
