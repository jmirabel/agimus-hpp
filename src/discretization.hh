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

#ifndef HPP_CORE_DISCRETIZATION_HH
#define HPP_CORE_DISCRETIZATION_HH

#include <hpp/util/pointer.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/constraints/matrix-view.hh>
#include <hpp/core/path.hh>

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

        inline bool addCenterOfMass (const CenterOfMassComputationPtr_t& c, int option)
        {
          return addCenterOfMass (c, (ComputationOption) option);
        }

        bool addCenterOfMass (const CenterOfMassComputationPtr_t& c, ComputationOption option);

        inline bool addOperationalFrame (const std::string& name, int option)
        {
          return addOperationalFrame (name, (ComputationOption) option);
        }

        bool addOperationalFrame (const std::string& name, ComputationOption option);

        /// \throw std::runtime_error if a joint is not found in the model.
        void setJointNames (const std::vector<std::string>& names);

        void path (const PathPtr_t& path)
        {
          path_ = path;
        }

      private:
        Discretization (const DevicePtr_t device)
          : device_ (device)
        {}

        void init (const DiscretizationWkPtr_t)
        {}

        PathPtr_t path_;
        DevicePtr_t device_;

        Configuration_t q_;
        vector_t v_;

        Eigen::RowBlockIndices qView_, vView_;

        struct COM {
          CenterOfMassComputationPtr_t com;
          ComputationOption option;
          COM (CenterOfMassComputationPtr_t _com, ComputationOption _option)
            : com(_com), option(_option) {}
          void compute(::hpp::pinocchio::DeviceData& d);
        };
        std::vector<COM> coms_;
        struct FrameData {
          pinocchio::FrameIndex index;
          ComputationOption option;
          FrameData (pinocchio::FrameIndex _index, ComputationOption _option)
            : index(_index), option(_option) {}
        };
        std::vector<FrameData> frames_;
    };
  } // namespace agimus
} // namespace hpp

#endif // HPP_CORE_DISCRETIZATION_HH
