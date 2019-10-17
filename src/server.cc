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

#include "server.hh"

#include <hpp/util/exception.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/servant-base.hh>

#include "hpp/agimus_idl/discretization.hh"

#include "discretization.hh"

namespace hpp {
  namespace agimus {
    namespace impl {
      agimus_idl::Discretization_ptr Server::getDiscretization () throw (Error)
      {
        DiscretizationPtr_t d =
          Discretization::create (server_->problemSolver()->robot());

        return corbaServer::makeServant<agimus_idl::Discretization_ptr>
          (server_->parent(), new corbaServer::agimus_impl::Discretization (server_->parent(), d));
      }

    } // namespace impl

    ServerPlugin::ServerPlugin (corbaServer::Server* server)
      : corbaServer::ServerPlugin (server),
      serverImpl_ (NULL)
    {}

    ServerPlugin::~ServerPlugin ()
    {
      if (serverImpl_  ) delete serverImpl_;
    }

    std::string ServerPlugin::name () const
    {
      return "agimus-hpp";
    }

    /// Start corba server
    void ServerPlugin::startCorbaServer(const std::string& contextId,
				  const std::string& contextKind)
    {
      bool mThd = parent()->multiThread();
      serverImpl_   = new corba::Server <impl::Server>   (0, NULL, mThd, "child");

      serverImpl_->implementation ().setServer (this);

      if (serverImpl_->startCorbaServer(contextId, contextKind,
				       "agimus", "server") != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception,
			     "Failed to start agimus corba server.");
      }
    }
  } // namespace agimus
} // namespace hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::agimus::ServerPlugin)
