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

#ifndef HPP_AGIMUS_SERVER_HH
# define HPP_AGIMUS_SERVER_HH

# include <stdexcept>

# include <hpp/corba/template/server.hh>

# include <hpp/corbaserver/server-plugin.hh>

# include <agimus/hpp/config.hh>

# include <hpp/agimus_idl/server-idl.hh>
# include <hpp/agimus_idl/discretization-idl.hh>

namespace hpp {
  namespace agimus {
    class ServerPlugin;

    namespace impl {
      class Server : public virtual POA_hpp::agimus_idl::Server
      {
        public:
          Server () : server_(NULL) {}

          void setServer (ServerPlugin* server)
          {
            server_ = server;
          }

          agimus_idl::Discretization_ptr getDiscretization () throw (Error);

        private:
          ServerPlugin* server_;
      };
    }

    class AGIMUS_HPP_DLLAPI ServerPlugin : public corbaServer::ServerPlugin
    {
    public:
      ServerPlugin (corbaServer::Server* parent);

      ~ServerPlugin ();

      /// Start corba server
      void startCorbaServer(const std::string& contextId,
			    const std::string& contextKind);

      std::string name () const;

    private:
      corba::Server <impl::Server>* serverImpl_;
    }; // class ServerPlugin
  } // namespace agimus
} // namespace hpp

#endif // HPP_AGIMUS_SERVER_HH
