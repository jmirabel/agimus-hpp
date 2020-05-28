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

#ifndef HPP_AGIMUS_SERVER_HH
# define HPP_AGIMUS_SERVER_HH

# include <stdexcept>

# include <hpp/corba/template/server.hh>

# include <hpp/corbaserver/server-plugin.hh>

# include <agimus/hpp/config.hh>

# include <hpp/agimus_idl/server-idl.hh>
# include <hpp/agimus_idl/discretization-idl.hh>
# include "discretization.hh"

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

          agimus_idl::Discretization_ptr getDiscretization ();

        private:
          ServerPlugin* server_;
          DiscretizationPtr_t discretization_;
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

      ::CORBA::Object_ptr servant (const std::string& name) const;

    private:
      corba::Server <impl::Server>* serverImpl_;
    }; // class ServerPlugin
  } // namespace agimus
} // namespace hpp

#endif // HPP_AGIMUS_SERVER_HH
