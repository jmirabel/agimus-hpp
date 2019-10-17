#!/usr/bin/env python
#
# Copyright (c) 2019 CNRS
# Author: Joseph Mirabel
#
# This file is part of agimus-hpp.
# agimus-hpp is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# agimus-hpp is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# agimus-hpp.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.client import Client as _Parent
from hpp_idl.hpp.agimus_idl import Server

class Client (_Parent):
  """
  Connect and create clients for agimus-hpp.
  """

  defaultClients = {
          'Server'  : Server,
          }
  def __init__(self, url = None, context = "corbaserver"):
    """
    Initialize CORBA and create default clients.
    :param url: URL in the IOR, corbaloc, corbalocs, and corbanames formats.
                For a remote corba server, use
                url = "corbaloc:iiop:<host>:<port>/NameService"
    """
    self._initOrb (url)
    self._makeClients ("agimus", self.defaultClients, context)
