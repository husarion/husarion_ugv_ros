# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

include(ExternalProject)

set(DEPENDENCIES)

list(APPEND DEPENDENCIES ep_cppuprofile)

ExternalProject_Add(
  ep_cppuprofile
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  GIT_REPOSITORY https://github.com/Orange-OpenSource/cppuprofile.git
  GIT_TAG 1.1.1
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ep_cppuprofile
  BUILD_COMMAND $(MAKE) -C <BINARY_DIR>
  INSTALL_COMMAND make install INSTALL_PREFIX=<INSTALL_DIR>
  # DCMAKE_POLICY_VERSION_MINIMUM is required because of using external
  # `cppuprofile` repo in the diagnostics pkg
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
             -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  BUILD_IN_SOURCE 1)

ExternalProject_Add(
  ep_husarion_ugv_diagnostics
  DEPENDS ${DEPENDENCIES}
  SOURCE_DIR ${PROJECT_SOURCE_DIR}
  CMAKE_ARGS -DUSE_SUPERBUILD=OFF
  INSTALL_COMMAND ""
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
