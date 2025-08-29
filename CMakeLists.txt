# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

# Make CMAKE_C_VISIBILITY_PRESET work properly.
set(CMAKE_POLICY_DEFAULT_CMP0063 NEW)
# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
# Avoid BUILD_SHARED_LIBS getting overridden by an option() in ccd.
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking
set(MSVC_INCREMENTAL_DEFAULT ON)

project(
  mujoco
  VERSION 3.3.4
  DESCRIPTION "MuJoCo Physics Simulator"
  HOMEPAGE_URL "https://mujoco.org"
)

enable_language(C)
enable_language(CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

option(MUJOCO_BUILD_EXAMPLES "Build samples for MuJoCo" OFF)
option(MUJOCO_BUILD_SIMULATE "Build simulate library for MuJoCo" OFF)


include(MujocoOptions)
include(MujocoMacOS)
# include(MujocoDependencies)

# set(MUJOCO_HEADERS
#     include/mujoco/mjdata.h
#     include/mujoco/mjexport.h
#     include/mujoco/mjmacro.h
#     include/mujoco/mjmodel.h
#     include/mujoco/mjplugin.h
#     include/mujoco/mjrender.h
#     include/mujoco/mjsan.h
#     include/mujoco/mjspec.h
#     include/mujoco/mjthread.h
#     include/mujoco/mjtnum.h
#     include/mujoco/mjui.h
#     include/mujoco/mjvisualize.h
#     include/mujoco/mjxmacro.h
#     include/mujoco/mujoco.h
# )

# Add metadata to mujoco.dll when building on Windows.
if(WIN32)
  set(MUJOCO_RESOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/dist/mujoco.rc)
else()
  set(MUJOCO_RESOURCE_FILES "")
endif()

# target_include_directories(
#   mujoco
#   PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#          $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/>
#   PRIVATE src
# )

add_subdirectory(plugin/hrpc)
#add_subdirectory(plugin/elasticity)
#add_subdirectory(plugin/actuator)
#add_subdirectory(plugin/sensor)
#add_subdirectory(plugin/sdf)



add_subdirectory(model)

add_subdirectory(controller)


# `simulate` defines a macro, embed_in_bundle, that's used by `sample`, so that
# subdirectory needs to be added first.
# TODO: Remove this order dependency.
if(MUJOCO_BUILD_SIMULATE)
 # add_subdirectory(simulate)
endif()

if(MUJOCO_BUILD_EXAMPLES)
 # add_subdirectory(sample)
endif()



