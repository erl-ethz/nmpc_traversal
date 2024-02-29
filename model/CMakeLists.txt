#     rpg_quadrotor_mpc
#     A model predictive control implementation for quadrotors.
#     Copyright (C) 2017-2018 Philipp Foehn, 
#     Robotics and Perception Group, University of Zurich
#  
#     Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
#     https://github.com/uzh-rpg/rpg_quadrotor_control
# 
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <http://www.gnu.org/licenses/>.


# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( mpc_impedance )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )
find_package(nlohmann_json 3.2.0 REQUIRED)

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )


#
# Build an executable
#
ADD_EXECUTABLE( mpc_impedance_codegen mpc_impedance_codegen.cpp )
TARGET_LINK_LIBRARIES( mpc_impedance_codegen 
  ${ACADO_SHARED_LIBRARIES} 
  nlohmann_json::nlohmann_json)
SET_TARGET_PROPERTIES( mpc_impedance_codegen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )