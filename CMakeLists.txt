cmake_minimum_required(VERSION 3.0.2)
project(mpc)


find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)


# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")


cs_add_library(mpc_solver
  external/qpoases/SRC/Bounds.cpp
  external/qpoases/SRC/Constraints.cpp
  external/qpoases/SRC/CyclingManager.cpp
  external/qpoases/SRC/Indexlist.cpp
  external/qpoases/SRC/MessageHandling.cpp
  external/qpoases/SRC/QProblem.cpp
  external/qpoases/SRC/QProblemB.cpp
  external/qpoases/SRC/SubjectTo.cpp
  external/qpoases/SRC/Utils.cpp
  external/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/mpc_impedance_based_codegen/acado_qpoases_interface.cpp
  model/mpc_impedance_based_codegen/acado_integrator.c
  model/mpc_impedance_based_codegen/acado_solver.c
  model/mpc_impedance_based_codegen/acado_auxiliary_functions.c)

target_include_directories(mpc_solver PUBLIC
  model/mpc_impedance_based_codegen/
  external/qpoases
  external/qpoases/INCLUDE
  external/qpoases/SRC)

cs_add_library(mpc_wrapper
  src/mpc_wrapper.cpp)

target_link_libraries(mpc_wrapper
  mpc_solver)

cs_add_library(mpc_controller
  src/mpc_controller.cpp)

target_link_libraries(mpc_controller
  mpc_wrapper)

# make an executable
cs_install()
cs_export()

# cs_add_executable(mpc_controller_node
#   src/mpc_node.cpp)
cs_add_executable(mpc_controller_node
  src/autopilot_mpc_instance.cpp)

target_link_libraries(mpc_controller_node
  mpc_controller
  mpc_wrapper
  mpc_solver)

cs_install()
cs_export()