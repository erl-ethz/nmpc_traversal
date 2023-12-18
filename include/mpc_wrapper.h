#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include "quadrotor_common/parameter_helper.h"

namespace mpc_impedance {

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

static constexpr int NSamples = ACADO_N;      // number of samples
static constexpr int NStateSize = ACADO_NX;   // number of states
static constexpr int NRefSize = ACADO_NY;     // number of reference states
static constexpr int NEndRefSize = ACADO_NYN; // number of end reference states
static constexpr int NInputSize = ACADO_NU;   // number of inputs
static constexpr int NOdSize = ACADO_NOD;     // number of online data
static constexpr int NboundAffSize = 3;       // number of Affine constraints

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

enum STATE {
  idPosX = 0,
  idPosY = 1,
  idPosZ = 2,
  idVX = 3,
  idVY = 4,
  idVZ = 5,
  idOriW = 6,
  idOriX = 7,
  idOriY = 8,
  idOriZ = 9,
  idWX = 10,
  idWY = 11,
  idWZ= 12
};

enum INPUT {
  idThrust = 0,
  idTauX = 1,
  idTauY = 2,
  idTauZ = 3
};

enum REFERENCE {
  idXRef = 0,
  idYRef = 1,
  idVXRef = 2,
  idVYRef = 3,
  idVZRef = 4,
  idOmegaX = 5,
  idOmegaY = 6,
  idOmegaZ = 7,
  idThrustRef = 8,
  idDynErrX = 9,
  idDynErrY = 10,
  idDynErrZ = 11,
};

enum END_REFERENCE {
  idxEndRef = 0,
  idyEndRef = 1,
  idVzEndRef = 2,
};


template<typename T>
class MPCWrapper
{
  public:
    
    MPCWrapper();
    MPCWrapper(
      const Eigen::Ref<const Eigen::Matrix<T, NRefSize, NRefSize>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, NEndRefSize, NEndRefSize>> QN);

    // TO DO: FILL WITH INPUTS
    void setCosts(
      const Eigen::Ref<const Eigen::Matrix<T, NRefSize, NRefSize>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, NEndRefSize, NEndRefSize>> QN);
    bool setLimits(
      T max_thrust,
      T max_phi_theta,
      T max_omega_xy,
      T max_omega_yaw);    // TO DO
    bool setReferences(
      const Eigen::Matrix<T, NRefSize, NSamples> reference_states,
      const Eigen::Matrix<T, NEndRefSize, 1> end_reference_state);
    bool solveFromScratch(
      Eigen::Ref<Eigen::Matrix<T, NStateSize, 1>> state);
    bool update(
      const Eigen::Ref<const Eigen::Matrix<T, NStateSize,1>> state,
      bool do_preparation=true);
    bool prepare();
    bool setOnlineData(
      const Eigen::Matrix<T, NOdSize, NSamples+1> online_data);

    void getState(
      const int node_index,
      Eigen::Ref<Eigen::Matrix<T, NStateSize, 1>> return_state);
    void getStates(Eigen::Ref<Eigen::Matrix<T, NStateSize, NSamples+1>> return_states);
    void getInput(
      const int node_index,
      Eigen::Ref<Eigen::Matrix<T, NInputSize,1>> return_input);
    void getInputs(Eigen::Ref<Eigen::Matrix<T, NInputSize, NSamples>> return_inputs);


    // Info&Log variables
    T getTimestep() { return dt_; };
    float kkt_value_;
    float obj_value_;
    float execution_time_;
    float prep_time_;
    float status_;

  private:
    
    T D_x_, D_y_, D_z_;
    T gravity_;

    Eigen::Map<Eigen::Matrix<float, NRefSize, NSamples, Eigen::ColMajor>> 
      acado_reference_states_{acadoVariables.y};

    Eigen::Map<Eigen::Matrix<float, NEndRefSize, 1, Eigen::ColMajor>> 
      acado_reference_end_states_{acadoVariables.yN};

    Eigen::Map<Eigen::Matrix<float, NStateSize, NSamples+1, Eigen::ColMajor>> 
      acado_states_{acadoVariables.x};

    Eigen::Map<Eigen::Matrix<float, NStateSize, 1, Eigen::ColMajor>> 
      acado_initial_state_{acadoVariables.x0};

    Eigen::Map<Eigen::Matrix<float, NInputSize, NSamples, Eigen::ColMajor>> 
      acado_inputs_{acadoVariables.u};

    Eigen::Map<Eigen::Matrix<float, NOdSize, NSamples+1, Eigen::ColMajor>> 
      acado_online_data_{acadoVariables.od};

    Eigen::Map<Eigen::Matrix<float, NRefSize, NRefSize*NSamples, Eigen::ColMajor>> 
      acado_W_{acadoVariables.W};

    Eigen::Map<Eigen::Matrix<float, NEndRefSize, NEndRefSize, Eigen::ColMajor>> 
      acado_WN_{acadoVariables.WN};

    Eigen::Map<Eigen::Matrix<float, NInputSize, NSamples, Eigen::ColMajor>> 
      acado_cmd_lower_bounds_{acadoVariables.lbValues};

    Eigen::Map<Eigen::Matrix<float, NInputSize, NSamples, Eigen::ColMajor>> 
      acado_cmd_upper_bounds_{acadoVariables.ubValues};

    Eigen::Map<Eigen::Matrix<float, NboundAffSize, NSamples, Eigen::ColMajor>> 
      acado_lower_bounds_{acadoVariables.lbAValues};

    Eigen::Map<Eigen::Matrix<float, NboundAffSize, NSamples, Eigen::ColMajor>> 
      acado_upper_bounds_{acadoVariables.ubAValues};

    Eigen::Matrix<T, NRefSize, 1> hover_reference_;

    // initialize weight matrix 
    Eigen::Matrix<T, NRefSize, NRefSize> W_ = Eigen::Matrix<T, NRefSize,NRefSize>::Identity();

    Eigen::Matrix<T, NEndRefSize, NEndRefSize> WN_ = 1 * Eigen::Matrix<T, NEndRefSize, NEndRefSize>::Identity();

    acado_timer t;
    bool acado_is_prepared_ = false;
    const T dt_{0.05};


    void setInitialValues();
};

template class MPCWrapper<float>;

}