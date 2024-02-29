#include "mpc_wrapper.h"

namespace mpc_impedance {

template<typename T>
MPCWrapper<T>::MPCWrapper():
    hover_reference_(Eigen::Matrix<T, NRefSize, 1>::Zero()),
    gravity_(9.81),
    D_x_(1),
    D_y_(1),
    D_z_(2)
{
  // Reset solver memories
  memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
  memset(&acadoVariables, 0, sizeof( acadoVariables ));

  // Initialize the solver
  acado_initializeSolver();
}

// Constructor with cost matrices as arguments.
template <typename T>
MPCWrapper<T>::MPCWrapper(
  const Eigen::Ref<const Eigen::Matrix<T, NRefSize, NRefSize>> Q,
  const Eigen::Ref<const Eigen::Matrix<T, NEndRefSize, NEndRefSize>> QN)
{
  setCosts(Q, QN);
  MPCWrapper();
}

// Set cost matrix
template <typename T>
void MPCWrapper<T>::setCosts(
  const Eigen::Ref<const Eigen::Matrix<T, NRefSize, NRefSize>> Q,
  const Eigen::Ref<const Eigen::Matrix<T, NEndRefSize, NEndRefSize>> QN) 
{
  W_ = Q;
  WN_ = QN;

  acado_W_ = W_.replicate(1,NSamples).template cast<float>();
  acado_WN_ = WN_.template cast<float>();
}

// Set the input limits.
template <typename T>
bool MPCWrapper<T>::setLimits(
  T max_thrust,
  T max_phi_theta,
  T max_omega_xy,
  T max_omega_yaw)
{

  if(max_thrust <= 0.0)
  {
    ROS_ERROR("MPC: Maximal thrust is not set properly, not changed.");
    return false;
  }

  if(max_phi_theta <= 0.0)
  {
    ROS_ERROR("MPC: Maximal xy-rate is not set properly, not changed.");
    return false;
  }

  if(max_omega_xy <= 0.0)
  {
    ROS_ERROR("MPC: Maximal thrust is not set properly, not changed.");
    return false;
  }

  if(max_omega_yaw <= 0.0)
  {
    ROS_ERROR("MPC: Maximal xy-rate is not set properly, not changed.");
    return false;
  }

  T pos_z_lower = 0;
  T pos_z_upper = 1000;

  // Set Control variables input boundaries
  Eigen::Matrix<float, 1, NSamples> template_input_bound = Eigen::Matrix<float, 1, NSamples>::Ones();
  acado_cmd_upper_bounds_.block(INPUT::idThrust,0,1,NSamples) = max_thrust * template_input_bound;    // thrust

  return true;
}


template<typename T>
void MPCWrapper<T>::setInitialValues() {
 
  // Initialize states x and xN and input u
  acado_initial_state_ = Eigen::Matrix<float, NStateSize, 1>::Zero();
  acado_states_ = Eigen::Matrix<float, NStateSize, NSamples+1>::Zero();

  // Initialize the reference y and yN
  acado_reference_states_ = Eigen::Matrix<float, NRefSize, NSamples>::Zero();
  acado_reference_end_states_ = Eigen::Matrix<float, NEndRefSize, 1>::Zero();
}

template<typename T>
bool MPCWrapper<T>::setOnlineData(
  const Eigen::Matrix<T, NOdSize, NSamples+1> online_data)
{
  acado_online_data_.block(0,0,NOdSize,NSamples+1) = 
    online_data.block(0,0,NOdSize,NSamples+1).template cast<float>();
  
  return true;
}



template<typename T>
bool MPCWrapper<T>::setReferences(
  const Eigen::Matrix<T, NRefSize, NSamples> reference_states,
  const Eigen::Matrix<T, NEndRefSize, 1> end_reference_state)
{
  acado_reference_states_.block(0,0,NRefSize,NSamples) = 
    reference_states.block(0,0,NRefSize,NSamples).template cast<float>();

  acado_reference_end_states_ = end_reference_state;

  return true;
} 


template<typename T>
void MPCWrapper<T>::getState(
  const int node_index,
  Eigen::Ref<Eigen::Matrix<T, NStateSize, 1>> return_state)
{
  if (node_index < 0)
    ROS_ERROR("Passing a negative index to getState function");
  if (node_index > NStateSize)
    ROS_ERROR("Passing an index > horizon_length to the getState function");
  
  return_state = acado_states_.col(node_index).cast<T>();
}

template<typename T>
void MPCWrapper<T>::getStates(
  Eigen::Ref<Eigen::Matrix<T, NStateSize, NSamples+1>> return_states)
{
  return_states = acado_states_.cast<T>();
}


template<typename T>
void MPCWrapper<T>::getInput(
  const int node_index,
  Eigen::Ref<Eigen::Matrix<T, NInputSize, 1>> return_input)
{
  if (node_index < 0)
    ROS_ERROR("Passing a negative index to getState function");
  if (node_index > NStateSize)
    ROS_ERROR("Passing an index > horizon_length to the getState function");
  
  return_input = acado_inputs_.col(node_index).cast<T>();
}


template<typename T>
void MPCWrapper<T>::getInputs(
  Eigen::Ref<Eigen::Matrix<T, NInputSize, NSamples>> return_inputs)
{
  return_inputs = acado_inputs_.cast<T>();
}


// Reset states and inputs and calculate new solution.
template<typename T>
bool MPCWrapper<T>::solveFromScratch(
  Eigen::Ref<Eigen::Matrix<T, NStateSize, 1>> state)
{
  acado_states_ = state.replicate(1, NSamples+1).template cast<float>();
  acado_inputs_ = Eigen::Matrix<float, NInputSize, NSamples>::Zero();
  return update(state);
}


// Calculate new solution from last known solution.
template<typename T>
bool MPCWrapper<T>::update(
  const Eigen::Ref<const Eigen::Matrix<T, NStateSize,1>> state,
  bool do_preparation)
{
  if(!acado_is_prepared_)
  {
    ROS_WARN("MPC: Solver was triggered without preparation, abort!");
    return false;
  }

  acado_initial_state_ = state.template cast<float>();

  acado_is_prepared_ = false;
  // Perform feedback step and reset preparation check.
  acado_tic(&t);
  status_ = acado_feedbackStep();
  execution_time_ = acado_toc(&t);
  obj_value_ = acado_getObjective();
  kkt_value_ = acado_getKKT();

  // Prepare the solver if wanted
  if(do_preparation)
  {
    acado_preparationStep();
    acado_is_prepared_ = true;
  }

  return true;
}



// Prepare the solver.
// Must be triggered between iterations if not done in the update function.
template <typename T>
bool MPCWrapper<T>::prepare()
{
  acado_tic(&t);
  acado_preparationStep();
  prep_time_ = acado_toc(&t);
  acado_is_prepared_ = true;

  return true;
}


}