#include "mpc_controller.h"

namespace mpc_impedance {  


  template<typename T>
  MPCController<T>::MPCController(const ros::NodeHandle& nh) :
      nh_(nh),
      mpc_wrapper_(),
      y_ref_(1.0),    // x reference
      z_ref_(1.0),    // y reference
      yaw_ref_(0.0),  // yaw reference
      vx_ref_(0.0),
      vy_ref_(0.0),
      vz_ref_(0.5),   // vz reference
      omega_z_ref_(0.0),
      estimate_state_(Eigen::Matrix<T, NStateSize, 1>::Zero()),
      reference_states_(Eigen::Matrix<T, NRefSize, NSamples>::Zero()),
      reference_inputs_(Eigen::Matrix<T, NInputSize, NSamples+1>::Zero()),
      predicted_states_(Eigen::Matrix<T, NStateSize, NSamples+1>::Zero()),
      predicted_inputs_(Eigen::Matrix<T, NInputSize, NSamples>::Zero()),
      update_sensors_values_(false),
      interaction_(false)
  {

    MPCParams<T> default_params;

    // Setting MPC Configuration
    setNewParams(default_params);
    Eigen::Matrix<T, NOdSize, NSamples+1> onlineData_start = Eigen::Matrix<T, NOdSize, NSamples+1>::Zero();
    mpc_wrapper_.setOnlineData(onlineData_start);
    mpc_wrapper_.prepare();

    mpc_log_pub_ = nh_.advertise<mpc::MpcLog>("mpc_log", 1);
    interaction_pub_ = nh_.advertise<std_msgs::Bool>("interaction",1);
    wrench_external_sub_ = nh_.subscribe("/ft_sensor_topic_filtered", 1,
      &MPCController<T>::WrenchExternalCallback, this);
    odometry_sub_ = nh_.subscribe("state_estimate", 1,
      &MPCController<T>::OdometryCallback, this);
    autopilot_state_sub_ =
      nh_.subscribe("autopilot/feedback", 10,
                    &MPCController<T>::autopilotStateCallback, this);

    // Set-up for the prediction horizon publisher
    mpc_log_msg_.horizon_trajectory.points = std::vector<quadrotor_msgs::TrajectoryPoint>(NSamples);
    timer_mpc_log_ = nh_.createTimer(
      ros::Duration(mpc_wrapper_.getTimestep()),
      &MPCController<T>::MPCLogCallback, this, false, true);
  }

  template<typename T>
  quadrotor_common::ControlCommand MPCController<T>::run(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const quadrotor_common::Trajectory& reference_trajectory,
      const MPCParams<T>& params) { 

    call_time_ = ros::Time::now();

    double t_start_mpc = ros::Time::now().toSec();

    if(first_time_after_init_) {
      setNewParams(params);
      first_time_after_init_ = false;
    }

    CheckForInteraction(params);

    updateReferencePoint(reference_trajectory.points.front(), params);

    defineOnlineData();

    ExecuteMPC();     // Execute the solver;

    GetSolution();    // Transfer to MPC class the acado_solution

    mpc_wrapper_.prepare();

    double t_end_mpc = ros::Time::now().toSec();
    if (params.print_info_)
    {
      ROS_INFO("Time for execution: %f s", (t_end_mpc-t_start_mpc));
      std::cout << "max_thrust: " << params.max_thrust_ << "\n";
    }
    
    // Return controlCmd
    updateControlCmd(params);
    return control_command_;
  }

  template<typename T>
  void MPCController<T>::ExecuteMPC() {

    estimate_state_[idPosX] =  (T)current_position_.x();
    estimate_state_[idPosY] =  (T)current_position_.y();
    estimate_state_[idPosZ] =  (T)current_position_.z();
    estimate_state_[idVX]   =  (T)current_velocity_.x();
    estimate_state_[idVY]   =  (T)current_velocity_.y();
    estimate_state_[idVZ]   =  (T)current_velocity_.z();
    estimate_state_[idOriW] =  (T)current_attitude_.w();
    estimate_state_[idOriX] =  (T)current_attitude_.x();
    estimate_state_[idOriY] =  (T)current_attitude_.y();
    estimate_state_[idOriZ] =  (T)current_attitude_.z();

    // TO DO: See the Rpg_mpc that instantiate a thread for the preparation step
    static const bool do_preparation_step(false);

    // Establish and set the reference to the mpc
    defineReference();
    mpc_wrapper_.setReferences(reference_states_, end_reference_state_);

    // Solving the optimization problem
    if (solve_from_scratch_) {
      ROS_INFO("Solving MPC with hover as initial guess.");
      mpc_wrapper_.solveFromScratch(estimate_state_);
      solve_from_scratch_ = false;
    } 
    else {
      mpc_wrapper_.update(estimate_state_, do_preparation_step);
    }
  }
  
  template<typename T>
  void MPCController<T>::GetSolution() {

    // Getting the state and input(s)
    mpc_wrapper_.getStates(predicted_states_);
    mpc_wrapper_.getInput(0,forward_input_);
    mpc_wrapper_.getInputs(predicted_inputs_);
  }


  template<typename T>
  void MPCController<T>::defineReference()
  {
    // Dummy reference: Straight line parallel to z
    Eigen::Matrix<T, NRefSize, 1> reference_state;

    reference_state << 
      y_ref_, z_ref_,                 // x,y reference for drone state
      vx_ref_, vy_ref_, vz_ref_,      // reference velocity for drone state
      yaw_ref_,                       // reference yaw (avoid spinning) and omega_z
      0, 0, omega_z_ref_,             // reference angular velocity (x,y,z - body)
      0, 0, 0, 0,                     // reference torque and thrust
      0, 0, 0;                        // dismatch between real and impedance dynamics

    end_reference_state_ <<
      y_ref_, z_ref_,
      vz_ref_;
    reference_states_ = reference_state.replicate(1, NSamples);
  }

  template<typename T>
  void MPCController<T>::updateReferencePoint(
    const quadrotor_common::TrajectoryPoint& reference_point,
    const MPCParams<T>& params)
  {
      y_ref_ = (T)reference_point.position.y();
      z_ref_ = (T)reference_point.position.z();
      yaw_ref_ = 0;
      vx_ref_ = (T)reference_point.velocity.x();
      vy_ref_ = (T)reference_point.velocity.y();
      vz_ref_ = (T)reference_point.velocity.z();
      omega_z_ref_ = 0;
      mpc_wrapper_.setCosts(params.Q_, params.QN_);  
  }

  template<typename T>
  void MPCController<T>::setNewParams(
      const MPCParams<T>& params)
  {
    mpc_wrapper_.setCosts(params.Q_, params.QN_);
    mpc_wrapper_.setLimits(params.max_thrust_, params.max_phi_theta_, params.max_omega_xy_, params.max_omega_xy_);
  }

  template<typename T>
  void MPCController<T>::WrenchExternalCallback(
    const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    // Force external should be trasmormed in NED coordinates
    measured_force_x_ = static_cast<T>(msg->wrench.force.x);
    measured_force_y_ = static_cast<T>(msg->wrench.force.y);
    measured_force_z_ = static_cast<T>(msg->wrench.force.z) - offset_cage_sensor_;
    measured_torque_x_ = static_cast<T>(msg->wrench.torque.x);
    measured_torque_y_ = static_cast<T>(msg->wrench.torque.y);
    measured_torque_z_ = static_cast<T>(msg->wrench.torque.z);
  }

  template<typename T>
  void MPCController<T>::autopilotStateCallback(
    const quadrotor_msgs::AutopilotFeedback::ConstPtr & msg)
  {
    if (msg->autopilot_state == 8)    // REFERENCE_CONTROL = 8
      update_sensors_values_ = true;
    else
      update_sensors_values_ = false;
  }

  template<typename T>
  void MPCController<T>::defineOnlineData()
  {
    Eigen::Matrix<T, NOdSize, 1> current_data;
    if (update_sensors_values_) {
      current_data <<
        measured_force_x_, measured_force_y_, measured_force_z_,
        measured_torque_x_, measured_torque_y_, measured_torque_z_,
        vx_ref_, vy_ref_, vz_ref_;
    }
    else {
      current_data <<
        0, 0, 0,
        0, 0, 0,
        vx_ref_, vy_ref_, vz_ref_;
    }

    online_data_ = current_data.replicate(1, NSamples+1);   // Extend current data to the whole horizon

    mpc_wrapper_.setOnlineData(online_data_);

  }

  template<typename T>
  void MPCController<T>::updateControlCmd(const MPCParams<T>& params)
  {
    control_command_.control_mode = quadrotor_common::ControlMode::ATTITUDE;

    control_command_.timestamp = call_time_;
    control_command_.armed = true;
    control_command_.expected_execution_time = call_time_;
    control_command_.collective_thrust = forward_input_(INPUT::idThrust);  

    step_ = params.index_step_;
    control_command_.orientation.w() = predicted_states_.col(step_)(STATE::idOriW);
    control_command_.orientation.x() = predicted_states_.col(step_)(STATE::idOriX);
    control_command_.orientation.y() = predicted_states_.col(step_)(STATE::idOriY);
    control_command_.orientation.z() = predicted_states_.col(step_)(STATE::idOriZ);
  }

  template<typename T>
  void MPCController<T>::CheckForInteraction(const MPCParams<T>& params)
  {
    if ((std::abs(measured_force_z_)+std::abs(measured_force_y_)+std::abs(measured_force_z_) > params.threshold_force_interaction_) && update_sensors_values_)
      interaction_ = true;
    else
      interaction_ = false;
  }

  template<typename T>
  void MPCController<T>::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    current_position_.x() = msg->pose.pose.position.x;
    current_position_.y() = msg->pose.pose.position.y;
    current_position_.z() = msg->pose.pose.position.z;

    if (msg->pose.pose.orientation.w > 0) {
      current_attitude_.w() = msg->pose.pose.orientation.w;
      current_attitude_.x() = msg->pose.pose.orientation.x;
      current_attitude_.y() = msg->pose.pose.orientation.y;
      current_attitude_.z() = msg->pose.pose.orientation.z;
    }
    else {
      current_attitude_.w() = - msg->pose.pose.orientation.w;
      current_attitude_.x() = - msg->pose.pose.orientation.x;
      current_attitude_.y() = - msg->pose.pose.orientation.y;
      current_attitude_.z() = - msg->pose.pose.orientation.z;
    }

    current_velocity_.x() = msg->twist.twist.linear.x;
    current_velocity_.y() = msg->twist.twist.linear.y;
    current_velocity_.z() = msg->twist.twist.linear.z;
  }

  template<typename T>
  void MPCController<T>::MPCLogCallback(const ros::TimerEvent&)
  {
    // MPC prediction horizon
    int i = 0;
    for (auto it = mpc_log_msg_.horizon_trajectory.points.begin(); 
      it != mpc_log_msg_.horizon_trajectory.points.end(); 
      it++)
    {
      it->pose.position.x = predicted_states_.col(i)[idPosX];
      it->pose.position.y = predicted_states_.col(i)[idPosY];
      it->pose.position.z = predicted_states_.col(i)[idPosZ];
      it->velocity.linear.x = predicted_states_.col(i)[idVX];
      it->velocity.linear.y = predicted_states_.col(i)[idVY];
      it->velocity.linear.z = predicted_states_.col(i)[idVZ];
      it->pose.orientation.w = predicted_states_.col(i)[idOriW];
      it->pose.orientation.x = predicted_states_.col(i)[idOriX];
      it->pose.orientation.y = predicted_states_.col(i)[idOriY];
      it->pose.orientation.z = predicted_states_.col(i)[idOriZ];
      it->velocity.angular.x = predicted_states_.col(i)[idWX];
      it->velocity.angular.y = predicted_states_.col(i)[idWY];
      it->velocity.angular.z = predicted_states_.col(i)[idWZ];
      i++;
    }
    
    // filling the msg
    mpc_log_msg_.header.stamp = ros::Time::now();
    mpc_log_msg_.header.seq++;

    // Current state
    mpc_log_msg_.pose.position.x = estimate_state_[idPosX];
    mpc_log_msg_.pose.position.y = estimate_state_[idPosY];
    mpc_log_msg_.pose.position.z = estimate_state_[idPosZ];
    mpc_log_msg_.pose.orientation.w = estimate_state_[idOriW];
    mpc_log_msg_.pose.orientation.x = estimate_state_[idOriX];
    mpc_log_msg_.pose.orientation.y = estimate_state_[idOriY];
    mpc_log_msg_.pose.orientation.z = estimate_state_[idOriZ];
    mpc_log_msg_.velocity.linear.x = estimate_state_[idVX];
    mpc_log_msg_.velocity.linear.y = estimate_state_[idVY];
    mpc_log_msg_.velocity.linear.z = estimate_state_[idVZ];
    mpc_log_msg_.velocity.angular.x = estimate_state_[idWX];
    mpc_log_msg_.velocity.angular.y = estimate_state_[idWY];
    mpc_log_msg_.velocity.angular.z = estimate_state_[idWZ];

    // Real MPC Command (Torques&Thrust)
    mpc_log_msg_.control_command_mpc.collective_thrust = predicted_inputs_.col(0)[idThrust];
    //mpc_log_msg_.control_command_mpc.torque.x = predicted_inputs_.col(0)[idTauX];
    //mpc_log_msg_.control_command_mpc.torque.y = predicted_inputs_.col(0)[idTauY];
    //mpc_log_msg_.control_command_mpc.torque.z = predicted_inputs_.col(0)[idTauZ];

    // Interaction
    mpc_log_msg_.interaction = interaction_;
    interaction_msg_.data = interaction_;

    // MPC-algorithm Logs
    mpc_log_msg_.kkt_value = mpc_wrapper_.kkt_value_;
    mpc_log_msg_.objective_function_value = mpc_wrapper_.obj_value_;
    mpc_log_msg_.execution_time = mpc_wrapper_.execution_time_;
    mpc_log_msg_.preparation_time = mpc_wrapper_.prep_time_;
    mpc_log_msg_.solver_status = mpc_wrapper_.status_;

    // External Wrench
    mpc_log_msg_.external_wrench.force.x = measured_force_x_;
    mpc_log_msg_.external_wrench.force.y = measured_force_y_;
    mpc_log_msg_.external_wrench.force.z = measured_force_z_; 
    mpc_log_msg_.external_wrench.torque.x = measured_torque_x_;
    mpc_log_msg_.external_wrench.torque.y = measured_torque_y_;
    mpc_log_msg_.external_wrench.torque.z = measured_torque_z_;

    // Publish the msg
    mpc_log_pub_.publish(mpc_log_msg_);
    interaction_pub_.publish(interaction_msg_);
  }


template class MPCController<float>;
} // namespace mpc_impedance
