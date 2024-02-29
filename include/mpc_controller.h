#pragma once

#include "mpc_wrapper.h"
#include "mpc_params.h"
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/trajectory.h>
#include <autopilot/autopilot_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <quadrotor_common/trajectory_point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <mpc/MpcLog.h>


namespace mpc_impedance {


  template<typename T>
  class MPCController {
    public:
      MPCController() : MPCController(ros::NodeHandle()) {}
      MPCController(const ros::NodeHandle& nh);
      // quadrotor_common::ControlCommand off();
      quadrotor_common::ControlCommand run(
        const quadrotor_common::QuadStateEstimate& state_estimate,
        const quadrotor_common::Trajectory& reference_trajectory,
        const MPCParams<T>& params);
    private:
      // Functions
      void updateControlCmd(const MPCParams<T>& params);
      void setNewParams(const MPCParams<T>& params);
      void updateReferencePoint(
        const quadrotor_common::TrajectoryPoint& reference_point, 
        const MPCParams<T>& params);
      void defineReference();
      void ExecuteMPC();
      void GetSolution();
      void WrenchExternalCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
      void defineOnlineData();
      void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void MPCLogCallback(const ros::TimerEvent&);
      void autopilotStateCallback(const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg);
      void CheckForInteraction(const MPCParams<T>& params);
      
      // Handles
      ros::NodeHandle nh_;

      // Subscribers and Publishers
      ros::Subscriber wrench_external_sub_;
      ros::Subscriber odometry_sub_;
      ros::Subscriber autopilot_state_sub_;
      ros::Publisher arm_pub_;
      ros::Publisher interaction_pub_;
      ros::Publisher mpc_log_pub_;

      // MPC
      MPCWrapper<T> mpc_wrapper_;

      // MPC parameters
      bool solve_from_scratch_;

      // Variables
      Eigen::Matrix<T, NStateSize, 1> estimate_state_;
      Eigen::Matrix<T, NRefSize, NSamples> reference_states_;
      Eigen::Matrix<T, NEndRefSize, 1> end_reference_state_;
      Eigen::Matrix<T, NInputSize, NSamples+1> reference_inputs_;
      Eigen::Matrix<T, NStateSize, NSamples+1> predicted_states_;
      Eigen::Matrix<T, NInputSize, 1> forward_input_;
      Eigen::Matrix<T, NInputSize, NSamples> predicted_inputs_;
      std_msgs::Bool interaction_msg_;
      mpc::MpcLog mpc_log_msg_;

      Eigen::Vector3d current_position_, current_velocity_, current_euler_;
      Eigen::Quaterniond current_attitude_;
      bool update_sensors_values_;
      bool interaction_;
      double current_yaw_;

      // Node parameters
      static constexpr double kExecLoopRate_ = 50.0;
      bool autopilot_was_in_reference_control_mode_ = false;
      bool first_time_after_init_ = true;
      T y_ref_, z_ref_, yaw_ref_, vx_ref_, vy_ref_, vz_ref_, omega_z_ref_;
      int step_;
      Eigen::Matrix<T, NOdSize, NSamples+1> online_data_;
      ros::Timer timer_mpc_log_;
      ros::Time call_time_;

      // Wrench variables
      T offset_cage_sensor_ = 0;    // For real-test
      T measured_force_x_;
      T measured_force_y_;
      T measured_force_z_;
      T measured_torque_x_;
      T measured_torque_y_;
      T measured_torque_z_;

      // Control Command
      quadrotor_common::ControlCommand control_command_;

  };
  
  
} // namespace mpc_impedance
