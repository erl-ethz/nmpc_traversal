#pragma once

#include <ros/ros.h>
#include "quadrotor_common/parameter_helper.h"

namespace mpc_impedance
{


template <typename T>
class MPCParams {
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MPCParams() :
      changed_(false),
      print_info_(false),
      max_phi_theta_(15.0),
      max_thrust_(1.4),
      max_omega_xy_(1.0),
      max_omega_yaw_(0.5),
      gravity_(9.81),
      Q_(Eigen::Matrix<T, NRefSize, NRefSize>::Identity()),
      QN_(Eigen::Matrix<T, NEndRefSize, NEndRefSize>::Identity())
    {
    }

    ~MPCParams()
    {
    }

    bool loadParameters(const ros::NodeHandle& pnh)
    {
      const std::string path_rel_to_node = "";
      #define GET_PARAM(name) \
      if (!quadrotor_common::getParam(path_rel_to_node+#name, name, pnh)) \
        return false

      #define GET_PARAM_(name) \
      if (!quadrotor_common::getParam(path_rel_to_node+#name, name ## _, pnh)) \
        return false

      // Read state costs.
      T Q_pos_xz, Q_velocity_xz, Q_velocity_y,
        Q_yaw, Q_angular, Q_torque, Q_thrust,
        Q_dynamics_dismatch,
        QN_pos_xz, QN_vel_z;

      GET_PARAM(Q_pos_xz);
      GET_PARAM(Q_velocity_xz);
      GET_PARAM(Q_velocity_y);
      GET_PARAM(Q_yaw);
      GET_PARAM(Q_angular);
      GET_PARAM(Q_torque);
      GET_PARAM(Q_thrust);
      GET_PARAM(Q_dynamics_dismatch);
      GET_PARAM(QN_pos_xz);
      GET_PARAM(QN_vel_z);

      // Check whether all state costs are positive.
      if(Q_pos_xz                         <= 0.0 ||
        Q_velocity_xz                     <= 0.0 ||
        Q_velocity_y                      <= 0.0 ||
        Q_torque                          <= 0.0 ||
        Q_thrust                          <= 0.0 ||
        Q_dynamics_dismatch               <= 0.0 ||
        QN_pos_xz                         <= 0.0 ||
        QN_vel_z                          <= 0.0)
      {
        ROS_ERROR("MPC: State cost Q has negative entries!");
        return false;
      }

      // Set state and input cost matrices.
      Q_ = (Eigen::Matrix<T, NRefSize, 1>() <<
        Q_pos_xz, Q_pos_xz, 
        Q_velocity_y, Q_velocity_xz, Q_velocity_xz,
        Q_yaw, Q_angular, Q_angular, Q_angular,
        Q_torque, Q_torque, Q_torque, Q_thrust,
        Q_dynamics_dismatch, Q_dynamics_dismatch, Q_dynamics_dismatch
        ).finished().asDiagonal();
      QN_ = (Eigen::Matrix<T, NEndRefSize, 1>() <<
        QN_pos_xz, QN_pos_xz, QN_vel_z).finished().asDiagonal();
      
      // Read input limits.
      GET_PARAM_(max_phi_theta);
      GET_PARAM_(min_thrust);
      GET_PARAM_(max_thrust);
      GET_PARAM_(max_omega_yaw);
      GET_PARAM_(max_omega_xy);

      // Choose what step of MPC-horizon to take as output
      GET_PARAM_(index_step);

      // Check whether all input limits are positive.
      if(max_phi_theta_   <= 0.0 ||
         max_thrust_      <= 0.0 ||
         min_thrust_      <= 0.0 ||
         max_omega_yaw_   <= 0.0 ||
         max_omega_xy_    <= 0.0
         )
      {
        ROS_ERROR("MPC: All limits must be positive non-zero values!");
        return false;
      }

      max_thrust_ *= gravity_;    // transform g-values into m/s^2

      quadrotor_common::getParam(path_rel_to_node+"print_info", print_info_, false, pnh);
      if(print_info_) ROS_INFO("MPC: Informative printing enabled.");

      changed_ = true;


      #undef GET_PARAM
      #undef GET_PARAM_

      return true;
    }

  bool changed_;
  int index_step_, index_step_interaction_;

  bool print_info_;

  T max_phi_theta_;
  T min_thrust_, max_thrust_;
  T max_omega_yaw_;
  T max_omega_xy_;
  T gravity_;

  Eigen::Matrix<T, NRefSize, NRefSize> Q_;
  Eigen::Matrix<T, NEndRefSize, NEndRefSize> QN_;
};

} //namespace mpc_impedance