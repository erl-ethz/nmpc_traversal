Nonlinear Model Predictive Control strategy for traversal (aerial physical interaction) with compliant obstacles.

System Requirements:

Ubuntu 18.04 (or 20.04) with ROS Melodic (or Noetic). On our drone the code runs on the Khadas Vim3 Pro.


Descripion and Usage:

The controller node needs to be included in a control loop. It has to subscribe to a state estimator (drone's full state can be obtained with a motion capture system or, like in our case, from a tracking camera Intel Realsense T261 https://www.intelrealsense.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf?_ga=2.85385625.1408955752.1709226166-1763584868.1709226166) and to a force sensor (in our case Medusa F/T sensor from Bota System AG https://www.botasys.com/force-torque-sensors/medusa). We suggest to filter the force sensor readings to have smoother measurements. 

The controller node sends thrust and attitude commands to the low-level controller (Flight Controller running Betaflight). The MPC is adapted from https://github.com/uzh-rpg/rpg_mpc, which is based on ACADO and qpoases. Parameters of the optimization-based controller can be changed, and are loaded when launched. The whole flight control architecture, as well as the HW part, is built upon https://github.com/uzh-rpg/rpg_quadrotor_control, where the controller is used in an autopilot fashion. This repo must be installed and run on your drone. Our NMPC can then be cloned and added to your workspace. A very detailed explanation of both HW and SW requirements is already provided there.
The NMPC controller node further needs to receive a reference trajectory command to start, which is done with a rosservice call on the service 'follow_trajectory', where the speed has to be specified. Example:

'rosservice call /khadas_drone/follow_trajectory "reference_velocity:
  x: -0.15
  y: 0.0
  z: 0.0" '

