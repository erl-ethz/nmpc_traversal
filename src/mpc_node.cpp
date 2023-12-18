#include "mpc_controller.h"
#include "mpc_params.h"
#include "ros/ros.h"
#include "ros/console.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "mpc");
  
  // mpc_impedance::Debug<float> a;

  mpc_impedance::MPCController<float> mpc;
  ros::spin();
  return 0;
}