// Code structure:
//   main
//     manages time
//     container for all vehicle positions at current time
//   class for each type of controller
//   class for each type of estimator
//   class for all sensors
//     each expects environment and true vehicle state [pos;vel;acc;att;ang_vel]
//   class for each type of vehicle
//     true state [pos;vel;acc;att;ang_vel]
//     contains dynamics
//     inherits controller
//     inherits estimator
//     inherits sensors
//     logs all self data
//   class for environment
//     contains landmarks
//     contains wind
//     logs all self data
#include "common_cpp/common.h"

int main()
{
  // Load simulation parameters
  std::string param_file = "../params/params.yaml";
  double tf, dt;
  common::get_yaml_node("tf", param_file, tf);
  common::get_yaml_node("dt", param_file, dt);
}   
