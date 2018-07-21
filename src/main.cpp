// Code structure:
//   main
//     manages time
//   class for each type of controller
//   class for each type of estimator
//   class for all sensors
//     each expects environment and true vehicle state
//     inherits environment
//   class for each type of vehicle
//     inherits vehicle base class and overrides dynamics
//     inherits controller
//     inherits estimator
//     inherits sensors
//     logs all self data
//   class for environment
//     contains landmarks
//     contains wind
//     container for all vehicle positions at current time
//     logs all self data
#include "common_cpp/common.h"
#include "quadrotor.h"

int main()
{
  // Load simulation parameters
  std::string param_file = "../params/params.yaml";
  double t(0), tf, dt;
  common::get_yaml_node("tf", param_file, tf);
  common::get_yaml_node("dt", param_file, dt);
  common::ProgressBar prog_bar;
  prog_bar.init(tf/dt,40);

  // Create environment

  // Create vehicles
  quadrotor::Quadrotor quad1(param_file);

  // Store initial vehicle states in environment

  // Main simulation loop
  while (t < tf)
  {
    // Run each vehicle
    Eigen::Vector3d vw(0,0,0);
    quad1.run(t,dt,vw);

    // Update wind and stored vehicle states in environment

    // Increment time step
    t += dt;
    prog_bar.print(t/dt);
  }
}   
