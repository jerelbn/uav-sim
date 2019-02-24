#include <experimental/filesystem>
#include "common_cpp/common.h"
#include "common_cpp/progress_bar.h"
#include "quadrotor.h"
#include "bicycle.h"
#include "environment.h"



int main()
{
  std::string param_file = "../params/params.yaml";

  bool use_random_seed;
  double t(0), tf, dt;
  common::get_yaml_node("use_random_seed", param_file, use_random_seed);
  common::get_yaml_node("tf", param_file, tf);
  common::get_yaml_node("dt", param_file, dt);
  if (use_random_seed)
    std::srand((unsigned)std::time(NULL));

  // Create progress bar
  common::ProgressBar prog_bar;
  prog_bar.init(tf/dt,40);

  // Create environment
  environment::Environment env(param_file);

  // Create vehicles
  quadrotor::Quadrotor quad1(param_file, env, 0);
  bicycle::Bicycle bike1(param_file, env, 1);

  // Store initial vehicle positions in environment
  env.initVehicle(quad1.getState().p, quad1.id_);
  env.initVehicle(bike1.getState().p, quad1.id_);

  // Main simulation loop
  while (t <= tf)
  {
    t += dt;
    prog_bar.print(t/dt);

    // Run each vehicle
    quad1.run(t, env);
    bike1.run(t, env);

    // Update wind and stored vehicle positions in environment
    env.updateWind(t);
    env.updateVehicle(quad1.getState().p, quad1.id_);
    env.updateVehicle(bike1.getState().p, bike1.id_);
  }
  prog_bar.finished();

  return 0;
}   
