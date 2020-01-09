#include "common_cpp/common.h"
#include "common_cpp/progress_bar.h"
#include "quadrotor.h"
#include "bicycle.h"
#include "environment.h"



int main()
{
  int seed;
  double t(0), tf, dt;
  common::get_yaml_node("seed", "../params/sim.yaml", seed);
  common::get_yaml_node("tf", "../params/sim.yaml", tf);
  common::get_yaml_node("dt", "../params/sim.yaml", dt);
  if (seed < 0)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine rng(seed);
  std::srand(seed);

  // Create progress bar
  common::ProgressBar prog_bar;
  prog_bar.init(tf/dt,40);

  // Create environment
  environment::Environment env("../params/sim.yaml", rng);

  // Create vehicles -- last input is vehicle ID
  quadrotor::Quadrotor quad1("../params/quadrotor1.yaml", env, rng, 0);
  bicycle::Bicycle bike1("../params/bicycle1.yaml", env, rng, 1);

  // Store initial vehicle positions in environment
  env.initVehicle(quad1.getState().p, quad1.id_);
  env.initVehicle(bike1.getState().p, bike1.id_);

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
