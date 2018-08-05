#include "common_cpp/common.h"
#include "quadrotor.h"
#include "environment.h"


void init(const std::string filename)
{
  // Load directory name
  std::string directory;
  common::get_yaml_node("log_directory", filename, directory);

  // Create logs directory if it doesn't exist
  if(!std::experimental::filesystem::exists(directory))
  {
    if (std::experimental::filesystem::create_directory(directory))
      std::cout << "*** Created logs/ directory! ***\n";
  }

  // Use random seed if desired
  bool use_random_seed;
  common::get_yaml_node("use_random_seed", filename, use_random_seed);
  if (use_random_seed)
    std::srand((unsigned)std::time(NULL));
}



int main()
{
  // Load simulation parameters
  std::string param_file = "../params/params.yaml";
  init(param_file);

  double t(0), tf, dt;
  common::get_yaml_node("tf", param_file, tf);
  common::get_yaml_node("dt", param_file, dt);

  common::ProgressBar prog_bar;
  prog_bar.init(tf/dt,40);

  // Create environment
  environment::Environment env(param_file);

  // Create vehicles
  quadrotor::Quadrotor quad1(param_file);

  // Store initial vehicle states in environment

  // Main simulation loop
  while (t <= tf)
  {
    // Run each vehicle
    quad1.run(t, env.get_vw(), env.get_points().matrix());

    // Update wind and stored vehicle states in environment
    env.updateWind(t);

    // Increment time step
    t += dt;
    prog_bar.print(t/dt);
  }

  return 0;
}   
