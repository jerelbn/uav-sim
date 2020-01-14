#include "common_cpp/common.h"
#include "common_cpp/progress_bar.h"
#include "environment.h"
#include "sensors.h"
#include "quadrotor.h"
#include "quad_control.h"
#include "gimbal.h"
#include "gmbl_ctrl_pid.h"
#include "gmbl_ekf.h"
#include "bicycle.h"



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

  // Create vehicles, controllers, estimators, sensor packages
  quadrotor::Quadrotor quad1("../params/quadrotor1.yaml", rng);
  quadrotor::Controller quad1_ctrl("../params/quadrotor1.yaml", rng, quad1.name());
  sensors::Sensors quad1_sensors("../params/quadrotor1.yaml", rng, quad1.name());

  gimbal::Gimbal gimbal1("../params/gimbal.yaml", rng);
  gmbl_ctrl_pid::Controller gimbal1_ctrl("../params/gimbal.yaml", gimbal1.name());
  gmbl_ekf::EKF gimbal1_ekf("../params/gimbal.yaml", gimbal1.name());
  sensors::Sensors gimbal1_sensors("../params/gimbal.yaml", rng, gimbal1.name());

  bicycle::Bicycle bike1("../params/bicycle1.yaml", env, rng, 1);

  // Store initial vehicle positions in Environment class
  env.insertVehicle(quad1.name(), quad1.x().p);
  env.insertVehicle(bike1.name(), bike1.x().p);

  // Main simulation loop
  while (t <= tf)
  {
    // Update wind and stored vehicle positions in Environment class
    env.updateWind(t);
    env.updateVehicle(quad1.name(), quad1.x().p);
    env.updateVehicle(bike1.name(), bike1.x().p);
    
    // Update vehicles, controllers, sensors, estimators
    quad1.propagate(t, quad1_ctrl.u_, env.getWindVel());
    quad1_ctrl.computeControl(quad1.x(), t, env.getVehiclePosition(bike1.name()));
    quad1.updateAccelerations(quad1_ctrl.u_, env.getWindVel());
    quad1_sensors.updateMeasurements(t, quad1.x(), env);

    gimbal1.propagate(t, gimbal1_ctrl.u(), quad1.x().q);
    gimbal1_ctrl.computeControl(t, quad1.x(), gimbal1_ekf.stateRelToBody(quad1.x()), common::e1);
    gimbal1.updateAccelerations(gimbal1_ctrl.u());
    gimbal1_sensors.updateMeasurements(t, quad1.x(), gimbal1.x(), env);
    gimbal1_ekf.run(t, gimbal1_sensors, quad1_sensors, gimbal1_sensors.qEnc(), gimbal1.x(), quad1.x());

    bike1.run(t, env);
    
    // Update time step
    t += dt;
    prog_bar.print(t/dt);
  }
  prog_bar.finished();

  return 0;
}   
