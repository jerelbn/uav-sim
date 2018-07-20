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

int main()
{
  // do stuff
}   
