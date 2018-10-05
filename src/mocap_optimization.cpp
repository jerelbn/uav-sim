#include "common_cpp/common.h"
#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;


int main()
{
  // Load Mocap and IMU data
  long acc_size, gyro_size, mocap_size;
  double* acc_ptr = common::load_binary<double>("../logs/accel.bin", acc_size);
  double* gyro_ptr = common::load_binary<double>("../logs/gyro.bin", gyro_size);
  double* mocap_ptr = common::load_binary<double>("../logs/mocap.bin", mocap_size);

  // Map into Eigen arrays with column for each time step
  Map<MatrixXd> acc(acc_ptr,10,acc_size/10);
  Map<MatrixXd> gyro(gyro_ptr,10,gyro_size/10);
  Map<MatrixXd> mocap(mocap_ptr,21,mocap_size/21);

  //

  // Clean up
  delete[] acc_ptr, gyro_ptr, mocap_ptr;

  return 0;
}
