#!/bin/bash

start_time=$(date +%s)
run_count=0
until [ $run_count -eq 100 ]
do
  # Increment run counter
  let "run_count++"
  
  # Run the UAV simulator
  echo "Run #$run_count, Time elapsed: $(($(date +%s) - $start_time)) seconds."
  ../build/uav_sim

  # Move needed log files in to storage directory with run number label
  cp /tmp/quad1_ekf_truth.log /tmp/monte_carlo_pb_vi_ekf_true_$run_count.log
  cp /tmp/quad1_ekf_est.log /tmp/monte_carlo_pb_vi_ekf_est_$run_count.log
  cp /tmp/quad1_ekf_cov.log /tmp/monte_carlo_pb_vi_ekf_cov_$run_count.log
done
end_time=$(date +%s)
echo "Monte Carlo simulation completed in $(($end_time - $start_time)) seconds."
