#!/bin/bash

run_count=0
until [ $run_count -eq 100 ]
do
  # Increment run counter
  let "run_count++"
  
  # Run the UAV simulator
  echo Run \#$run_count
  ../build/uav_sim

  # Move needed log files in to storage directory with run number label
  cp /tmp/wing1_true_state.log /tmp/fw_monte_carlo_truth_$run_count.log
done

