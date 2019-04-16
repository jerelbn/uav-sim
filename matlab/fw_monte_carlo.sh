#!/bin/bash

directory=/tmp/fw_monte_carlo

for value in {1..3}
do
  # Run the UAV simulator
  echo Run \#$value
  ../build/uav_sim

  # Create directory to store data from all runs
  if [ ! -d "$directory" ]; then
    mkdir $directory
    echo Created directory $directory
  fi  

  # Move needed log files in to storage directory with run number label
  mv /tmp/wing1_true_state.log $directory/truth_$value.log
  mv /tmp/wing1_commanded_state.log $directory/command_$value.log
done

