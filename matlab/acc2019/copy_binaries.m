% This script copies binaries from the logs to the binaries folder
bearings_only = true;
if bearings_only == true
    !cp ../../logs/true_state.bin binaries/aircraft_bo.bin
    !cp ../../logs/target.bin binaries/relative_estimates_bo.bin
    !cp ../../logs/command.bin binaries/command_bo.bin
else
    !cp ../../logs/true_state.bin binaries/aircraft_rb.bin
    !cp ../../logs/target.bin binaries/relative_estimates_rb.bin
    !cp ../../logs/command.bin binaries/command_rb.bin
end
!cp ../../logs/bicycle_true_state.bin binaries/target.bin