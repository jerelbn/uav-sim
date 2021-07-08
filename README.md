# UAV Simulator
Currently, this is a port of my Matlab simulator into C++. Plotting is still done in Matlab.

## Setup
1. Install Docker - https://docs.docker.com/engine/install/ubuntu/
1. Install Visual Studio Code - https://code.visualstudio.com/download
1. Install local Nvidia drivers
    * Change `Line 74` of `Dockerfile` to use the correct driver for your GPU
1. Install Cuda 11.2 on host system, if not installed by Nvidia driver
1. Install nvidia-docker2 - https://github.com/NVIDIA/nvidia-docker

## Build and Run
1. Create `build` folder in root directory
1. Run `cd build` to navigate to `build` folder
1. Run `cmake ..` followed by `make -j $(nproc)`
1. Run any of the executables now located in `<root>/build`
