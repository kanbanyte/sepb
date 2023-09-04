<!-- TOC ignore:true -->
# BUILD STEPS
**Table of Contents**
<!-- TOC -->

* [Installations](#installations)
* [Build](#build)
* [Debug](#debug)

<!-- /TOC -->

## Installations
The following software needs to be installed:
* CMake
* PyTorch C++ distribution

## Build
From the `cpp` directory, run the following commands:
```bash
# Make a 'build' directory and navigate there
mkdir build
cd build

# Configure and build the file in Release config
cmake -S ../ -DCMAKE_PREFIX_PATH=`python -c 'import torch;print(torch.utils.cmake_prefix_path)'`
cmake --build . --config Release
```

## Debug
...
