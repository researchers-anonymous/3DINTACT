![](https://github.com/edisonslightbulbs/INTACToolkit/blob/main/doc/figures/concept.png)

## INTACT: Toolkit for fast segmentation of tabletop interaction context

Fast segmentation for tabletop interaction context. This project uses
Microsoft's Azure kinect but should be adaptable for any other development kit
which facilitates point clouds. One critical assumption is that sensor is
mounted overhead as illustrated.


### Overview:

|   Platform |   Hardware	|  Dependencies 	|
|---	|---	|---	|
|   :white_square_button: Linux	|   :white_square_button: Azure Kinect 	| :white_square_button: [ gflags](https://github.com/gflags/gflags)	|
|| |  :white_square_button: [ glog ](https://github.com/google/glog)  	|
|| |  :white_square_button: [ Eigen ](https://gitlab.com/libeigen/eigen.git) |
||| :white_square_button:  [ Azure Kinect SDK ](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) |
||| :white_square_button:  [ opencv ](https://github.com/opencv/opencv) |

### Project structure
```
.
├── cmake
├── doc
│   └── figures
├── external
│   └── Azure-Kinect-Sensor-SDK
│       ├── cmake
│       ├── docs
│       ├── doxygen
│       ├── examples
│       ├── extern
│       ├── include
│       ├── proposals
│       ├── scripts
│       ├── src
│       ├── tests
│       └── tools
├── libs
│   ├── svd
│   │   ├── include
│   │   └── src
│   ├── geometry
│   │   ├── include
│   │   └── src
│   ├── kinect
│   │   ├── include
│   │   └── src
│   ├── outliers
│   │   ├── include
│   │   └── src
│   ├── segment
│   │   ├── include
│   │   └── src
│   └── utility
│       ├── include
│       └── src
├── output
├── scripts
│   └── setup-azure-kinect
│       └── resources
└── src
```

### [1/4] Getting started

***

```bash
# fork and clone repo  using the --recursive parameter
git clone --recurse-submodules <link to github fork>

# if already cloned without the --recurse-submodules parameter
cd INTACToolkit
git submodule update --init --recursive
```

### [2/4] Installing the dependencies

***

###### 1. The depth engine

A dated [Depth Engine](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md) binary is provided and can be installed by running this [`install-depthengine.sh`](./scripts/setup-azure-kinect/) helper script. Please follow [ this hyperlink ](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md) for the official step-by-step on how to get the most up-to-date depth engine.

###### 2. The USB rules

For convenience, the USB rules can be installed by running this [`install-usb-rules.sh`](./scripts/setup-azure-kinect/) helper script.

###### 3. Shared system libraries

To install the shared system dependencies, run this [`install-project-dependencies.sh`](./scripts/setup-azure-kinect/) helper script.

### [3/4] Developer options in project [`CMakeLists.txt`](https://github.com/edisonslightbulbs/INTACToolkit/blob/main/CMakeLists.txt) file

***

###### option 1

```cpp
# recursively pull-update all project submodules (default behaviour is ON)
option(GIT_SUBMODULE "Check submodules during build" ON)
```

###### option 2

```cpp
# build the Azure-Kinect-Sensor-SDK submodule (default behaviour is ON)
option(K4A_SDK_BUILD "Check submodules during build" ON)
```

###### option 3

```cpp
# run target (default behaviour is OFF)
option(RUN "execute target" OFF)
```

#### [4/4] Building project and executing target

***

1) from the project/root directory

```bash
mkdir build && cd build || return
cmake ..
make
```

2) from the project root directory

```bash
# change to project binary directory
cd build/bin

# run target
./main

# to stdout logs [ optional ]
# ./main --logtostderr=1
```
