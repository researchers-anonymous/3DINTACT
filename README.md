![](https://github.com/edisonslightbulbs/traceless/blob/cluster-apps/doc/figures/README_illustration.png)

# 3DINTACT
#### An open-source CXX_11 project for segmenting interaction regions on tabletop surfaces near real-time

##### `Overview:`


|   Platform |   Hardware	|  Dependencies 	|
|---	|---	|---	|
|   :white_square_button: Linux	|   :white_square_button: Azure Kinect 	| :white_square_button: [ gflags](https://github.com/gflags/gflags)	|
|| |  :white_square_button: [ glog ](https://github.com/google/glog)  	|
|| |  :white_square_button: [ Eigen 3.3 ](https://gitlab.com/libeigen/eigen.git) |
||| :white_square_button:  [ Azure Kinect SDK ](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) |
||| :white_square_button:  [ opencv ](https://github.com/opencv/opencv) |
||| :white_square_button:  [ Pangolin ](https://github.com/stevenlovegrove/Pangolin) |
||| :white_square_button:  [ pytorch ](https://github.com/pytorch/pytorch) |

##### `[1/3] Getting started`

***

Clone this repository using `--recurse-submodules` flag.

If already cloned without the `--recurse-submodules` flag,  use `git submodule update --init --recursive` to initialize the submodules.

##### `[2/3] Installing the dependencies`

***

Make sure to install all the dependencies listed in the table.

###### Caveat (for developers using the AzureKinect)

Microsoft's Azure Kinect has a ceremonious list dependencies. Be sure to install all those as well. Here are a few steps to give you a headstart.

###### 1. The depth engine

A dated [Depth Engine](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md) binary is provided and can be installed by running this [`install_depthengine.sh`](./scripts/) helper script. Please follow [ this hyperlink ](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md) for the official step-by-step on how to get the most up-to-date depth engine.

###### 2. The USB rules

For convenience, the USB rules can be installed by running this [`install_usb_rules.sh`](./scripts/) helper script.

###### 3. Shared system libraries

For convenience, run this [`install_kinect_sdk_dependencies.sh`](./scripts/) helper script to install the Kinect dependencies.

##### `[3/3] Building the project`

***

1) from the project directory

```bash
mkdir build && cd build || return
cmake ..
make
```

2) from the project directory

```bash
# change to project binary directory
cd build/bin

# run target
./3DINTACToolkit

# to stdout logs [ optional ]
# ./3DINTACToolkit --logtostderr=1
```

## Notes
This project uses Microsoft's Azure Kinect to form a concrete example. The point cloud can be [adapted](/doc/README.md).
The project documentation is underway. In the meantime, please feel free to request support and submit issues and any feature requests.

* check out an illustration of what this project offers over on [YouTube](https://youtu.be/E0CcBCk7rjU)

* cite as:
```tex
@misc{3DINTACToolkit2021,
author = {Anonymous},
title = {{3DINTACT: an open-source CXX{\_}11 project for segmenting interaction regions on tabletop surfaces near real-time}},
url = {https://github.com/researchers-anonymous/3DINTACT},
year = {2021}
}
```
