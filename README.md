![](https://github.com/edisonslightbulbs/traceless/blob/develop/doc/figures/README_illustration.png)

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

###### 2. The USB rules and shared system libraries

For convenience, the USB rules can be installed by running [`install_usb_rules.sh`](./scripts/), and shared system libs using  [`install_kinect_sdk_dependencies.sh`](./scripts/). We recommend using the most [up-to-date instructions](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md).

##### `[3/3] Building the project`

***

1) from the project directory

```bash
mkdir build && cd build || return
cmake ..
make
```

2) see `build/bin` directory for built examples and applications

* CAVEAT: use `--logtostderr=1` to see output logs in STDOUT. E.g, run the chromakey exdample as follows.

```bash
cd build/bin
chromakey --logtostderr=1
```


## Notes
This project uses Microsoft's Azure Kinect to form a concrete example. The point cloud can be [adapted](/doc/README.md).
The project documentation is underway. In the meantime, we offer support and welcome issues and feature requests. You can check out an illustration of segmenting a surface in real-time over on [YouTube](https://www.youtube.com/watch?v=wdg6U8jZmpU&ab_channel=edisonslightbulbs).

* if the project is helpful with your work (or any part of it),  citing is one way of letting us know we are doing some good in this world:

```tex
@misc{3DINTACToolkit2021,
author = {Anonymous},
title = {{3DINTACT: an open-source CXX{\_}11 project for segmenting interaction regions on tabletop surfaces near real-time}},
url = {https://github.com/researchers-anonymous/3DINTACT},
year = {2021}
}
```

* if you find the application of how to detect mobile device on surfaces helpful, and put it to some good use, you can also cite the work as:

```tex
@misc{Traceless2021,
author = {Anonymous},
title = {{Traceless: an open-source CXX{\}_11 project for seamless communication between augmented interactive surfaces and personal mobile devices}},
url = {https://github.com/researchers-anonymous/3DINTACT},
year = {2021}
}
```
