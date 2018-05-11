# qflight_ros

This is a meta-repo containing various Qualcomm Flight ROS packages as
git submodules along with instructions for performing demos using the packages.

## Table of Contents

1. [Pre-requisites](#pre-requisites)
1. [Clone and build the code on the robot](#clone-and-build-the-code-on-the-robot)
1. [Clone and build the code on the host computer](#clone-and-build-the-code-on-the-host-computer)
1. [Demos](#demos)
1. [Additional Resources](#additional-resources)

## Pre-requisites

### Hardware

These demos require the following hardware:

* [Qualcomm Flight Kit](https://shop.intrinsyc.com/collections/product-development-kits/products/qualcomm-snapdragon-flight-sbc)
* [Qualcomm Electronic Speed Control (ESC) board](https://shop.intrinsyc.com/collections/dragonboard-accessories/products/qualcomm-electronic-speed-control-board)
* Drone Frame, Motors, and Props; such as the [Dragon Drone Development Kit](https://worldsway.com/product/dragon-drone-development-kit/)

Note that if you're using the Dragon DDK, a URDF ros package is available at: [qflight_descriptions](https://github.com/ATLFlight/qflight_descriptions)

### Software

These demos require the following software:

* [Plaform Image from Intrynsic 3.1.3.1](https://support.intrinsyc.com/attachments/download/1597/Flight_3.1.3.1_JFlash.zip)
* [Platform Addon from Intrynsic 3.1.3.1](https://support.intrinsyc.com/attachments/download/1571/Flight_3.1.3.1_qcom_flight_controller_hexagon_sdk_add_on.zip)
* [Qualcomm Navigator Flight Controller SDK 1.2.58](https://developer.qualcomm.com/download/snapdragon-flight/navigator-controller-v1.2.58)
* [Qualcomm Machine Vision SDK 1.1.8](https://developer.qualcomm.com/download/machine-vision/machine-vision-sdk-v1.1.8.deb)
* [ROS on target](https://github.com/ATLFlight/ATLFlightDocs/blob/master/SnapdragonROSInstallation.md)
* [Qualcomm ESC Firmware 1.2.0](https://developer.qualcomm.com/download/snapdragon-flight/navigator-controller-esc-firmware-v1.2.0)

## Clone and build the code on the robot

These instructions assume that the Qualcomm Flight board has an internet
connection.

### Setup ROS workspace

ADB or SSH into the target and run the following commands to setup a ROS
catkin workspace:

```bash
# run these commands on the flight board
mkdir -p /home/linaro/qflight_ws/src
cd /home/linaro/qflight_ws/src
catkin_init_workspace
cd ..
catkin_make
echo "source /home/linaro/qflight_ws/devel/setup.bash" >> /home/linaro/.bashrc
```

### Clone the repo

```bash
# run these commands on the flight board
source /home/linaro/.bashrc
roscd
cd ../src
git clone https://github.com/ATLFlight/qflight_ros.git
cd qflight_ros
git submodule update --init --recursive
```

### Install ROS dependencies

```bash
# run these commands on the flight board
cd /home/linaro/qflight_ws/src/qflight_ros
rosdep install --from-paths src --skip-keys snav_msgs
```

### Build the code

```bash
# run these commands on the flight board
cd /home/linaro/qflight_ws
catkin_make
```

## Clone and build the code on the host computer

These instructions assume that ROS is installed on the host computer.

### Setup ROS workspace

```bash
# run these commands on the host computer
mkdir -p /home/$USER/qflight_ws/src
cd /home/$USER/qflight_ws/src
catkin_init_workspace
cd ..
catkin_make
echo "source /home/$USER/qflight_ws/devel/setup.bash" >> /home/$USER/.bashrc
```

### Clone the repo

```bash
# run these commands on the host computer
source /home/$USER/.bashrc
roscd
cd ../src
git clone https://github.com/ATLFlight/qflight_ros.git
cd qflight_ros
git checkout host
git submodule update --init --recursive
```

### Build the code

```bash
# run these commands on the host computer
cd /home/$USER/qflight_ws
catkin_make
```

## Demos

1. [Trajectory Generation and Tracking Demo](demos/traj_follow_demo.md)

## Addition Resources

Be sure to check out the README files in each repo to find more documentation
about each package.

